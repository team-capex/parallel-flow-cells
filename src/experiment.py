
"""
experiment.py

YAML-driven scheduler for a 4-cell electrochemical flow-cell system controlled by 3 serial PCBs:

- PCB #1: Load cells (4 channels) - read container mass.
- PCB #2: MFC + 4 stepper pumps + 6 valves (4 used) - CO2 to all cells, electrolyte per cell, divert valve per cell.
- PCB #3: 4 PWM peristaltic pumps - top up each cell's container when volume is low.

This file intentionally keeps step semantics explicit and conservative.
If anything fails, it attempts to shut off CO2 and stop PWM pumps before exiting.

Usage:
  python experiment.py --config config.yaml
  python experiment.py --config config.yaml --sim           # no serial I/O (uses sim=True in board classes)
"""

from __future__ import annotations

import argparse
import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

from load_cells import LoadCells
from mfc_board import mfc_with_steppers
from pump_board import pump_controller


log = logging.getLogger("experiment")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")


# ----------------------------
# Utility helpers
# ----------------------------

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def now_s() -> float:
    return time.time()

def sleep_with_heartbeat(seconds: float, heartbeat_s: float = 10.0, msg: str = "") -> None:
    """Sleep while emitting periodic log messages (helps during long steps)."""
    end = now_s() + max(0.0, seconds)
    while True:
        remaining = end - now_s()
        if remaining <= 0:
            return
        chunk = min(heartbeat_s, remaining)
        if msg:
            log.info("%s (%.1f s remaining)", msg, remaining)
        time.sleep(chunk)


# ----------------------------
# Config schema (minimal)
# ----------------------------

@dataclass
class SerialCfg:
    port: str
    baud: int

@dataclass
class TopUpCfg:
    density_g_per_ml: float
    target_volume_ml: float
    min_volume_ml: float
    pump_pwm: float
    pump_rate_ml_per_s: float
    max_pump_s_per_cell: float
    recheck_after_pump_s: float
    max_attempts: int

@dataclass
class SystemCfg:
    cells: List[int]
    valves: Dict[int, int]   # cell -> valve_no

@dataclass
class DefaultsCfg:
    electrolyte_stepper_flow_rate_ml_per_s: float

@dataclass
class Step:
    name: str
    type: str
    raw: Dict[str, Any]


def load_config(path: Path) -> Dict[str, Any]:
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError("config.yaml root must be a mapping")
    return data


# ----------------------------
# Experiment runner
# ----------------------------

class ExperimentRunner:
    def __init__(self, cfg: Dict[str, Any], sim: bool = False):
        self.cfg = cfg
        self.sim = sim

        serial_cfg = cfg["serial"]
        self.serial_load = SerialCfg(**serial_cfg["load_cell_board"])
        self.serial_mfc = SerialCfg(**serial_cfg["mfc_stepper_valve_board"])
        self.serial_pump = SerialCfg(**serial_cfg["pump_board"])

        self.system = SystemCfg(
            cells=list(cfg["system"]["cells"]),
            valves={int(k): int(v) for k, v in cfg["system"]["valves"].items()},
        )

        self.topup = TopUpCfg(**cfg["topup"])
        self.defaults = DefaultsCfg(**cfg.get("defaults", {}))

        self.steps: List[Step] = []
        for s in cfg["experiment"]["steps"]:
            self.steps.append(Step(name=str(s["name"]), type=str(s["type"]), raw=dict(s)))

        # Boards (constructed in connect())
        self.load_cells: Optional[LoadCells] = None
        self.mfc: Optional[mfc_with_steppers] = None
        self.pumps: Optional[pump_controller] = None

    # ----- lifecycle -----

    def connect(self) -> None:
        log.info("Connecting to boards (sim=%s)...", self.sim)
        self.load_cells = LoadCells(self.serial_load.port, self.serial_load.baud, sim=self.sim)
        self.mfc = mfc_with_steppers(self.serial_mfc.port, self.serial_mfc.baud, sim=self.sim)
        self.pumps = pump_controller(self.serial_pump.port, self.serial_pump.baud, sim=self.sim)

    def close(self) -> None:
        log.info("Closing board connections...")
        for b in (self.load_cells, self.mfc, self.pumps):
            try:
                if b is not None:
                    b.close_ser()
            except Exception as e:
                log.warning("Close failed: %s", e)

    # ----- safety -----

    def safe_shutdown(self) -> None:
        """
        Best-effort safe state:
          - CO2 off
          - close all divert valves (OFF => to waste)
          - stop all PWM pumps
        """
        log.warning("Attempting safe shutdown...")
        try:
            if self.mfc:
                # Close all used valves
                for cell, valve_no in self.system.valves.items():
                    try:
                        self.mfc.close_valve(valve_no)
                    except Exception:
                        pass
                # CO2 off
                try:
                    self.mfc.mfc_set_flow(0.0)
                except Exception:
                    pass
                try:
                    self.mfc.mfc_off()
                except Exception:
                    pass
        except Exception:
            pass

        try:
            if self.pumps:
                for cell in self.system.cells:
                    try:
                        self.pumps.pwm_pump(cell, 0.0)
                    except Exception:
                        pass
        except Exception:
            pass

    # ----- step implementations -----

    def _set_divert(self, cell: Optional[int]) -> None:
        """
        Ensure ONLY one divert valve is ON (or none).
        OFF means the CO2 outlet goes to waste manifold.
        """
        assert self.mfc is not None
        # Close all in-use valves first
        for c in self.system.cells:
            valve_no = self.system.valves[c]
            self.mfc.close_valve(valve_no)

        if cell is None:
            log.info("Divert: none (all valves OFF => waste)")
            return

        if cell not in self.system.valves:
            raise ValueError(f"Invalid cell for divert: {cell}")
        valve_no = self.system.valves[cell]
        log.info("Divert: cell %d via valve %d", cell, valve_no)
        self.mfc.open_valve(valve_no)

    def _flow_step(self, s: Step) -> None:
        """
        Start/maintain CO2 and electrolyte flow for a duration.
        Electrolyte "flow rate" is implemented as repeated stepper dosing chunks.
        """
        assert self.mfc is not None

        duration_s = float(s.raw["duration_s"])
        co2_flow_sccm = float(s.raw["co2_flow_sccm"])
        divert_cell = s.raw.get("divert_cell", None)
        if divert_cell is not None:
            divert_cell = int(divert_cell)

        elec_rate = float(s.raw.get(
            "electrolyte_flow_rate_ml_per_s",
            self.defaults.electrolyte_stepper_flow_rate_ml_per_s
        ))
        # How often to send stepper dosing commands (keeps volumes smaller and avoids huge moves)
        chunk_s = float(s.raw.get("electrolyte_chunk_s", 30.0))
        chunk_s = clamp(chunk_s, 1.0, duration_s)

        log.info("CO2: ON, set flow=%.2f sccm", co2_flow_sccm)
        self.mfc.mfc_on()
        self.mfc.mfc_set_flow(co2_flow_sccm)

        self._set_divert(divert_cell)

        # Implement electrolyte flow as repeated multiStepperPump calls
        log.info("Electrolyte: %.3f ml/s per cell for %.1f s (chunk %.1f s)", elec_rate, duration_s, chunk_s)

        t_end = now_s() + duration_s
        while now_s() < t_end:
            remaining = t_end - now_s()
            this_chunk = min(chunk_s, remaining)
            ml = elec_rate * this_chunk

            # NOTE: mfc_board.multi_pump takes ml per cell and a flow_rate parameter.
            # We pass flow_rate=elec_rate (ml/s). Board firmware must interpret this consistently.
            self.mfc.multi_pump([ml, ml, ml, ml], flow_rate=elec_rate)

            # The board likely blocks until pumping is complete. Still, keep a tiny guard sleep.
            time.sleep(0.05)

        log.info("Flow step complete.")

    def _topup_step(self, s: Step) -> None:
        """
        Check each cell's load cell volume estimate and top up with PWM pump if below min_volume_ml.
        """
        assert self.load_cells is not None
        assert self.pumps is not None

        density = float(s.raw.get("density_g_per_ml", self.topup.density_g_per_ml))
        target_ml = float(s.raw.get("target_volume_ml", self.topup.target_volume_ml))
        min_ml = float(s.raw.get("min_volume_ml", self.topup.min_volume_ml))
        pwm = float(s.raw.get("pump_pwm", self.topup.pump_pwm))
        pump_rate = float(s.raw.get("pump_rate_ml_per_s", self.topup.pump_rate_ml_per_s))
        max_pump_s = float(s.raw.get("max_pump_s_per_cell", self.topup.max_pump_s_per_cell))
        recheck_s = float(s.raw.get("recheck_after_pump_s", self.topup.recheck_after_pump_s))
        max_attempts = int(s.raw.get("max_attempts", self.topup.max_attempts))

        log.info("Top-up check: density=%.3f g/ml, target=%.1f ml, min=%.1f ml", density, target_ml, min_ml)

        for cell in self.system.cells:
            for attempt in range(1, max_attempts + 1):
                mass = float(self.load_cells.get_cell_weight(cell))  # assumed grams
                vol_ml = mass / density

                log.info("Cell %d: mass=%.2f g => %.1f ml (attempt %d/%d)", cell, mass, vol_ml, attempt, max_attempts)

                if vol_ml >= min_ml:
                    break  # ok

                missing_ml = max(0.0, target_ml - vol_ml)
                pump_s = missing_ml / max(1e-6, pump_rate)
                pump_s = clamp(pump_s, 0.5, max_pump_s)

                log.warning("Cell %d LOW: pumping at PWM=%.2f for %.1f s (~%.1f ml)", cell, pwm, pump_s, pump_s * pump_rate)
                self.pumps.pwm_pump(cell, pwm)
                time.sleep(pump_s)
                self.pumps.pwm_pump(cell, 0.0)

                time.sleep(recheck_s)

            else:
                log.error("Cell %d still low after %d attempts.", cell, max_attempts)

        log.info("Top-up step complete.")

    def _measure_step(self, s: Step) -> None:
        """
        Placeholder for potentiostat control. Currently just waits.
        """
        duration_s = float(s.raw["duration_s"])
        note = s.raw.get("note", "")
        if note:
            log.info("Measure placeholder: %s", note)
        sleep_with_heartbeat(duration_s, heartbeat_s=30.0, msg="Measurement placeholder running")

    def _divert_step(self, s: Step) -> None:
        """
        Divert one cell's CO2 outlet to analyzer for a duration, then revert to waste.
        """
        assert self.mfc is not None
        cell = s.raw.get("cell", None)
        cell = None if cell is None else int(cell)
        duration_s = float(s.raw.get("duration_s", 0.0))

        self._set_divert(cell)
        if duration_s > 0:
            sleep_with_heartbeat(duration_s, heartbeat_s=10.0, msg=f"Diverting cell {cell} to analyzer")
        # revert
        self._set_divert(None)

    def _co2_off_step(self, s: Step) -> None:
        assert self.mfc is not None
        log.info("CO2: OFF")
        # best effort: set flow to zero then off
        try:
            self.mfc.mfc_set_flow(0.0)
        except Exception:
            pass
        self.mfc.mfc_off()
        # ensure all divert valves are off (waste)
        self._set_divert(None)

    def _deprime_step(self, s: Step) -> None:
        """
        Reverse electrolyte flow to de-prime.
        This assumes the stepper firmware supports negative ml to reverse.
        If your firmware uses a separate command or flag, change here.
        """
        assert self.mfc is not None

        duration_s = float(s.raw["duration_s"])
        elec_rate = float(s.raw.get(
            "electrolyte_flow_rate_ml_per_s",
            self.defaults.electrolyte_stepper_flow_rate_ml_per_s
        ))
        chunk_s = float(s.raw.get("electrolyte_chunk_s", 30.0))
        chunk_s = clamp(chunk_s, 1.0, duration_s)

        log.info("Deprime: reversing electrolyte at %.3f ml/s per cell for %.1f s", elec_rate, duration_s)

        t_end = now_s() + duration_s
        while now_s() < t_end:
            remaining = t_end - now_s()
            this_chunk = min(chunk_s, remaining)
            ml = -elec_rate * this_chunk  # negative => reverse

            self.mfc.multi_pump([ml, ml, ml, ml], flow_rate=abs(elec_rate))
            time.sleep(0.05)

        log.info("Deprime complete.")

    # ----- execution -----

    def run(self) -> None:
        log.info("Experiment: %s", self.cfg["experiment"].get("name", "unnamed"))
        for idx, s in enumerate(self.steps, start=1):
            log.info("---- Step %d/%d: %s (%s) ----", idx, len(self.steps), s.name, s.type)
            if s.type == "flow":
                self._flow_step(s)
            elif s.type == "topup":
                self._topup_step(s)
            elif s.type == "measure":
                self._measure_step(s)
            elif s.type == "divert":
                self._divert_step(s)
            elif s.type == "co2_off":
                self._co2_off_step(s)
            elif s.type == "deprime":
                self._deprime_step(s)
            else:
                raise ValueError(f"Unknown step type: {s.type}")

        log.info("Experiment complete.")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, help="Path to config.yaml")
    ap.add_argument("--sim", action="store_true", help="Run without serial IO (sim=True in board classes)")
    args = ap.parse_args()

    cfg = load_config(Path(args.config))

    runner = ExperimentRunner(cfg, sim=args.sim)
    runner.connect()

    try:
        runner.run()
    except KeyboardInterrupt:
        log.warning("Interrupted by user.")
        runner.safe_shutdown()
        return 130
    except Exception as e:
        log.exception("Experiment failed: %s", e)
        runner.safe_shutdown()
        return 1
    finally:
        runner.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
