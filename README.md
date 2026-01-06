# User Guide for Parallel MEA / H-Cell Setups

## Install Dependencies from PyProject

Build venv in root directory:

```
python -m venv .venv
```

Upgrade pip:

```
.venv/bin/pip install --upgrade pip
```

Install dependencies into new venv:

```
.venv/bin/pip install -e .
```

Activate venv:

```
source .venv/bin/activate
```

Note: Replace *bin* with *Scripts* if using windows.

## Calibrating Load Cells

## Other Requirements

- Platformio vscode extension to flash firmware to PCBs
- RS485 converter for MFC

## Hardware References

See [BOM](part_files/BOM.pdf) for complete list.

1. [1kg load cells](https://www.digikey.dk/da/products/detail/adafruit-industries-llc/4540/12323569?srsltid=AfmBOop3O_A9xKGq612NTgOTVNnTNE8ITZH4t8R0--68j-zy2UcEIYY0)
2. [0.5slm MFC](https://sensirion.com/products/catalog/SFC5500-05slm)
3. [5slm MFC](https://sensirion.com/products/catalog/SFC6000D-5slm)
4. [3/2 Solenoid Valves](https://www.festo.com/dk/en/a/196132/)
5. [9QX Peristaltic Pump](https://www.boxerpumps.com/peristaltic-pumps-for-liquid/9qx/)