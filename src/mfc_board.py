import logging
import time
import serial

logging.basicConfig(level = logging.INFO)

def skip_if_sim(default_return = None):
    def decorator(func):
        def wrapper(self, *args, **kwargs):
            if self.sim:
                return default_return
            return func(self, *args, **kwargs)
        return wrapper
    return decorator

class mfc_with_steppers:
    def __init__(self, COM: str, baud: int = 115200, sim: bool = False, timeout: float = 120.0):
        self.sim = sim
        self.timeout = timeout

        if self.sim:
            logging.info("Simulated connection to controller board established.")

        else:
            logging.info("Configuring controller board serial port..")
            self.ser = serial.Serial(
                port=COM,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )

            self.ser.baudrate = baud
            self.ser.bytesize = 8 
            self.ser.parity = 'N' # No parity
            self.ser.stopbits = 1
            self.ser.timeout = self.timeout

            # Turn ESP ON (active low)
            self.ser.setDTR(False)  # False = inactive = HIGH (because DTR is active-low)
            self.ser.setRTS(False)

            logging.info("Attempting to open controller board serial port..")

            if self.ser.isOpen() is False:
                self.ser.open()
            else:
                self.ser.close()
                time.sleep(0.05)
                self.ser.open()

            # Give time for controller to wake up
            time.sleep(2)

            # Check connection (blocking)
            if self.check_status():
                logging.info("Serial connection to controller board established.")

    @skip_if_sim(default_return="0")
    def get_data(self) -> str:
        start = time.time()

        while(time.time() - start < self.timeout):
            if self.ser.in_waiting > 0:
                break

            time.sleep(0.1)
        
        if self.ser.in_waiting == 0:
             raise RuntimeError("Timed out waiting for response.")
        else:
            return self.ser.readline().decode().rstrip().replace("\x00", "")
        
    @skip_if_sim()
    def check_response(self) -> None:
        data = self.get_data()

        while(1):
            if '#' in data:
                return
            elif "Unknown command" in data:
                raise RuntimeError("Controller board failed to recognise command: " + data)
            else:
                logging.info(data)

    @skip_if_sim()
    def close_ser(self) -> None:
        logging.info("Closing serial connection to controller board.")
        if self.ser.isOpen():
            self.ser.close()

    @skip_if_sim(default_return = True)
    def check_status(self) -> bool:
        self.ser.write("statusCheck()".encode())
        self.check_response()

    @skip_if_sim(default_return = 25)
    def get_temperature(self) -> float:
        self.ser.write("getTemperature()".encode())
        return float(self.get_data())
        
    @skip_if_sim(default_return = 50)
    def get_humidity(self) -> float:
        self.ser.write("getHumidity()".encode())
        return float(self.get_data())
        
    @skip_if_sim()
    def single_pump(self, pump_no: int, ml: float, flow_rate: float = 0.05) -> None:
        self.ser.write(f"singleStepperPump({pump_no},{ml:.3f},{flow_rate:.3f})".encode())
        self.check_response()

    @skip_if_sim()
    def multi_pump(self, ml: list[float], flow_rate: float = 0.05) -> None:
        if len(ml) != 4:
            raise ValueError("Exactly 4 volumes are required")
        
        args = ",".join(f"{float(v):.3f}" for v in ml)
        self.ser.write(f"multiStepperPump({args},{flow_rate:.3f})".encode())
        self.check_response()

    @skip_if_sim()
    def open_valve(self, valve_no: int) -> None:
        self.ser.write(f"valveOpen({valve_no})".encode())
        self.check_response()

    @skip_if_sim()
    def close_valve(self, valve_no: int) -> None:
        self.ser.write(f"valveClose({valve_no})".encode())
        self.check_response()

    @skip_if_sim()
    def mfc_on(self) -> None:
        self.ser.write("mfcOn()".encode())
        self.check_response()

    @skip_if_sim()
    def begin_co2(self) -> None:
        self.ser.write("beginCO2()".encode())
        self.check_response()

    @skip_if_sim()
    def mfc_off(self) -> None:
        self.ser.write("mfcOff()".encode())
        self.check_response()

    @skip_if_sim()
    def mfc_set_flow(self, flow_rate_sccm: float = 0.05) -> None:
        self.ser.write(f"mfcSetFlow({flow_rate_sccm})".encode())
        self.check_response()

    @skip_if_sim()
    def mfc_get_flow(self) -> float:
        self.ser.write("mfcGetFlow()".encode())
        return float(self.get_data())
    
    @skip_if_sim()
    def mfc_emergency_stop(self) -> None:
        self.ser.write("mfcForceClose()".encode())
        self.check_response()
    
    @skip_if_sim()
    def mfc_reset_emergency_close(self) -> None:
        self.ser.write("mfcForceCloseReset()".encode())
        self.check_response()