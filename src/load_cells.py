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

class LoadCells:
    def __init__(self, COM: str, baud: int = 115200, sim: bool = False, timeout: float = 60.0):
        self.sim = sim
        self.timeout = timeout

        if self.sim:
            logging.info("Simulated connection to load cell board established.")

        else:
            logging.info("Configuring load cell board serial port..")
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

            logging.info("Attempting to open load cell board serial port..")

            if self.ser.isOpen() is False:
                self.ser.open()
            else:
                self.ser.close()
                time.sleep(0.05)
                self.ser.open()

            # Give time for controller to wake up
            time.sleep(2)

            # Check connection (blocking)
            self.check_response()
            logging.info("Serial connection to load cell board established.")

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
        while True:
            data = self.get_data()

            if data.startswith("ESP-ROM:"):
                continue

            if '#' in data:
                return
            if "Unknown command" in data:
                raise RuntimeError("Controller board failed to recognise command: " + data)

            logging.info(data)

    @skip_if_sim()
    def close_ser(self) -> None:
        logging.info("Closing serial connection to load cell board.")
        if self.ser.isOpen():
            self.ser.close()

    @skip_if_sim(default_return = True)
    def check_status(self) -> bool:
        self.ser.write("statusCheck()".encode())
        self.check_response()

    @skip_if_sim(default_return = 0)
    def get_cell_weight(self, cell_no: int) -> None:
        self.ser.write(f"readCell({cell_no})".encode())
        return float(self.get_data())
    
    @skip_if_sim()
    def tare_cell(self, cell_no: int) -> None:
        self.ser.write(f"tareCell({cell_no})".encode())
        self.check_response()

    @skip_if_sim()
    def calibrate_cell(self, cell_no: int) -> None:
        self.ser.write(f"calibrateCell({cell_no})".encode())
        self.check_response()

    @skip_if_sim()
    def change_calibration_weight(self, weight_kg: float) -> None:
        self.ser.write(f"changeCalWeight({weight_kg})".encode())
        self.check_response()
        
    @skip_if_sim()
    def tare_all(self) -> None:
        self.ser.write("tareAll()".encode())
        self.check_response()

       
        
        
    