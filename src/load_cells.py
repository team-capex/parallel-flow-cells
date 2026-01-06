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
            if self.check_status():
                logging.info("Serial connection to load cell board established.")

    @skip_if_sim(default_return="0")
    def get_data(self) -> str:
        while self.ser.in_waiting == 0:
            pass

        return self.ser.readline().decode().rstrip().replace("\x00", "")
        
    @skip_if_sim()
    def check_response(self) -> None:
        start = time.time()
        while(time.time() - start < self.timeout):
            data = self.get_data()
            if data is None:
                logging.warning("Timed out waiting for response")
                break
            if '#' in data:
                break
            elif "Unknown command" in data:
                raise RuntimeError("Load cell board failed to recognise command: " + data)
            else:
                logging.info("Response from load cell board: " + data)

    @skip_if_sim()
    def close_ser(self) -> None:
        logging.info("Closing serial connection to load cell board.")
        if self.ser.isOpen():
            self.ser.close()

    @skip_if_sim(default_return = True)
    def check_status(self) -> bool:
        self.ser.write("statusCheck()".encode())
        self.check_response()

    @skip_if_sim(default_return = 25)
    def get_readings(self) -> tuple[float]:
        self.ser.write("readCells()".encode())

        try:
            values = self.get_data().split(",")

            if len(values) != 4:
                raise ValueError("Received invalid message from load cell board")
            
            return float(values[0]), float(values[1]), float(values[2]), float(values[3])

        except Exception as e:
            raise RuntimeError(f"Failed to read load cells: {e}")
        
    @skip_if_sim()
    def tare(self) -> None:
        self.ser.write("tare()".encode())
        self.check_response()

       
        
        
    