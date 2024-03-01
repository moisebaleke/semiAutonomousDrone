import pyrplidar
from pyrplidar import PyRPlidar
import time

class RPLidarTest:
    def __init__(self, port):
        self.lidar = PyRPlidar()
        self.port = port
        self.connected = False

    def connect(self):
        """Connect to the RPLidar"""
        self.lidar.connect(port=self.port, baudrate=115200, timeout=3)
        # Check if the connection is successful
        if self.lidar.get_info() != None:
            self.connected = True
            print("RPLidar connected successfully.")
        else:
            print("Failed to connect to RPLidar.")

    def start_scanning(self):
        """Start the RPLidar motor and scanning"""
        if not self.connected:
            print("RPLidar is not connected.")
            return

        self.lidar.start_motor()
        # Start scanning with the scan mode you prefer
        for scan in self.lidar.iter_scans(max_buf_meas=500):
            print(scan)
            # For demo, break after the first scan, remove this line to continuously get scans
            break

    def stop(self):
        """Stop the RPLidar motor and disconnect"""
        if self.connected:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            self.connected = False
            print("RPLidar disconnected successfully.")

if __name__ == "__main__":
    # Replace '/dev/ttyUSB0' with the correct port for your setup
    lidar_test = RPLidarTest('/dev/ttyUSB0')
    lidar_test.connect()
    if lidar_test.connected:
        lidar_test.start_scanning()
    lidar_test.stop()
