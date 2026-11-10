# CT6056 Python GUI for Serial Interfacing with the Micro Arm
# Please install the below using pip:
# If pip doesn't work make sure you have python and check the following link: https://www.geeksforgeeks.org/how-to-install-pip-on-windows/ or https://pip.pypa.io/en/stable/installation/
# pip install PyQt5 pyserial

import sys
import serial
import time
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt

class ArmControlGUI(QWidget):
    def __init__(self, serial_port):
        super().__init__()
        
        # Initialize serial communication
        self.ser = serial.Serial(serial_port, 115200, timeout=1) # Change the baudrate accordingly
        time.sleep(2)  # Wait for the serial connection to initialize
        
        # Servo control layout
        self.initUI()

    def initUI(self):
        # Main layout for the window
        main_layout = QVBoxLayout()

        # Add sliders and labels for each servo
        self.servo_sliders = []
        self.servo_labels = []
        
        for i in range(5):
            servo_label = QLabel(f'Servo {i}: 90', self)  # Initial label for each servo
            self.servo_labels.append(servo_label)
            
            # Slider for each servo
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(180)
            slider.setValue(90)
            slider.valueChanged.connect(lambda value, index=i: self.update_servo(index, value))
            self.servo_sliders.append(slider)

            # Horizontal layout to hold label and slider
            h_layout = QHBoxLayout()
            h_layout.addWidget(servo_label)
            h_layout.addWidget(slider)

            # Add to the main layout
            main_layout.addLayout(h_layout)
        
        # Set the main layout and the title
        self.setLayout(main_layout)
        self.setWindowTitle('Robotic Arm Control')
        self.setGeometry(100, 100, 400, 300)  # Window size

    def update_servo(self, servo_num, angle_value):
        # Update label text with current servo value
        self.servo_labels[servo_num].setText(f'Servo {servo_num}: {angle_value}')

        # Send serial command to control the servo
        command = f'S{servo_num} {angle_value}\n'
        print(f"Sending: {command.strip()}")
        self.ser.write(command.encode())  # Send the command over serial

    def closeEvent(self, event):
        # Close the serial connection when the window is closed
        self.ser.close()
        event.accept()

# Main application loop
if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    # Replace 'COM3' with the correct serial port on your system
    # For Linux, it might be '/dev/ttyUSB0', and for macOS, something like '/dev/cu.usbserial'
    serial_port = 'COM3'
    
    # Initialize the GUI with the serial port
    gui = ArmControlGUI(serial_port)
    gui.show()

    sys.exit(app.exec_())
