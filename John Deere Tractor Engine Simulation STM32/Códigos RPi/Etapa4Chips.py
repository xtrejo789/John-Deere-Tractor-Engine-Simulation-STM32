import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import numpy as np
import threading
import time
import serial
import csv
from queue import Queue
import random

# Configuration for the serial port
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200

# Create a Qt application
app = QtWidgets.QApplication([])

# Set the default background to white and the color of the lines to black
pg.setConfigOption('background', 'k')
pg.setConfigOption('foreground', 'w')

control_values = [0, 0, 0, 0]

# Timer for incrementing the 'A' button value
a_button_timer = QtCore.QTimer()

# Create windows for different purposes
win_normal = pg.GraphicsLayoutWidget(show=True, title="Data")
win_normal.resize(1000, 600)
win_normal.setWindowTitle('data')

win_control = QtWidgets.QWidget()
win_control.setWindowTitle('Controls')
win_control.resize(400, 300)
control_layout = QtWidgets.QVBoxLayout(win_control)

# Create 2x2 grid of plots
plot1 = win_normal.addPlot(row=0, col=0, title="Velocidad Motor (RPM)")
curve1 = plot1.plot(pen='y')

plot2 = win_normal.addPlot(row=0, col=1, title="Velocidad Vehiculo (m/s)")
curve2 = plot2.plot(pen='r')

plot3 = win_normal.addPlot(row=1, col=0, title="Marcha")
curve3 = plot3.plot(pen='b')

plot4 = win_normal.addPlot(row=1, col=1, title="Acelerador")
curve4 = plot4.plot(pen='g')

button_down = QtWidgets.QPushButton('Freno')
button_right = QtWidgets.QPushButton('Derecha')
button_center = QtWidgets.QPushButton('Izquierda')

# Define a green button style
button_style = """
QPushButton {
    background-color: red;
    color: white;
    font-size: 14px;
    border: none;
    padding: 10px;
}
QPushButton:pressed {
    background-color: green;
}
"""

button_down.setStyleSheet(button_style)
button_right.setStyleSheet(button_style)
button_center.setStyleSheet(button_style)

# Layout for the buttons
button_layout = QtWidgets.QVBoxLayout()

ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)

# Layout for the cross control
cross_layout = QtWidgets.QGridLayout()
cross_layout.addWidget(button_down, 2, 1)
cross_layout.addWidget(button_right, 1, 2)
cross_layout.addWidget(button_center, 1, 1)

# Add layouts to the control window
control_layout.addLayout(button_layout)
control_layout.addLayout(cross_layout)

# Data collections
data1 = np.zeros(100)
data2 = np.zeros(100)
data3 = np.zeros(100)
data4 = np.zeros(100)
all_data_motor = []
all_data_vehiculo = []
all_data_marcha = []
all_data_acelerador = []
time_data = np.zeros(100)
all_time_data = []

# Queue for communication between threads
data_queue = Queue()
csv_queue = Queue()

# Record the start time
start_time = time.time()

# Open CSV file for writing
csv_file = open('datos.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Time (s)', 'Velocidad Motor (RPM)', 'Velocidad Vehiculo (km/h)', 'Marcha', 'Acelerador'])

# Global flag to control data reading
reading_serial = True

def update():
    while not data_queue.empty():
        line = data_queue.get()
        try:
            velocidad_motor = int(line[1])
            velocidad_vehiculo = int(line[0])
            marcha = int(line[2])
            acelerador = int(line[3])

            current_time = time.time() - start_time  # Time in seconds since start

            time_data[:-1] = time_data[1:]  # Shift data in the array one sample left
            time_data[-1] = current_time  # Add the new time value

            data1[:-1] = data1[1:]  # Shift data in the array one sample left
            data1[-1] = velocidad_motor  # Add the new value

            data2[:-1] = data2[1:]
            data2[-1] = velocidad_vehiculo

            data3[:-1] = data3[1:]
            data3[-1] = marcha

            data4[:-1] = data4[1:]
            data4[-1] = acelerador

            # Append data to the lists for the all data plots
            all_data_motor.append(velocidad_motor)
            all_data_vehiculo.append(velocidad_vehiculo)
            all_data_marcha.append(marcha)
            all_data_acelerador.append(acelerador)
            all_time_data.append(current_time)

            curve1.setData(time_data, data1)
            curve2.setData(time_data, data2)
            curve3.setData(time_data, data3)
            curve4.setData(time_data, data4)

            # Calculate the new y-axis limits with a small margin
            min_y1, max_y1 = np.min(data1), np.max(data1)
            min_y2, max_y2 = np.min(data2), np.max(data2)
            min_y3, max_y3 = np.min(data3), np.max(data3)
            min_y4, max_y4 = np.min(data4), np.max(data4)

            margin1 = (max_y1 - min_y1) * 0.1 if (max_y1 - min_y1) > 0 else 1
            margin2 = (max_y2 - min_y2) * 0.1 if (max_y2 - min_y2) > 0 else 1
            margin3 = (max_y3 - min_y3) * 0.1 if (max_y3 - min_y3) > 0 else 1
            margin4 = (max_y4 - min_y4) * 0.1 if (max_y4 - min_y4) > 0 else 1

            plot1.setYRange(min(0, min_y1 - margin1), max_y1 + margin1)
            plot2.setYRange(min(0, min_y2 - margin2), max_y2 + margin2)
            plot3.setYRange(min(0, min_y3 - margin3), max_y3 + margin3)
            plot4.setYRange(min(0, min_y4 - margin4), max_y4 + margin4)

            csv_queue.put([current_time, velocidad_motor, velocidad_vehiculo, marcha, acelerador])
        except ValueError:
            pass
    
    app.processEvents()  # process the plot events

def read_serial():
    global ser, reading_serial
    try:
        #ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)
        while reading_serial:
            line = ser.readline().decode().strip().split(',')
            if len(line) == 4:
                data_queue.put(line)
            else:
                print(line)
        ser.close()
    except serial.SerialException:
        reading_serial = False

def write_csv():
    while True:
        if not csv_queue.empty():
            csv_writer.writerow(csv_queue.get())
            csv_file.flush()
        time.sleep(0.01)  # Adjust the interval as necessary
        
button_states = {'b': False, 'r': False, 'l': False}


def control_pressed(button):
    global control_values, reading_serial, button_states
    button_states[button] = True
    try:
        if button == 'b':
            ser.write(chr(50).encode())  # Enviar los valores de control
            print(chr(50).encode())
        elif button == 'r':
            ser.write(chr(51).encode())  # Enviar los valores de control
            print(chr(51).encode())
        elif button == 'l':
            ser.write(chr(49).encode())  # Enviar los valores de control
            print(chr(49).encode())
    except Exception as e:
        print(f"An error occurred: {e}")


def main():
    button_down.pressed.connect(lambda: control_pressed('b')) #freno
    button_right.pressed.connect(lambda: control_pressed('r')) #derecha
    button_center.pressed.connect(lambda: control_pressed('l')) #izquierda


    # Start threads for reading serial data and writing to CSV
    threading.Thread(target=read_serial, daemon=True).start()
    threading.Thread(target=write_csv, daemon=True).start()
    
    # Start the data update loop
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)  # Update every 50 ms

    win_control.show()
    win_normal.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
