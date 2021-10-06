# -*- coding: utf-8 -*-
import csv
import Adafruit_PureIO.smbus as smbus  # import SMBus module of I2C
from BMP085 import BMP085  # import SMBus module of I2C
import time  # import time for library

bus = smbus.SMBus(1)


# Calibration function
def Calibrate():
    tmpAx = 0
    tmpAy = 0
    tmpAz = 0
    tmpGx = 0
    tmpGy = 0
    tmpGz = 0
    for i in range(0, 20):

        # collect acceleration data
        # read Accelerometer raw value
        acc_x = read_raw_data(59)
        acc_y = read_raw_data(61)
        acc_z = read_raw_data(63)
        # read Gyroscope raw value
        gyro_x = read_raw_data(67)
        gyro_y = read_raw_data(69)
        gyro_z = read_raw_data(71)
        # Full scale range +/- 250 degree/C as per sensitivity scale factor
        tmpAx += acc_x / 16384.0
        tmpAy += acc_y / 16384.0
        tmpAz += acc_z / 16384.0
        tmpGx += gyro_y / 131.0
        tmpGy += gyro_y / 131.0
        tmpGz += gyro_z / 131.0
        
    offAx = tmpAx / 20
    offAy = tmpAy / 20
    offAz = tmpAz / 20
    offGx = tmpGx / 20
    offGy = tmpGy / 20
    offGz = tmpGz / 20
    return[offAx, offAy, offAz, offGx, offGy, offGz]


# Accelerometer initiation
def MPU_Init():
    # write to sample rate register
    bus.write_byte_data(Device_Address, 25, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, 107, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address, 26, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, 27, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, 56, 1)


def read_raw_data(addr):
    # Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)

    # concatenate higher and lower value
    value = (high << 8) | low

    # to get signed value from mpu6050
    if value > 32768:
        value = value - 65536
    return value

def altitude_method(num_of_avg):
    alt = 0
    for i in range(0, num_of_avg):
        alt += bmp.read_altitude()
    return (alt / num_of_avg)

Device_Address = 0x68  # MPU6050 device address

MPU_Init()

print("Reading Data of Gyroscope and Accelerometer")


# Initialise the BMP085 and use STANDARD mode (default value)
# bmp = BMP085(0x77, debug=True)
bmp = BMP085(address=0x77)
time.sleep(45)
calData = Calibrate()
print("Calibration Data: {}".format(calData))
offAx = calData[0]
offAy = calData[1]
offAz = calData[2]
offGx = calData[3]
offGy = calData[4]
offGz = calData[5]
print([offAx, offAz, offAy, offGz, offGy, offGx])
# Start new CSV log entry
with open(r"/home/pi/MainCode62/log/dataLog.csv", "a", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([""])
    writer.writerow(["New Entry"])
    writer.writerow(
        ["Temperature", "Pressure", "Altitude", "Gx", "Gy", "Gz", "Ax", "Ay", "Az"]
    )
    writer.close()
#Pre-liftoff loop (sensors on standby until takeoff)
#alt_start_req = (altitude_method(20))+1

#while (altitude_method(20) < alt_start_req):
 # time.sleep(0.2)


# Main program
while True:
    # analyze altitude
    altitude = altitude_method(2)


    # collect temperature data
    temp = bmp.read_temperature()

    # collect preassure data
    pressure = bmp.read_pressure()

    # collect acceleration data
    # Read Accelerometer raw value
    acc_x = read_raw_data(59)
    acc_y = read_raw_data(61)
    acc_z = read_raw_data(63)
    # Read Gyroscope raw value
    gyro_x = read_raw_data(67)
    gyro_y = read_raw_data(69)
    gyro_z = read_raw_data(71)
    # Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = (acc_x / 16384.0) - offAx
    Ay = (acc_y / 16384.0) - offAy
    Az = (acc_z / 16384.0) - offAz
    Gx = (gyro_x / 131.0) - offGx
    Gy = (gyro_y / 131.0) - offGy
    Gz = (gyro_z / 131.0) - offGz

    # Store sensor data to CSV log
    with open(r"/home/pi/MainCode62/log/dataLog.csv", "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([temp, (pressure / 100.0), altitude, Gx, Gy, Gz, Ax, Ay, Az])
        writer.close()
    # display altitude
    print("Altitude:    %.2f m" % altitude)

    # display acceleration data
    print(
        "Gx=%.2f" % Gx,
        "\u00b0" + "/s",
        "\tGy=%.2f" % Gy,
        "\u00b0" + "/s",
        "\tGz=%.2f" % Gz,
        "\u00b0" + "/s",
        "\tAx=%.2f g" % Ax,
        "\tAy=%.2f g" % Ay,
        "\tAz=%.2f g" % Az,
    )

    # display temperature data
    print("Temperature: %.2f C" % temp)

    # display pressure data
    print("Pressure:    %.2f hPa" % (pressure / 100.0))

    time.sleep(0.25)
