import serial

address = "/dev/ttyTHS0"
# address = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_11909170-if00"
baud = 9600
device = serial.Serial(address, baud, timeout=0.25)

try:
    while True:
        print(device.readline())
finally:
    device.close()
