import time
import serial

address = "/dev/ttyTHS0"
baud = 1000000
# address = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_12396120-if00"
# baud = 9600
device = serial.Serial(address, baud, timeout=0.25)

written = []

try:
    count = 0
    while True:
        now = time.time()
        packet = str(now)
        packet += "\n"
        packet = packet.encode()
        written.append(packet)
        device.write(packet)
        line = device.readline()
        count += 1
        if line in written:
            written.remove(line)
        else:
            print("Bad packet:", line)
        if count % 10 == 0:
            ping_return = float(line.decode())
            print("Ping:", time.time() - ping_return)
        time.sleep(0.005)
finally:
    print("Read %s lines" % count)
    print("Non returned pings: %s" % len(written))
    device.close()
