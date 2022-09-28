import socket


class TCPClient:
    def __init__(self, host, port):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host = host
        self.port = port
        self.drive_command = (0.0, 0.0, 0.0)
        self.enabled = False
    
    def set_command(self, vx, vy, vt):
        self.drive_command = (vx, vy, vt)
    
    def set_enable(self, enabled):
        self.enabled = enabled
        data = f"e,{int(self.enabled)}\n"
        self.s.sendall(data.encode())

    def start(self):
        print(f"Connecting to {self.host}:{self.port}")
        self.s.connect((self.host, self.port))

    def update(self):
        data = f"d,{self.drive_command[0]:0.4f}," \
               f"{self.drive_command[1]:0.4f}," \
               f"{self.drive_command[2]:0.4f}\n"
        self.s.sendall(data.encode())
