import select
import socket

import re

class TCPServer:
    def __init__(self, port):
        self.s = None
        self.client_socket = None
        self.disconnect_time = None
        self.port = port
        self.drive_command = [0.0, 0.0, 0.0]
        self.enabled = False
    
    def get_command(self):
        return self.drive_command
    
    def get_enabled(self):
        return self.enabled

    def update(self):
        if self.s is None:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.setblocking(False)
            self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.s.bind(('', self.port))
            self.s.listen()
        elif self.client_socket is None:
            # returns tuple: (readable, writable, errored)
            readable, _, _ = select.select([self.s], [], [], 0)
            for s in readable:
                if s is self.s:
                    self.client_socket, address = self.s.accept()
                    self.client_socket.setblocking(False)
                    self.client_socket.settimeout(0)
                    print('connection accepted from {}'.format(address))
        else:
            readable, _, _ = select.select([self.client_socket], [], [], 0)
            for cs in readable:
                if cs is self.client_socket:
                    try:
                        packet = self.client_socket.recv(1024)
                    except socket.timeout as e:
                        print('socket.timeout exception, try again later')
                        err = e.args[0]
                        raise err
                    except socket.error as e:
                        print('socket.error: {}'.format(e))
                        raise e
                    if not packet:
                        cs.close()
                        self.client_socket = None
                        print('client disconnected')
                        return
                    packet = packet.decode()
                    packet = re.split('[\r\n]', packet)
                    for data in packet:
                        is_valid = bool(re.fullmatch('[a-zA-Z0-9\,\.\-]*', data))
                        if is_valid:
                            command = list(data.split(','))
                            for ix in range(len(command)):
                                if command[ix].isnumeric():
                                    command[ix] = int(command[ix])  # convert to integer
                                elif command[ix].replace(".", "").isnumeric():
                                    command[ix] = float(command[ix])  # convert to float
                            if data != '':
                                if command[0] == 'd':
                                    self.drive_command[0] = command[1]
                                    self.drive_command[1] = command[2]
                                    self.drive_command[2] = command[3]
                                elif command[0] == 'e':
                                    self.enabled = bool(command[1])
                        else:
                            print('not valid: {}'.format(data))
