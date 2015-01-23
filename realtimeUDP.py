#!/usr/bin/python

#   Calder Phillips-Grafflin
#   Driver for low-level robot controller based on a Beaglebone Black (or compatible)
#   controller, with "realtime" communications over UDP. Since UDP provides no
#   guarantees, this driver should only be relied upon in point-to-point network
#   topologies where the controller is directly connected to a dedicated ethernet
#   port on the master.

import sys
import socket
import struct

class CommandArgsTooLargeException(Exception):
    pass

class RealtimeUDPMaster(object):

    def __init__(self, master_ip_address, mosi_port, miso_port):
        self.mosi_port = mosi_port
        self.mosi_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.miso_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.miso_socket.setblocking(0)
        self.miso_socket.bind((master_ip_address, miso_port))

    def send(self, slave_ip_address, command, command_arg_bytes):
        if len(command_arg_bytes) > 251:
            raise CommmandArgsTooLargeException("Command arg bytes exceeds limit of 251 bytes")
        # Make the new frame of 256 bytes
        send_bytes = bytearray(256)
        # Set the first 4 bytes as the command
        send_bytes[0:4] = struct.pack('!I', command)
        # Set the next byte as the len of the arg bytes
        send_bytes[4:5] = struct.pack('!B', len(command_arg_bytes))
        # Set the arg bytes
        send_bytes[5:(5 + len(command_arg_bytes))] = command_arg_bytes
        # The rest of the frame is already filled with zeros, se we don't need to set it
        # Send the frame to the target slave address
        self.mosi_socket.sendto(send_bytes, (slave_ip_address, self.mosi_port))
        # Check to see if we need to wait for data to be returned
        # Read and Write/Read commmands are all odd (LSb = 1)
        if (command % 2) == 0:
            return None
        else:
            self.miso_socket.setblocking(1)
            [response, response_arg_bytes] = self.recv()
            self.miso_socket.setblocking(0)
            return [response, response_arg_bytes]

    def recv(self):
        (recv_bytes, address) = self.miso_socket.recvfrom(256)
        (response,) = struct.unpack('!I', recv_bytes[0:4])
        (response_arg_len,) = struct.unpack('!B', recv_bytes[4:5])
        response_arg_bytes = recv_bytes[5:(5 + response_arg_len)]
        return [response, response_arg_bytes]

if __name__ == '__main__':
    if len(sys.argv) > 1:
        flag = sys.argv[1]
        if flag == '-m':
            master = RealtimeUDPMaster('192.168.1.2', 10001, 10002)
            while True:
                msg = raw_input("Enter text: ")
                [code, response] = master.send('192.168.1.3', 1, msg)
                print struct.unpack('f', response[0:4])[0]
                print struct.unpack('f', response[4:8])[0]
                #print(response)
        else:
            print("Invalid flag - needs -m for testing")
    else:
        print("Needs a flag -m for testing")