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
import threading


class ReservedCommands(object):
    pass

class CommandReservedException(Exception):
    pass

class CommandArgsTooLargeException(Exception):
    pass

class ResponseArgsTooLargeException(Exception):
    pass

class CommandNotRegisteredException(Exception):
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

class RealtimeUDPSlave(object):

    def __init__(self, master_ip_address, slave_ip_address, mosi_port, miso_port, registered_commands={}):
        '''
        In the python reference version, commands are registered in a dictionary of str(int command):command_callback_fn
        In other languages, commands can be registered with map<int command, (bool *)command_callback_fn(vector& arg_bytes, vector& ret_bytes)>
        '''
        self.master_ip_address = master_ip_address
        self.miso_port = miso_port
        self.miso_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mosi_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mosi_socket.bind((slave_ip_address, mosi_port))
        self.command_dict = registered_commands

    def loop(self):
        while True:
            (command_bytes, address) = self.mosi_socket.recvfrom(256)
            (command,) = struct.unpack('!I', command_bytes[0:4])
            (command_arg_len,) = struct.unpack('!B', command_bytes[4:5])
            command_arg_bytes = command_bytes[5:(5 + command_arg_len)]
            [response, response_arg_bytes] = self.process_command(command, command_arg_bytes)
            self.reply(response, response_arg_bytes)

    def reply(self, response, response_arg_bytes):
        if len(response_arg_bytes) > 251:
            raise ResponseArgsTooLargeException("Response arg bytes exceeds limit of 251 bytes")
        # Make the new frame of 256 bytes
        send_bytes = bytearray(256)
        # Set the first 4 bytes as the command
        send_bytes[0:4] = struct.pack('!I', response)
        # Set the next byte as the len of the arg bytes
        send_bytes[4:5] = struct.pack('!B', len(response_arg_bytes))
        # Set the arg bytes
        send_bytes[5:(5 + len(response_arg_bytes))] = response_arg_bytes
        # The rest of the frame is already filled with zeros, se we don't need to set it
        # Send the frame to the master address
        self.miso_socket.sendto(send_bytes, (self.master_ip_address, self.miso_port))

    def process_command(self, command, command_arg_bytes):
        try:
            [response, response_arg_bytes] = self.command_dict[str(command)](command_arg_bytes)
            return [response, response_arg_bytes]
        except KeyError as ex:
            print("Command " + str(command) + " not registered")
            return [0, '']

if __name__ == '__main__':
    if len(sys.argv) > 1:
        flag = sys.argv[1]
        if flag == '-m':
            master = RealtimeUDPMaster('anette.local', 10001, 10002)
            while True:
                msg = raw_input("Enter text: ")
                [code, response] = master.send('ada.local', 1, msg)
                print(response)
        elif flag == '-s':
            handler = lambda msg: [2, "echo: " + msg]
            slave = RealtimeUDPSlave('anette.local', 'ada.local', 10001, 10002, {'1':handler})
            slave.loop()
        else:
            print("Invalid flag - needs -m or -s for testing")
    else:
        print("Needs a flag -m or -s for testing")