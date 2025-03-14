import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), "pb"))

import socket
from pb import command_pb2

UDP_IP = "127.0.0.1"
UDP_PORT = 25564
RECV_BUFFER_SIZE = 1024

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#udp_sock.bind((UDP_IP, UDP_PORT))


def set_pb_vec3(vec, x, y, z):
    vec.x, vec.y, vec.z = x, y, z

def set_pb_vec4(vec, w, x, y, z):
    vec.w, vec.x, vec.y, vec.z = w, x, y, z


def send_command(command):
    udp_sock.sendto(command.SerializeToString(), (UDP_IP, UDP_PORT))

def copter_set_angular_velocity(angular_velocity, thrust):
    command = command_pb2.Command()
    subcommand = command.copter_command.set_angular_velocity
    
    set_pb_vec3(subcommand.angular_velocity, *angular_velocity)
    subcommand.thrust = thrust

    send_command(command)

def copter_set_linear_velocity(velocity, direction):
    command = command_pb2.Command()
    subcommand = command.copter_command.set_linear_velocity
    
    set_pb_vec3(subcommand.velocity, *velocity)
    subcommand.direction = direction

    send_command(command)
