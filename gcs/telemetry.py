import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), "pb"))

import socket
from pb import telemetry_pb2
from telemetry_gui import plt, create_state_plot

UDP_IP = "127.0.0.1"
UDP_PORT = 25565
RECV_BUFFER_SIZE = 1024

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.bind((UDP_IP, UDP_PORT))

# update_state = create_state_plot()
# plt.show(block = False)
# plt.pause(1)

while True:
    data, addr = udp_sock.recvfrom(RECV_BUFFER_SIZE)
    
    tel_msg = telemetry_pb2.TelemetryMessage()
    tel_msg.ParseFromString(data)
    print(tel_msg.state)
    print(tel_msg.sensor_data)
    # update_state(tel_msg.state, 1)