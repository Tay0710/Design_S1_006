import socket
import matplotlib.pyplot as plt
import numpy as np
import threading
import re
import time
import csv
from datetime import datetime
import os

# Note: need to make sure both PC and ESP32 are on the same network (better to set up ESP32 as AP - need to test range)

HOST = "192.168.4.2"  # IP address of PC
PORT = 7050  # Port to listen on (non-privileged ports are > 1023)
current_datetime = datetime.now().replace(microsecond=0)


foldername = "SBUS_DATA" # REPLACE WITH FOLDER NAME
filename = str(current_datetime) + ".csv"
filename = filename.replace(":", "-")

file_path = os.path.join(foldername, filename)


# Initialise lists to store plot
timestamp = []
roll = []
pitch = []
throttle = []
yaw = []
aux1 = []
aux2 = []
aux3 = []

def TCP_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            while True:
                data = conn.recv(1024)  
                print(data)          
                s = data.decode('utf-8')  # decode bytes to string using UTF-8 encoding
                groups = re.findall(r'\((.*?)\)', s)

                for g in groups:
                    parts = g.split(", ")           # split by comma+space
                    data = [int(x) for x in parts]
                    row = [str(x) for x in parts]
                    timestamp.append(data[0])
                    roll.append(data[1])
                    pitch.append(data[2])
                    throttle.append(data[3])
                    yaw.append(data[4])
                    aux1.append(data[5])
                    aux2.append(data[6])
                    aux3.append(data[7])

                    with open(file_path, mode="a", newline="") as file:
                        writer = csv.writer(file)                        
                        writer.writerow(row)

                if not data:
                    break
                #conn.sendall(b'Acknowledged')


def main():
    # Write new file
    with open(file_path, mode="w", newline="") as file:
        writer = csv.writer(file)
        data = ['time', 'Channel 0 (Roll)', 'Channel 1 (Pitch)', 'Channel 2 (Throttle)', 'Channel 3 (Yaw)', 'Aux 1 (Arm)', 'Aux 2 (Angle/Horizon)', 'Aux 3 (Failsafe)']
        writer.writerow(data)

    # Start TCP server thread
    t_server = threading.Thread(target=TCP_server)
    t_server.start()

if __name__ == "__main__":
    main()