
import socket
import matplotlib.pyplot as plt
import numpy as np
import threading
import re
import time


# Note: need to make sure both PC and ESP32 are on the same network (better to set up ESP32 as AP - need to test range)
# TODO: implement multithreading into this to process the data...

HOST = "192.168.4.2"  # IP address of PC
PORT = 7050  # Port to listen on (non-privileged ports are > 1023)


# Initialise lists to store plot
index = []
x = []
y = []
distance = []

def TCP_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            while True:
                data = conn.recv(1024)
                s = data.decode('utf-8')  # decode bytes to string using UTF-8 encoding
                groups = re.findall(r'\((.*?)\)', s)

                for g in groups:
                    parts = g.split(", ")           # split by comma+space
                    data = [float(x) for x in parts]
                    index.append(data[0])
                    x.append(data[1])
                    y.append(data[2])
                    distance.append(data[-1])

                if not data:
                    break
                #conn.sendall(b'Acknowledged')

def display_map():
    print("TODO: write code")

    while(True):
        print(x)
        print(y)
        print(distance)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(x, y, distance, color='blue', marker='o')
        plt.show()
        time.sleep(1)

def main():

    # Start TCP server thread
    t_server = threading.Thread(target=TCP_server)
    t_server.start()

    # start plotting thread
    t_plot = threading.Thread(target=display_map)
    t_plot.start()


if __name__ == "__main__":
    main()
