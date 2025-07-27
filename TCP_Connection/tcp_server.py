
import socket
import matplotlib.pyplot as plt
import numpy as np
import threading


# Note: need to make sure both PC and ESP32 are on the same network (better to set up ESP32 as AP - need to test range)
# TODO: implement multithreading into this to process the data...

HOST = "192.168.4.2"  # IP address of PC
PORT = 7050  # Port to listen on (non-privileged ports are > 1023)


# Initialise lists to store plot
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
                print(data)
                s = data.decode('utf-8')  # decode bytes to string using UTF-8 encoding
                print(s)              # Output: hello world
                print(type(s))        # <class 'str'>
                # TODO: process data and add to lists
                # "(%d, %d, %.2f, %.2f, %.2f, %.2f)"
                # s = "(1, 2, 3.1415, 2.718, 9.81, 0.99)"
                # s = s.strip("()")               # remove parentheses
                # parts = s.split(", ")           # split by comma+space
                # data = [float(x) for x in parts]

                # print(data)                    # [1.0, 2.0, 3.1415, 2.718, 9.81, 0.99]

                if not data:
                    break
                #conn.sendall(b'Acknowledged')

def display_map():
    print("TODO: write code")
    print(x)
    print(y)
    print(distance)

def main():

    # Start TCP server thread
    t_server = threading.Thread(target=TCP_server)
    t_server.start()

    # start plotting thread
    t_plot = threading.Thread(target=display_map)
    t_plot.start()
    


if __name__ == "__main__":
    main()
