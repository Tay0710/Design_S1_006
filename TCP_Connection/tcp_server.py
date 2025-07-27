
import socket

# Note: need to make sure both PC and ESP32 are on the same network (better to set up ESP32 as AP - need to test range)
# TODO: implement multithreading into this to process the data...

HOST = "192.168.4.2"  # IP address of PC
PORT = 7050  # Port to listen on (non-privileged ports are > 1023)


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            while True:
                data = conn.recv(1024)
                print(data)
                if not data:
                    break
                #conn.sendall(b'Acknowledged')


if __name__ == "__main__":
    main()
