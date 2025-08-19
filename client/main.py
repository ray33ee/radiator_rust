import socket

HOST = "192.168.1.214"  # ESP32-C3 IP address
PORT = 8080             # Port to connect to

with socket.create_connection((HOST, PORT), timeout=5) as sock:
    #sock.sendall(b'{"function":{ "CalibratePull": 10 } }\0')
    #sock.sendall(b'{"function":{ "CalibratePush": 10 } }\0')
    #sock.sendall(b'{"function":"Unlock" }\0')
    #sock.sendall(b'{"function":"Test" }\0')
    #sock.sendall(b'{"function":{ "DebugSetPosition": 10 } }\0')
    sock.sendall(b'{"function":"DebugGetPosition" }\0')