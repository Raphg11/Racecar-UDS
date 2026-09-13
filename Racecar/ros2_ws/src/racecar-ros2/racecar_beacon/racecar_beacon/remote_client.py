#!/usr/bin/env python3

import socket
from struct import unpack

"""
NOTES:

- This process MUST listen to a different port than the PositionBroadcast client;
- A socket MUST be closed BEFORE exiting the process.
"""


def main():
    # TODO: Implement the RemoteRequest client here.
    server_ip = "10.0.1.21"
    server_port =65432


    sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    sock.connect((server_ip,server_port))

    command=input("Commande (RPOS, OBSF ou RBID) : ")

    print(f"essai d'envoyer la commande {command}")
    sock.sendall(command.encode('ascii'))

    data=sock.recv(16)
    print(f"data : {data}")
    if command=="RPOS":
        x,y,theta=unpack("fff4x",data)
        print(f"X: {x}, Y: {y}, theta: {theta}")
    elif command=="OBSF":
        obstacle=unpack("I12x",data)
        print(bool(obstacle))
    elif command=="RBID":
        id=unpack("I12x",data)[0]
        print(id)

    
    pass

    sock.close()

if __name__ == "__main__":
    main()
