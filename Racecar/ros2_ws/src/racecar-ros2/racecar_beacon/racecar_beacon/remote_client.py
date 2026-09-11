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
    #server_ip = "10.0.1.21"
    server_ip = "127.0.0.1"
    server_port = 5000


    sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    sock.connect((server_ip,server_port))

    command=input("Commande (RPOS, OBSF ou RBID) : ")

    print(f"essai d'envoyer la commande {command}")
    sock.sendall(command.encode('ascii'))

    data=sock.recv(16)
    print(f"data : {data}")
    if command=="RPOS":
        x,y,theta,_=unpack("!fffI",data)
        print(f"X: {x}, Y: {y}, theta: {theta}")
    elif command=="OBSF":
        obstacle,_,_,_=unpack("!IIII",data)
        print(bool(obstacle))
    elif command=="RBID":
        id,_,_,_=unpack("!IIII",data)
        print(id)

    pass


if __name__ == "__main__":
    main()
