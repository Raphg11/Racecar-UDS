#!/usr/bin/env python3

import socket
from struct import unpack

"""
NOTES:

- This process MUST listen to a different port than the RemoteRequest client;
- A socket MUST be closed BEFORE exiting the process.
"""


def main():
    # TODO: Implement the PositionBroadcast client here.

    s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    s.bind(('',5001))

    while True:
        data, address = s.recvfrom(16)

        x, y, z, theta, robotid = unpack('fffI', data)

        print(address[0], robotid, x, y, z, theta)

    s.close()


if __name__ == "__main__":
    main()
