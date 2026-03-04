#!/bin/bash
sudo slcand -o -c -s6 /dev/ttyACM0 can0
sudo ip link set can0 up
