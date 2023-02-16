#!/bin/bash

LIMO_IP_1=$1
LIMO_IP_2=$2
BACKEND_IP=$(ipconfig.exe | grep -a 'Wi-Fi' -A 4 | grep -a 'IPv4' | cut -d":" -f 2  | tail -n1 | sed -e 's/\s*//g')

docker-compose -f interface-docker-compose.yml build
BACKEND_IP=$BACKEND_IP LIMO_IP_1=$LIMO_IP_1 LIMO_IP_2=$LIMO_IP_2 docker-compose -f interface-docker-compose.yml up