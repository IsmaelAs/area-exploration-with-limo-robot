#!/bin/bash

LIMO_IP_1=$1
LIMO_IP_2=$2
BACKEND_IP=$(hostname -I | head -n1 | awk '{print $1;}')

docker-compose -f interface-docker-compose.yml build
BACKEND_IP=$BACKEND_IP LIMO_IP_1=$LIMO_IP_1 LIMO_IP_2=$LIMO_IP_2 docker-compose -f interface-docker-compose.yml up