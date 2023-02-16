#!/bin/bash

LIMO_IP_1=$1
LIMO_IP_2=$2
BACKEND_IP=$(ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')

docker-compose -f interface-docker-compose.yml build
BACKEND_IP=$BACKEND_IP LIMO_IP_1=$LIMO_IP_1 LIMO_IP_2=$LIMO_IP_2 docker-compose -f interface-docker-compose.yml up