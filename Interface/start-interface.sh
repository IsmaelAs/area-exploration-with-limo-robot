#!/bin/bash

IS_SIMULATION=$1
BACKEND_IP=$(hostname -I | head -n1 | awk '{print $1;}')

docker-compose -f interface-docker-compose.yml build
BACKEND_IP=$BACKEND_IP IS_SIMULATION=$IS_SIMULATION docker-compose -f interface-docker-compose.yml up