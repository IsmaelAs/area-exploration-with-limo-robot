#!/bin/bash

BACKEND_IP=$(hostname -I | head -n1 | awk '{print $1;}')

docker-compose -f interface-docker-compose.yml build
BACKEND_IP=$BACKEND_IP docker-compose -f interface-docker-compose.yml up