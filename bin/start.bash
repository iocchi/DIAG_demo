#!/bin/bash

DCF=docker-compose.yml
if [ "$1" == "dev" ]; then
  DCF=docker-compose-dev.yml
fi

# docker services
docker-compose -f $DCF up -d stage navigation actions pnp

sleep 5

docker ps

sleep 1

# Stage with map
echo 'DISB1;marrtino' | netcat -w 1 localhost 9235
sleep 5

# Navigation
echo '@loc' | netcat -w 1 localhost 9238
sleep 3
echo '@movebase' | netcat -w 1 localhost 9238
sleep 3

# check
docker exec -it stage bash -ci "rosnode list"


