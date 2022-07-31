#!/bin/bash


echo '@lockill' | netcat -w 1 localhost 9238
echo '@movebasekill' | netcat -w 1 localhost 9238
echo '@stagekill' | netcat -w 1 localhost 9235
echo '@audiokill' | netcat -w 1 localhost 9239


sleep 3

docker-compose down

docker container prune -f

docker ps


