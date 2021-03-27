#!/bin/bash

docker exec -it actions bash -ci "rostopic pub pnp/action_str  std_msgs/String  \"data: 'say_going_printer1.start' \" --once "

docker exec -it actions bash -ci "rostopic pub pnp/action_str  std_msgs/String  \"data: 'goto_printer1.start' \" --once "

sleep 10

docker exec -it actions bash -ci "rostopic pub pnp/action_str  std_msgs/String  \"data: 'say_going_home.start' \" --once "

docker exec -it actions bash -ci "rostopic pub pnp/action_str  std_msgs/String  \"data: 'goto_home.start' \" --once "

