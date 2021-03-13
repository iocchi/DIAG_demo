#!/bin/bash

docker exec -it actions bash -ci "rostopic pub pnp/action_str  std_msgs/String  \"data: 'say_going_home.start' \" --once "

docker exec -it actions bash -ci "rostopic pub pnp/action_str  std_msgs/String  \"data: 'goto_home.start' \" --once "


