#!/bin/bash

# Euler to quaternion
# qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
# qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
# qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
# qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

# roll = 0, pitch = 0, 
yaw=`awk "BEGIN {print $4/180.0*3.14159}"`

qz=`awk "BEGIN {print sin($yaw/2)}"`
qw=`awk "BEGIN {print cos($yaw/2)}"`

#echo "0 0 $qz $qw"



docker exec -it stage bash -ci \
"rostopic pub /$1/setpose geometry_msgs/Pose \"position: { x: $2,  y: $3,  z: 0.0 }
orientation: { x: 0.0, y: 0.0, z: $qz, w: $qw }\" --once" 

#docker exec -it stage bash -ci \
#"rostopic pub /alice/setpose geometry_msgs/Pose \"position: { x: -3.6,  y: 1.6,  z: 0.0 }
#orientation: { x: 0.0, y: 0.0, z: -0.17, w: 1.0 }\" --once" 

#docker exec -it stage bash -ci \
#"rostopic pub /bob/setpose geometry_msgs/Pose \"position: { x: 1.25,  y: -2.7,  z: 0.0 } 
#orientation: { x: 0.0, y: 0.0, z: 1.8, w: 1.0 } \" --once"

#docker exec -it stage bash -ci \
#"rostopic pub /carol/setpose geometry_msgs/Pose \"position: { x: 10.5,  y: -4.2,  z: 0.0 }
#orientation: { x: 0.0, y: 0.0, z: 1.2, w: 1.0 }\" --once" 

