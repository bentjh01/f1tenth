#!/bin/sh

container_name="f1tenth_gym_ros-sim-1"

if [ "$(docker ps -aq -f status=running -f name=${container_name})" ]
then
	echo "Container is Running. Starting new session."
	docker exec -it ${container_name} bash 
fi
