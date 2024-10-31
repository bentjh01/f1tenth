#!/bin/sh

container_name="f1tenth_bt"
sim_container_name="f1tenth_gym_ros-sim-1"
novnc_container_name="f1tenth_gym_ros-novnc-1"

function create_ws_container {
    docker run -dt --rm \
        --name $container_name \
        -h "f1tenth_bt" \
        --network=host \
        --volume="$(pwd)/f1tenth_ws":"/root/f1tenth_ws" \
        bentjh01:f1tenth-foxy
}

if [ "$(docker ps -aq -f status=running -f name=${sim_container_name})" ]
then
	echo "Container is Running. Starting new session."
	docker exec -it ${sim_container_name} bash 
else
    echo "Starting container"
    docker compose -f ./f1tenth_gym_ros/docker-compose.yml up -d
    create_ws_container

    docker exec -it ${sim_container_name} bash
    
    docker stop ${container_name}
    docker stop ${sim_container_name}
    docker stop ${novnc_container_name}
    docker rm ${container_name}
    docker rm ${sim_container_name}
    docker rm ${novnc_container_name}
fi