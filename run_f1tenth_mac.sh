#!/bin/sh

container_name="f1tenth_bt"

function create_ws_container {
    docker run -it --rm \
        --name $container_name \
        -h "f1tenth_bt" \
        --network=host \
        --volume="$(pwd)/f1tenth_ws":"/root/f1tenth_ws" \
        bentjh01:f1tenth-foxy
}

if [ "$(docker ps -aq -f status=running -f name=${container_name})" ]
then
	echo "Container is Running. Starting new session."
	docker exec -it ${container_name} bash 
else
    echo "Starting container"
    create_ws_container
fi