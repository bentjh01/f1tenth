#!/bin/sh

echo "Starting F1TENTH Container"

if [ "$(docker ps -aq -f status=running -f name=${container_name})" ]
then
	echo "Container is Running. Starting new session."
	docker exec -it "f1tenth_bt" bash 
else
	echo "Creating new container."

    docker run -it --rm \
    --name "f1tenth_bt" \
    -h "f1tenth_bt" \
    --network=host \
	--volume="$(pwd)/f1tenth_ws":"/root/f1tenth_ws" \
    bentjh01:f1tenth-foxy
fi
