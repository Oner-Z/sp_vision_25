docker run \
    --user=rm \
    --network=host \
    -it \
    -v $PWD:/sp_vision \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --env=DISPLAY \
    -w /sp_vision \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    sp_vision