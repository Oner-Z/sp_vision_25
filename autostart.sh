sleep 1
cd ~/Desktop/yrl/sp_vision_25/
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./build/auto_buff_debug configs/standard4.yaml"