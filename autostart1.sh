sleep 5
source /opt/ros/humble/setup.bash
cd /home/rm/sp_loc_25/
bash -c "./autostart.sh"
cd ~/Desktop/sp_vision_25/
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./watchdog.sh"