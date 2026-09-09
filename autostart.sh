sudo jetson_clocks
sleep 5
cd ~/Desktop/sp_vision_25_rbclone/build
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./rbnx_auto_aim_debug_async"