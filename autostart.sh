sleep 5
cd /home/wele/ros_ws/src/sp_vision_25_psp/
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./watchdog.sh"

# source /opt/ros/jazzy/setup.bash 
# source ~/ros_ws/install/setup.bash 
# ./build/sentry_standard_mpc ./configs/sentry.yaml 
