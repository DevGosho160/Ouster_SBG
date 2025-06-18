#!/bin/bash

# Exit on any error
set -e

# Default arguments
#Lidar mode options: 512x10, 512x20, 1024x10, 1024x20, 2048x10, 4096x5 (CaptureResolution x Framerate)
LIDAR_MODE="1024x10"
#Window start in millidegrees
WINDOW_START="0"
#Window end in millidegrees
WINDOW_END="360000"
#Run Rviz2
RVIZ="False"
#sudo ./run_ouster.sh --mode '2048x10' --start '90000' --end '270000' --viz 'True'


# Parse arguments (if provided)
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --mode) LIDAR_MODE="$2"; shift ;;
        --start) WINDOW_START="$2"; shift ;;
        --end) WINDOW_END="$2"; shift ;;
        --viz) RVIZ="$2"; shift ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

# Source ROS 2 environment and workspace
source /opt/ros/humble/setup.bash
cd /home/imsel/ros_ws
colcon build --symlink-install
source install/setup.bash

#Set multipurpose io as a trigger
#curl -i -X POST http://169.254.12.39/api/v1/sensor/config -H 'Accept: application/json' -H 'Content-Type: application/json' --data-raw '{"multipurpose_io_mode": "OUTPUT_FROM_ENCODER_ANGLE", "sync_pulse_out_angle": 360}'
# Launch driver in background
ros2 launch ouster_ros driver.launch.py \
    lidar_mode:="${LIDAR_MODE}" \
    azimuth_window_start:="${WINDOW_START}" \
    azimuth_window_end:="${WINDOW_END}" \
    viz:="${RVIZ}" &
DRIVER_PID=$!

curl -i -X POST http://os-122423000060.local/api/v1/sensor/config \
  -H "Content-Type: application/json" \
  --data "{\"multipurpose_io_mode\":\"INPUT_NMEA_UART\",\"nmea_baud_rate\":\"BAUD_115200\",\"nmea_in_polarity\":\"ACTIVE_LOW\",\"nmea_leap_seconds\":37,\"sync_pulse_in_polarity\":\"ACTIVE_HIGH\"}"

# Wait briefly to ensure the driver is publishing
sleep 5

# Run your logger node
ros2 run ouster_logger_c ouster_logger_c
ros2 launch sbg_driver sbg_device_launch.py
ros2 run sbg_logger imu_logger_node
# Optional: kill the launch when logger exits
kill $DRIVER_PID

