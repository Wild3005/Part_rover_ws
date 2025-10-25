akses resolusi kamera:

v4l2-ctl --list-devices

v4l2-ctl -d /dev/video2 --list-formats-ext


Liat param untuk mengubah parameter

ros2 param list

ros2 param get /arm_pov fps_main

ros2 param set /arm_pov fps_mini_arm 30