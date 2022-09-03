#! /bin/bash
gnome-terminal 	--tab --command "sh -c 'roslaunch \
                                    lidar_localizer \
                                    ndt_matching.launch \
                                    method_type:=0 \
                                    use_odom:=False \
                                    use_imu:=False \
                                    imu_upside_down:=False \
                                    imu_topic:=/os_cloud_node/imu \
                                    get_height:=False \
                                    output_log_data:=True'" \

xdotool key ctrl+L