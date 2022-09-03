#! /bin/bash
gnome-terminal 	--tab --command "sh -c 'roscore'" \
--tab --command "sh -c 'sleep 2;roslaunch \
							aev_controller \
							webserver.launch'" \
--tab --command "sh -c 'sleep 2;roslaunch \
							aev_controller \
							web_image.launch'" \
--tab --command "sh -c 'sleep 2;roslaunch \
							usb_cam \
							usb_cam-test.launch'" \
--tab --command "sh -c 'sleep 2;roslaunch \
							aev_controller \
							AEV.launch'" \
--tab --command "sh -c 'sleep 2;rosrun \
							aev_controller \
							aev_controller.py'" \
--tab --command "sh -c 'sleep 2;rosrun \
							aev_controller \
							initial.py'" \
--tab --command "sh -c 'sleep 2;rosrun aev_controller \
 							subCameraDetection.py'"\
		
xdotool key ctrl+L