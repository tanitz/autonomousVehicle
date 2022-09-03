#! /bin/bash		   	
gnome-terminal --tab --command "sh -c 'roslaunch map_file \
									points_map_loader.launch \
									scene_num:=noupdate \
									path_area_list:=None \
									path_pcd:=/home/administrator/map/260722/0.pcd'" \
--tab --command "sh -c 'sleep 2;roslaunch \
			   			map_file \
						lanelet2_map_loader.launch \
						file_name:=/home/administrator/map/260722/8/2.osm'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						points_downsampler \
						points_downsample.launch \
						node_name:=voxel_grid_filter \
						points_topic:=/os_cloud_node/points'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						twist_generator \
						vehicle_status_converter.launch \
						enable_adaptive_estimate:=True \
						enable_steering_offset_estimate:=False \
						wheelbase:=2.9'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						autoware_connector \
						vel_pose_connect.launch \
						topic_pose_stamped:=/ndt_pose \
						topic_twist_stamped:=/estimate_twist \
						sim_mode:=False'" \
--tab --command "sh -c 'sleep ;roslaunch \
						waypoint_planner \
						velocity_set_option.launch \
						use_crosswalk_detection:=False \
						enable_multiple_crosswalk_detection:=False \
						points_topic:=points_no_ground \
						use_ll2:=False'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						pure_pursuit \
						pure_pursuit.launch \
						is_linear_interpolation:=True \
						publishes_for_steering_robot:=True \
						add_virtual_end_waypoints:=True'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						twist_filter \
						twist_filter.launch \
						use_decision_maker:=False'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						op_local_planner \
						op_common_params.launch \
						horizonDistance:=120 \
						maxLocalPlanDistance:=80 \
						pathDensity:=0.5 \
						rollOutDensity:=0.5 \
						rollOutsNumber:=4 \
						maxVelocity:=3 \
						maxAcceleration:=3 \
						maxDeceleration:=-3 \
						enableFollowing:=True \
						enableSwerving:=True \
						minFollowingDistance:=30 \
						minDistanceToAvoid:=15 \
						maxDistanceToAvoid:=4 \
						enableStopSignBehavior:=True \
						enableTrafficLightBehavior:=False \
						enableLaneChange:=False \
						horizontalSafetyDistance:=1 \
						verticalSafetyDistance:=1 \
						velocitySource:=1'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						op_local_planner \
						op_trajectory_generator.launch \
						samplingTipMargin:=4 \
						samplingOutMargin:=12'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						op_local_planner \
						op_motion_predictor.launch \
						enableCurbObstacles:=False \
						enableGenrateBranches:=False \
						max_distance_to_lane:=2 \
						prediction_distance:=25 \
						enableStepByStepSignal:=False \
						enableParticleFilterPrediction:=False'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						op_local_planner \
						op_trajectory_evaluator.launch \
						enablePrediction:=True'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						op_local_planner \
						op_behavior_selector.launch'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						lattice_planner \
						lattice_velocity_set.launch \
						use_crosswalk_detection:=False'" \
--tab --command "sh -c 'sleep 2;rosrun \
						lattice_planner \
						path_select'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						lattice_planner \
						lattice_trajectory_gen.launch \
						sim_mode:=False \
						prius_mode:=False \
						mkz_mode:=False'" \
--tab --command "sh -c 'sleep 2;rosrun \
						lattice_planner \
						lattice_twist_convert'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						lane_planner \
						lane_rule_option.launch \
						acceleration:=1 \
						stopline_search_radius:=1 \
						number_of_zeros_ahead:=0 \
						number_of_zeros_behind:=0 \
						number_of_smoothing_count:=0 \
						use_ll2:=False'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						lane_planner \
						lane_select.launch \
						search_closest_waypoint_minimum_dt:=5'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						runtime_manager \
						calibration_publisher.launch \
						target_frame:=os_sensor \
						camera_frame:=camera \
						file:=/home/administrator/20220729_1022_autoware_camera_calibration.yaml \
						register_lidar2camera_tf:=True \
						publish_extrinsic_mat:=True \
						publish_camera_info:=True \
						camera_info_topic:=/camera_info \
						image_topic_src:=/usb_cam/raw_image \
						projection_matrix_topic:=/projection_matrix \
						camera_id:=/usb_cam'" \
--tab --command "sh -c 'sleep 2;roslaunch \
						vision_darknet_detect \
						vision_yolo3_detect.launch \
						score_threshold:=0.40 \
						nms_threshold:=0.50 \
						image_src:=usb_cam/raw_image \
						network_definition_file:=/home/administrator/autoware.ai/install/vision_darknet_detect/share/vision_darknet_detect/darknet/cfg/yolov3-tiny.cfg \
						pretrained_model_file:=/home/administrator/autoware.ai/src/autoware/core_perception/vision_darknet_detect/darknet/data/yolov3-tiny.weights \
						names_file:=/home/administrator/autoware.ai/install/vision_darknet_detect/share/vision_darknet_detect/darknet/cfg/coco.names \
						gpu_device_id:=0 \
						camera_id:='" \
--tab --command "sh -c 'sleep 2;rosrun \
						rviz rviz'" \
						
xdotool key ctrl+L
   