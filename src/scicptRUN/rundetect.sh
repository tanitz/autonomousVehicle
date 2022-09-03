#! /bin/bash
gnome-terminal --tab --command "sh -c 'roslaunch \
						points_preprocessor \
						ray_ground_filter.launch \
						node_name:=ray_ground_filter \
						input_point_topic:=/os_cloud_node/points \
						base_frame:=base_link clipping_height:=2 \
						min_point_distance:=1.85 \
						radial_divider_angle:=0.08 \
						concentric_divider_distance:=0 \
						local_max_slope:=8 \
						general_max_slope:=5 \
						min_height_threshold:=0.05 \
						reclass_distance_threshold:=0.5'" \
--tab --command "sh -c 'sleep 2;roslaunch \
                            lidar_naive_l_shape_detect \
                            lidar_naive_l_shape_detect.launch \
                            input_topic:=/detection/lidar_detector/objects \
                            output_topic:=/detection/lidar_objects/l_shaped \
                            random_ponts:=500 slope_dist_thres:=6 \
                            num_points_thres:=8 sensor_height:=2.3'" \
--tab --command "sh -c 'roslaunch \
                        lidar_euclidean_cluster_detect \
                        lidar_euclidean_cluster_detect.launch \
                        use_gpu:=False \output_frame:=os_sensor \
                        pose_estimation:=True \
                        downsample_cloud:=False \
                        points_node:=/os_cloud_node/points \
                        leaf_size:=0.1 \
                        cluster_size_min:=5 \
                        cluster_size_max:=100000 \
                        clustering_distance:=0.75 \
                        clip_min_height:=-1.3 \
                        clip_max_height:=0.5 \
                        use_vector_map:=False \
                        vectormap_frame:=map \
                        wayarea_gridmap_topic:=grid_map_wayarea \
                        wayarea_gridmap_layer:=wayarea \
                        wayarea_no_road_value:=255 \
                        remove_points_upto:=2.5 \
                        keep_lanes:=True \
                        keep_lane_left_distance:=1.25 \
                        keep_lane_right_distance:=1.25 \
                        cluster_merge_threshold:=2 \
                        use_multiple_thres:=False \
                        clustering_ranges:=[15,30,45,60] \
                        clustering_distances:=[0.5,1.1,1.6,2.1,2.6] \
                        remove_ground:=False \
                        use_diffnormals:=False \
                        publish_filtered:=False'" \
--tab --command "sh -c 'sleep 2;roslaunch \
                            waypoint_maker \
                            waypoint_loader.launch \
                            load_csv:=True \
                            multi_lane_csv:=/home/administrator/map/260722/8/2.csv \
                            replanning_mode:=True \
                            realtime_tuning_mode:=False \
                            resample_mode:=True \
                            resample_interval:=1 \
                            replan_curve_mode:=True \
                            replan_endpoint_mode:=True \
                            velocity_max:=10 \
                            radius_min:=6 \
                            velocity_min:=4 \
                            accel_limit:=0.5 \
                            decel_limit:=0.3 \
                            lateral_accel_limit:=2 \
                            use_decision_maker:=False'" \
--tab --command "sh -c 'sleep 2;roslaunch \
                            lidar_shape_estimation \
                            shape_estimation_clustering.launch \
                            input:=/detection/lidar_detector/objects \
                            output:=/detection/shape_estimation/objects'" \

xdotool key ctrl+L