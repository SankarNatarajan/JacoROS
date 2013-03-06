Issue : we got quite number of points from joint_trajectory_action/goal.... for eg., to move 30 cm we will get sometimes 400 trajectory points....
Cause : the trajectory filter from arm navigation is fitlering/discretising the trajectory quite heavily
Solution : change  the params: {discretization: 0.01} in the config file filter.yaml to  params: {discretization: 0.05}. Before this config file is in arm
		/opt/ros/electric/stacks/arm_navigation/trajectory_filter_server/config/filters.yaml...now it has been moved to jaco_arm_navigation/config
