def get_params()
	params = ['default']
	if ARGV.length >= 1
		params.push(ARGV[0])
	else
		puts "Too few arguments! USAGE: #{__FILE__} <scenario_profile>"
		exit
	end
	return params
end

def setup_mapping(scan_port, mapper, params)

	## Setup the mapper
	Orocos.conf.apply(mapper, params, true)
	Bundles.transformer.setup(mapper)
	mapper.configure
	mapper.start

	## Setup the data connections
	scan_port.connect_to mapper.scan

end

def setup_mapping_with_filter(scan_port, filter, mapper, params)

	# Setup the pointcloud filter
	Orocos.conf.apply(filter, params, true)
	filter.configure
	filter.start

	setup_mapping(filter.output, mapper, params)

end

def setup_navigation(planner, follower, calc_pose, traversability, mapper, odo, motion_controller)

	Orocos.conf.apply(planner, ['default'])
	planner.configure
	planner.start

	Orocos.conf.apply(traversability, ['default'])
	Bundles.transformer.setup(traversability)
	traversability.configure
	traversability.start

	Orocos.conf.apply(calc_pose, ['default'])
	calc_pose.configure
	calc_pose.start

	Orocos.conf.apply(follower, ['default'])
	follower.configure
	follower.start
		
	## Setup the data connections
	mapper.odometry2map.connect_to(calc_pose.odometry2map)
	odo.odometry_samples.connect_to(calc_pose.robot2odometry)
	mapper.envire_map.connect_to(traversability.mls_map)
	traversability.traversability_map.connect_to(planner.traversability_map)
	calc_pose.robot2map.connect_to(planner.start_pose_samples)
	calc_pose.robot2map.connect_to(follower.robot_pose)
	planner.trajectory.connect_to(follower.trajectory)
	planner.escape_trajectory.connect_to(follower.trajectory)
	follower.motion_command.connect_to(motion_controller.motion_command)
end

