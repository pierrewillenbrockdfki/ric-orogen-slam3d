def visualize()
	# Show robot pose
	rbsPoseViz = Vizkit.default_loader.RigidBodyStateVisualization
	rbs_pose = Orocos::Async.proxy('mapper')
	rbs_pose.port('map2robot').connect_to do |data,_|
		rbsPoseViz.updateData(data)
	end

	# Show odometry drift
	rbsDriftViz = Vizkit.default_loader.RigidBodyStateVisualization
	rbs_drift = Orocos::Async.proxy('mapper')
	rbs_drift.port('map2odometry').connect_to do |data,_|
		rbsDriftViz.updateData(data)
	end
	
	# Show envire
	projector = Orocos.name_service.get 'projector'
	envireViz = Vizkit.display projector.envire_map
	mlsviz = envireViz.getVisualizer("envire::MLSVisualization")
	mlsviz.cycle_height_color = 1
	mlsviz.cycle_color_interval = 3
	
	pclviz = envireViz.getVisualizer("envire::PointcloudVisualization")
	pclviz.enabled = 0

end
