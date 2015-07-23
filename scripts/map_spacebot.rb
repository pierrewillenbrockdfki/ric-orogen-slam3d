require 'orocos'
require 'orocos/log'
require 'vizkit'

include Orocos

log = Orocos::Log::Replay.open("/home/dfki.uni-bremen.de/skasperski/Robotics/samples/sb_rh1")
log.use_sample_time = false

## Initialize orocos ##
Orocos.initialize

# track only needed ports
velodyne_ports = log.find_all_output_ports("/velodyne_lidar/MultilevelLaserScan", "laser_scans")
velodyne_ports.each do |port|
    port.tracked = true
end

odometry_ports = log.find_all_output_ports("/base/samples/RigidBodyState_m", "odometry_samples")
odometry_ports.each do |port|
    port.tracked = true
end

log.transformer_broadcaster.track(false)
log.transformer_broadcaster.rename("foo")

## Execute the tasks ##
Orocos.run	'slam3d::convert_scan' => 'converter',
			'slam3d::indoor_filter' => 'filter',
			'slam3d::PointcloudMapper' => 'mapper',
			'slam3d::mls_renderer' => 'projector' do

	## Configure the scan converter ##
	converter = Orocos.name_service.get 'converter'
	converter.configure
		
	## Configure the pointcloud filter ##
	filter = Orocos.name_service.get 'filter'
	filter.min_distance = 0
	filter.max_distance = 10
	filter.min_height = -2.0
	filter.max_height = 0.5
	filter.configure

	## Configure the projector
	projector = Orocos.name_service.get 'projector'
	projector.size_x = 50
	projector.size_y = 50
	projector.offset_x = -10
	projector.offset_y = -40
	projector.min_z = -5;
	projector.max_z = 5;
	projector.resolution = 0.1
	projector.configure

	## Configure the mapper ##
	mapper = Orocos.name_service.get 'mapper'
	mapper.scan_resolution = 0.1
	mapper.map_resolution = 0.05
	mapper.neighbor_radius = 3.0
	mapper.min_translation = 0.25
	mapper.min_rotation = 0.05
	mapper.use_odometry = true
	mapper.add_odometry_edges = true
	mapper.log_level = 1
	
	mapper.gicp_config do |c|
		c.max_correspondence_distance = 1.0
		c.max_fitness_score = 20
	end
	
	mapper.configure

	## Connect ports with the task ##
	velodyne_ports.each do |port|
		port.connect_to converter.scan, :type => :buffer, :size => 10
	end
	converter.cloud.connect_to    filter.cloud_in,  :type => :buffer, :size => 10
	filter.cloud_out.connect_to   mapper.scan,      :type => :buffer, :size => 10
	odometry_ports.each do |port|
		port.connect_to mapper.odometry, :type => :buffer, :size => 10
	end
	mapper.cloud.connect_to projector.cloud, :type => :buffer, :size => 10

	## Start the tasks ##
	mapper.start
	filter.start
	converter.start
	projector.start

	Vizkit.control log
	
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

	envireViz = Vizkit.display projector.envire_map
	mlsviz = envireViz.getVisualizer("envire::MLSVisualization")
	mlsviz.cycle_height_color = 1
	mlsviz.cycle_color_interval = 3
	
	pclviz = envireViz.getVisualizer("envire::PointcloudVisualization")
	pclviz.enabled = 0

	begin
		Vizkit.exec
	rescue Interrupt => e
		mapper.stop
		filter.stop
		converter.stop
		projector.stop
		
		mapper.cleanup
		filter.cleanup
		converter.cleanup
		projector.cleanup
	end
end
