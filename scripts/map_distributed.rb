require 'orocos'
require 'orocos/log'
require 'vizkit'
require 'transformer/runtime'

include Orocos

log = Orocos::Log::Replay.open("/media/data/replays/sb_rh1")
log.use_sample_time = false

## Initialize orocos ##
Orocos.initialize
Orocos.load_typekit "slam3d"

# track only needed ports
velodyne_ports = log.find_all_output_ports("/velodyne_lidar/MultilevelLaserScan", "laser_scans")
velodyne_ports.each do |port|
    port.tracked = true
end

## Execute the tasks ##
Orocos.run	'slam3d::ScanConverter' => 'converter',
			'slam3d::PointcloudFilter' => 'filter',
			'slam3d::DistributedPointcloudMapper' => ['mapper1', 'mapper2'],
			'slam3d::MLSMapProjector' => 'projector',
			'slam3d::Multiplexer' => 'multiplexer',
			'slam3d::Demultiplexer' => 'demultiplexer',
			'telemetry_provider::FIPAPublisher' => 'fipa_publisher',
			'telemetry_provider::FIPASubscriber' => 'fipa_subscriber' do

	## Configure the scan converter ##
	converter = Orocos.name_service.get 'converter'
	converter.configure
		
	## Configure the pointcloud filter ##
	filter = Orocos.name_service.get 'filter'
	filter.min_distance = 0
	filter.max_distance = 15
	filter.min_height = -2.0
	filter.max_height = 1.0
	filter.configure

	## Configure the projector
	projector = Orocos.name_service.get 'projector'
	projector.size_x = 50
	projector.size_y = 50
	projector.offset_x = -10
	projector.offset_y = -40
	projector.min_z = -5;
	projector.max_z = 5;
	projector.resolution = 0.05
	projector.configure


	## Configure the mapper ##
	mapper1 = Orocos.name_service.get 'mapper1'
	mapper2 = Orocos.name_service.get 'mapper2'
	[mapper1, mapper2].each do |mapper|
		mapper.scan_resolution = 0.1
		mapper.map_resolution = 0.05
		mapper.map_outlier_radius = 0.1
		mapper.map_outlier_neighbors = 5
		mapper.neighbor_radius = 2.0
		mapper.min_translation = 0.25
		mapper.min_rotation = 0.05
		mapper.use_odometry = true
		mapper.add_odometry_edges = true
		mapper.log_level = 1
		
		mapper.gicp_config do |c|
			c.max_correspondence_distance = 1.0
			c.max_fitness_score = 20
		end
		
		mapper.scan_period = 0.1
		mapper.robot_frame = "body"
	end
	
	mapper1.robot_name = "R1"
	mapper2.robot_name = "R2"
	mapper1.configure
	mapper2.configure

	## Connect ports with the task ##
	velodyne_ports.each do |port|
		port.connect_to converter.scan, :type => :buffer, :size => 10
	end
	converter.cloud.connect_to    filter.cloud_in,  :type => :buffer, :size => 10
	filter.cloud_out.connect_to   mapper1.scan,      :type => :buffer, :size => 10
	
	## Use multiagent communication from mapper1 to mapper2 ##
	
	multiplexer = TaskContext.get 'multiplexer'
	multiplexer.configure

	demultiplexer = TaskContext.get 'demultiplexer'
	demultiplexer.configure

	publisher = TaskContext.get 'fipa_publisher'
	publisher.sender = 'publisher'
	publisher.receiver = 'subscriber'
	publisher.configure

	subscriber = TaskContext.get 'fipa_subscriber'
	subscriber.configure

	multiplexer.telemetry_package.connect_to publisher.telemetry_package
	publisher.fipa_message.connect_to subscriber.fipa_message

	subscriber.telemetry_package.connect_to demultiplexer.telemetry_package

	multiplexer.start
	demultiplexer.start
	publisher.start
	subscriber.start

# /slam3d/LocalizedPointcloud
	if(!multiplexer.createTelemetryInputPort('vertices', '/slam3d/LocalizedPointcloud'))
		raise "multiplexer.createTelemetryPort returned false on [/slam3d/LocalizedPointcloud]!"
	end

	if(!demultiplexer.createTelemetryOutputPort('vertices', '/slam3d/LocalizedPointcloud'))
		raise "telemetry.createTelemetryPort returned false on [/slam3d/LocalizedPointcloud]!"
	end
	
	mapper1.external_out.connect_to multiplexer.vertices
	mapper2.external_in.connect_to demultiplexer.vertices
	
	##########################################################
	
	mapper2.cloud.connect_to projector.cloud, :type => :buffer, :size => 10

	## Setup the transformer ##
	Orocos.transformer.load_conf("transforms.rb")
	Orocos.transformer.setup(mapper1)
	Orocos.transformer.setup(mapper2)


	## Start the tasks ##
	filter.start
	converter.start
	projector.start
	mapper1.start
	mapper2.start

	Vizkit.control log
	
	# Show envire
	projector = Orocos.name_service.get 'projector'
	envireViz = Vizkit.display projector.envire_map
	mlsviz = envireViz.getVisualizer("envire::MLSVisualization")
	mlsviz.cycle_height_color = 1
	mlsviz.cycle_color_interval = 3
	
	pclviz = envireViz.getVisualizer("envire::PointcloudVisualization")
	pclviz.enabled = 0

	begin
		Vizkit.exec
	rescue Interrupt => e
		filter.stop
		converter.stop
		projector.stop
		mapper1.stop
		mapper2.stop
		
		filter.cleanup
		converter.cleanup
		projector.cleanup
		mapper1.cleanup
		mapper2.cleanup
	end
end
