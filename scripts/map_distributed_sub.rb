require 'orocos'
require 'vizkit'
require 'transformer/runtime'

include Orocos

## Initialize orocos ##
Orocos.initialize

## Execute the tasks ##
Orocos.run	'slam3d::DistributedPointcloudMapper' => 'mapper2',
			'slam3d::MLSMapProjector' => 'projector',
			'slam3d::Demultiplexer' => 'demultiplexer',
			'telemetry_provider::FIPASubscriber' => 'fipa_subscriber' do
	
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
	mapper = Orocos.name_service.get 'mapper2'
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
	mapper.robot_name = "R2"
	mapper.configure
	
	## Use multiagent communication from mapper1 to mapper2 ##
	demultiplexer = TaskContext.get 'demultiplexer'
	demultiplexer.configure

	subscriber = TaskContext.get 'fipa_subscriber'
	subscriber.configure
	
	publisher = TaskContext.get 'fipa_publisher'
	publisher.fipa_message.connect_to subscriber.fipa_message
	subscriber.telemetry_package.connect_to demultiplexer.telemetry_package
	
	demultiplexer.start
	subscriber.start

	
	if(!demultiplexer.createTelemetryOutputPort('vertices', '/slam3d/LocalizedPointcloud'))
		raise "telemetry.createTelemetryPort returned false on [/slam3d/LocalizedPointcloud]!"
	end
	
	mapper.external_in.connect_to demultiplexer.vertices
	mapper.cloud.connect_to projector.cloud, :type => :buffer, :size => 10
	
	## Start the tasks ##
	projector.start
	mapper.start
	
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
		projector.stop
		mapper.stop
		
		projector.cleanup
		mapper.cleanup
	end

end