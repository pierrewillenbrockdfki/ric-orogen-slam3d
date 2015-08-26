require 'orocos'
require 'orocos/log'
require 'vizkit'
require 'transformer/runtime'

include Orocos

log = Orocos::Log::Replay.open("/media/data/replays/sb_rh1")
log.use_sample_time = false

## Initialize orocos ##
Orocos.initialize

robot_name = "R1"
port_name = "#{robot_name}-sensors"

# track only needed ports
velodyne_ports = log.find_all_output_ports("/velodyne_lidar/MultilevelLaserScan", "laser_scans")
velodyne_ports.each do |port|
    port.tracked = true
end

## Execute the tasks ##
Orocos.run	'slam3d::ScanConverter' => 'converter',
			'slam3d::PointcloudFilter' => 'filter',
			'slam3d::DistributedPointcloudMapper' => 'mapper1',
			'slam3d::Multiplexer' => 'multiplexer',
			'fipa_services::MessageTransportTask' => 'transport1',
			'telemetry_provider::FIPAPublisher' => 'fipa_publisher' do

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

	## Configure the mapper ##
	mapper = Orocos.name_service.get 'mapper1'
	mapper.scan_resolution = 0.1
	mapper.map_resolution = 0.05
	mapper.map_outlier_radius = 0.1
	mapper.map_outlier_neighbors = 5
	mapper.neighbor_radius = 2.0
	mapper.min_translation = 0.25
	mapper.min_rotation = 0.05
	mapper.use_odometry = false
	mapper.add_odometry_edges = true
	mapper.log_level = 1
	
	mapper.gicp_config do |c|
		c.max_correspondence_distance = 1.0
		c.max_fitness_score = 20
	end
	
	mapper.scan_period = 0.1
	mapper.robot_frame = "body"
	mapper.robot_name = "#{robot_name}"
	mapper.configure

	## Connect ports with the task ##
	velodyne_ports.each do |port|
		port.connect_to converter.scan, :type => :buffer, :size => 10
	end
	converter.cloud.connect_to    filter.cloud_in,  :type => :buffer, :size => 10
	filter.cloud_out.connect_to   mapper.scan,      :type => :buffer, :size => 10
	
	## Use multiagent communication from mapper1 to mapper2 ##
	transport = Orocos.get 'transport1'
	transport.configure
	transport.start
	transport.addReceiver(port_name, true)
	
	multiplexer = Orocos.get 'multiplexer'
	multiplexer.configure
	multiplexer.start

	publisher = Orocos.get 'fipa_publisher'
	publisher.sender = port_name
	publisher.receiver = '.*-sensors'
	publisher.configure
	publisher.start

	multiplexer.telemetry_package.connect_to publisher.telemetry_package
	publisher.fipa_message.connect_to transport.letters

	if(!multiplexer.createTelemetryInputPort('vertices', '/slam3d/LocalizedPointcloud'))
		raise "multiplexer.createTelemetryPort returned false on [/slam3d/LocalizedPointcloud]!"
	end

	mapper.external_out.connect_to multiplexer.vertices
	
	## Setup the transformer ##
	Orocos.transformer.load_conf("transforms.rb")
	Orocos.transformer.setup(mapper)


	## Start the tasks ##
	filter.start
	converter.start
	mapper.start

	Vizkit.control log

	begin
		Vizkit.exec
	rescue Interrupt => e
		filter.stop
		converter.stop
		projector.stop
		mapper.stop
		
		filter.cleanup
		converter.cleanup
		mapper.cleanup
	end
end
