require 'orocos'
require 'orocos/log'
require 'vizkit'
require 'transformer/runtime'
require_relative 'visualize'

include Orocos

log = Orocos::Log::Replay.open("/media/data/replays/aila_01")
log.use_sample_time = false

## Initialize orocos ##
Orocos.initialize

# track only needed ports
scan_ports = log.find_all_output_ports("/base/samples/LaserScan", "scans")
scan_ports.each do |port|
	port.tracked = true
end

odometry_ports = log.find_all_output_ports("/base/samples/RigidBodyState_m", "odometry")
odometry_ports.each do |port|
	port.tracked = true
end

## Execute the task 'message_producer::Task' ##
Orocos.run 'slam3d::LineScanConverter' => ['rear_converter', 'front_converter'],
           'slam3d::PointcloudMapper' => 'mapper' do

	############################################################################
	## Configure the scan converter ##
	f_converter = Orocos.name_service.get 'front_converter'
	r_converter = Orocos.name_service.get 'rear_converter'
	scan_ports.each do |port|
		if port.task.name == "aila-control/hokuyo_front"
			port.connect_to f_converter.scan
		elsif port.task.name == "aila-control/hokuyo_rear"
			port.connect_to r_converter.scan			
		end
	end
	f_converter.configure
	f_converter.start
#	r_converter.configure
#	r_converter.start
	
	############################################################################
	## Configure the mapper ##
	mapper = Orocos.name_service.get 'mapper'
	mapper.scan_resolution = 0.001
	mapper.map_resolution = 0.001
	mapper.map_outlier_radius = 0.2
	mapper.map_outlier_neighbors = 0
	mapper.neighbor_radius = 2.0
	mapper.min_translation = 0.025
	mapper.min_rotation = 0.2
	mapper.use_odometry = false
	mapper.add_odometry_edges = true
	mapper.log_level = 1
	
	mapper.gicp_config do |c|
		c.max_correspondence_distance = 1.0
		c.max_fitness_score = 20
		c.point_cloud_density = 0.01
		c.maximum_iterations = 50
	end
	
	f_converter.cloud.connect_to mapper.scan,      :type => :buffer, :size => 10
	
	mapper.scan_period = 0.1
	mapper.robot_frame = "body"
	mapper.robot_name = "Aila"
	
	############################################################################
	## Setup the transformer ##
#	Orocos.transformer.load_conf("aila_tf.rb")
#	Orocos.transformer.setup(mapper)
	
	mapper.configure
	mapper.start
	
	############################################################################
	## Start the replay ##
	Vizkit.control log

	Vizkit.display f_converter.cloud
#	Vizkit.display r_converter.cloud
	Vizkit.display mapper.cloud

	begin
		Vizkit.exec
	rescue Interrupt => e
		converter.stop
		converter.cleanup
	end
end
