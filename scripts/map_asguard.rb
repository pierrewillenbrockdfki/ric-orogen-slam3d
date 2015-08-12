require 'orocos'
require 'orocos/log'
require 'vizkit'
require 'transformer/runtime'
require_relative 'visualize'

include Orocos

log = Orocos::Log::Replay.open("/media/data/replays/asguard_01")
log.use_sample_time = false

## Initialize orocos ##
Orocos.initialize

# track only needed ports
pcl_ports = log.find_all_output_ports("/base/samples/Pointcloud_m", "pointcloud")
pcl_ports.each do |port|
	port.tracked = true
end

odometry_ports = log.find_all_output_ports("/base/samples/RigidBodyState_m", "odometry_samples")
odometry_ports.each do |port|
    port.tracked = true
end

## Execute the task 'message_producer::Task' ##
Orocos.run 'slam3d::PointcloudFilter' => 'filter',
           'slam3d::PointcloudMapper' => 'mapper',
           'slam3d::MLSMapProjector' => 'projector' do

	## Configure the pointcloud filter ##
	filter = Orocos.name_service.get 'filter'
	filter.min_distance = 1.0
	filter.max_distance = 15
	filter.min_height = -5.0
	filter.max_height = 5.0
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
	mapper = Orocos.name_service.get 'mapper'
	mapper.scan_resolution = 0
	mapper.map_resolution = 0.05
	mapper.map_outlier_radius = 0.1
	mapper.map_outlier_neighbors = 5
	mapper.neighbor_radius = 3.0
	mapper.min_translation = 0.01
	mapper.min_rotation = 0.01
	mapper.use_odometry = true
	mapper.add_odometry_edges = true
	mapper.odometry_time_tolerance = 500
	mapper.log_level = 1
	mapper.gicp_config do |c|
		c.max_correspondence_distance = 1.0
		c.max_fitness_score = 50
	end
	mapper.configure
  
	## Connect ports with the task ##
	pcl_ports.each do |port|
		port.connect_to filter.cloud_in, :type => :buffer, :size => 10
	end
	filter.cloud_out.connect_to   mapper.scan,      :type => :buffer, :size => 10
	mapper.cloud.connect_to projector.cloud, :type => :buffer, :size => 10

	## Setup the transformer ##
	Orocos.transformer.load_conf("asguard_tf.rb")
	Orocos.transformer.setup(mapper)

	## Start the tasks ##
	filter.start
	mapper.start
	projector.start

	Vizkit.control log

	visualize()

	begin
		Vizkit.exec
	rescue Interrupt => e
		mapper.stop
		mapper.cleanup
		filter.stop
		filter.cleanup
		projector.stop
		projector.cleanup
		
	end

end
