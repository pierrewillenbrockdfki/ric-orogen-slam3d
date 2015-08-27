require 'orocos'
require 'orocos/log'
require 'vizkit'
require 'transformer/runtime'
require_relative 'visualize'

include Orocos

log = Orocos::Log::Replay.open("/media/data/samples/20150728-1232_slam_error")
log.use_sample_time = false

## Initialize orocos ##
Orocos.initialize

# track only needed ports
velodyne_ports = log.find_all_output_ports("/velodyne_lidar/MultilevelLaserScan", "laser_scans")
velodyne_ports.each do |port|
    port.tracked = true
end

odometry_ports = log.find_all_output_ports("/base/samples/RigidBodyState_m", "car_state")
odometry_ports.each do |port|
    port.tracked = true
end

log.transformer_broadcaster.track(false)
log.transformer_broadcaster.rename("foo")

## Execute the task 'message_producer::Task' ##
Orocos.run 'slam3d::PointcloudMapper' => 'mapper',
           'slam3d::ScanConverter' => 'converter',
           'slam3d::PointcloudFilter' => 'filter',
           'slam3d::MLSMapProjector' => 'projector' do

############################################################################
	## Configure the scan converter ##
	converter = Orocos.name_service.get 'converter'
	velodyne_ports.each do |port|
		port.connect_to converter.scan, :type => :buffer, :size => 10
	end
	converter.configure
	converter.start

############################################################################
	## Configure the pointcloud filter ##
	filter = Orocos.name_service.get 'filter'
	filter.min_distance = 0
	filter.max_distance = 15
	filter.min_height = -3.0
	filter.max_height = 1.0

	converter.cloud.connect_to    filter.cloud_in,  :type => :buffer, :size => 10
	filter.configure
	filter.start

############################################################################
	## Configure the mapper ##
	mapper = Orocos.name_service.get 'mapper'
	mapper.scan_resolution = 0.1
	mapper.map_resolution = 0.1
	mapper.map_outlier_radius = 0.2
	mapper.map_outlier_neighbors = 1
	mapper.neighbor_radius = 2.0
	mapper.min_translation = 0.3
	mapper.min_rotation = 0.2
	mapper.use_odometry = false
	mapper.add_odometry_edges = true
	mapper.log_level = 1
	
	mapper.gicp_config do |c|
		c.max_correspondence_distance = 1.0
		c.max_fitness_score = 20
		c.point_cloud_density = 0.5
		c.maximum_iterations = 50
	end
	
	filter.cloud_out.connect_to mapper.scan,      :type => :buffer, :size => 10
	
	mapper.scan_period = 0.1
	mapper.robot_frame = "body"
	mapper.robot_name = "EO-2"
	mapper.configure
	mapper.start

############################################################################
	## Setup the transformer ##
	Orocos.transformer.load_conf("eo2_tf.rb")
	Orocos.transformer.setup(mapper)

############################################################################
	## Projector used to create MLS from Pointcloud
	projector = Orocos.name_service.get 'projector'
	projector.size_x = 100
	projector.size_y = 100
	projector.offset_x = -20
	projector.offset_y = -80
	projector.min_z = -10;
	projector.max_z = 10;
	projector.resolution = 0.1	
	mapper.cloud.connect_to projector.cloud, :type => :buffer, :size => 10
	projector.configure
	projector.start

############################################################################
	Vizkit.control log

	visualize()

	begin
		Vizkit.exec
	rescue Interrupt => e
		mapper.stop
		converter.stop
		projector.stop
		
		mapper.cleanup
		converter.cleanup
		projector.cleanup
	end

end
