require 'orocos'
require 'orocos/log'
require 'vizkit'
require_relative 'visualize'

include Orocos

log = Orocos::Log::Replay.open("/home/dfki.uni-bremen.de/skasperski/Robotics/samples/seekur_track")
log.use_sample_time = false

## Initialize orocos ##
Orocos.initialize

# track only needed ports
velodyne_ports = log.find_all_output_ports("/velodyne_lidar/MultilevelLaserScan", "laser_scans")
velodyne_ports.each do |port|
    port.tracked = true
end

odometry_ports = log.find_all_output_ports("/base/samples/RigidBodyState_m", "robot_pose")
odometry_ports.each do |port|
    port.tracked = true
end

## Execute the task 'message_producer::Task' ##
Orocos.run	'slam3d::PointcloudMapper' => 'mapper',
           'slam3d::ScanConverter' => 'converter',
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
	## Configure the mapper ##
	mapper = Orocos.name_service.get 'mapper'
	mapper.scan_resolution = 0.25
	mapper.map_resolution = 0.1
	mapper.neighbor_radius = 1.0
	mapper.min_translation = 0.3
	mapper.min_rotation = 0.1
	mapper.use_odometry = true
	mapper.add_odometry_edges = false
	mapper.log_level = 1
	
	mapper.gicp_config do |c|
		c.max_correspondence_distance = 1.0
		c.max_fitness_score = 20
		c.point_cloud_density = 0.5
		c.maximum_iterations = 50
	end
	
	converter.cloud.connect_to mapper.scan,      :type => :buffer, :size => 10
	odometry_ports.each do |port|
		port.connect_to mapper.odometry, :type => :buffer, :size => 10
	end
	
	mapper.configure
	mapper.start

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
