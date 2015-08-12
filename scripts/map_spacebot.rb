require 'orocos'
require 'orocos/log'
require 'vizkit'
require 'transformer/runtime'
require_relative 'visualize'

include Orocos

log = Orocos::Log::Replay.open("/media/data/replays/sb_rh1")
log.use_sample_time = false

## Initialize orocos ##
Orocos.initialize

# track only needed ports
velodyne_ports = log.find_all_output_ports("/velodyne_lidar/MultilevelLaserScan", "laser_scans")
velodyne_ports.each do |port|
    port.tracked = true
end

## Execute the tasks ##
Orocos.run	'slam3d::ScanConverter' => 'converter',
			'slam3d::PointcloudFilter' => 'filter',
			'slam3d::PointcloudMapper' => 'mapper',
			'slam3d::MLSMapProjector' => 'projector' do

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
	mapper = Orocos.name_service.get 'mapper'
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
	mapper.configure

	## Connect ports with the task ##
	velodyne_ports.each do |port|
		port.connect_to converter.scan, :type => :buffer, :size => 10
	end
	converter.cloud.connect_to    filter.cloud_in,  :type => :buffer, :size => 10
	filter.cloud_out.connect_to   mapper.scan,      :type => :buffer, :size => 10
	mapper.cloud.connect_to projector.cloud, :type => :buffer, :size => 10

	## Setup the transformer ##
	Orocos.transformer.load_conf("transforms.rb")
	Orocos.transformer.setup(mapper)

	## Start the tasks ##
	mapper.start
	filter.start
	converter.start
	projector.start

	Vizkit.control log

	visualize()

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
