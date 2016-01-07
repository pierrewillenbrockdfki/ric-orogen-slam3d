require 'orocos'
require 'orocos/log'
require 'rock/bundle'
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
Orocos.run 'laserscan_fusion::MergeTwoScans' => 'fusion',
           'slam3d::PointcloudMapper' => 'mapper',
           'traversability::Simple' => 'traversability',
		   'slam3d::MLSMapProjector' => 'projector' do

	Bundles.transformer.start_broadcaster do

		############################################################################
		## Configure the scan fusion ##
		fusion = Orocos.name_service.get 'fusion'
		scan_ports.each do |port|
			if port.task.name == "aila-control/hokuyo_front"
				port.connect_to fusion.scan1
			elsif port.task.name == "aila-control/hokuyo_rear"
				port.connect_to fusion.scan2			
			end
		end
		Orocos.transformer.load_conf("aila_tf.rb")
		Orocos.transformer.setup(fusion)
		fusion.configure
	
		############################################################################
		## Configure the mapper ##
		mapper = Orocos.name_service.get 'mapper'
		mapper.scan_resolution = 0.01
		mapper.map_resolution = 0.01
		mapper.map_outlier_radius = 0.2
		mapper.map_outlier_neighbors = 2
		mapper.neighbor_radius = 2.0
		mapper.min_translation = 0.1
		mapper.min_rotation = 0.25
		mapper.use_odometry = false
		mapper.add_odometry_edges = true
		mapper.scan_period = 0.1
		mapper.log_level = 1
		mapper.map_frame = "world_osg"
		mapper.robot_frame = "body"
		mapper.robot_name = "Aila"
	
		mapper.gicp_config do |c|
			c.max_correspondence_distance = 0.25
			c.max_fitness_score = 20
			c.point_cloud_density = 0.01
			c.maximum_iterations = 50
		end
		
		mapper.configure
	
		## Configure the projector
		projector = Orocos.name_service.get 'projector'
		projector.size_x = 50
		projector.size_y = 50
		projector.offset_x = -25
		projector.offset_y = -25
		projector.min_z = -5;
		projector.max_z = 5;
		projector.resolution = 0.05
		projector.configure

		## Configure traversability
		traversability = Orocos.name_service.get 'traversability'
		traversability.mls_id = "/slam3d-mls"
		traversability.env_name = "/traversability/"
		traversability.env_save_path = ""
		traversability.map_extents = []
		traversability.map_max_extent = 1000
		traversability.mls_id = "/slam3d-mls"
		traversability.traversability_conf do |c|
			c.maximum_slope = 0.65
			c.class_count = 10
			c.min_width = 0.0
			c.ground_clearance = 0.25 #0.2
			c.obstacle_clearance = 0.0
		end
		
		traversability.configure

		fusion.cloud.connect_to mapper.scan,              :type => :buffer, :size => 10
		mapper.cloud.connect_to projector.cloud,          :type => :buffer, :size => 10
		projector.envire_map.connect_to traversability.mls_map

		############################################################################
	
		fusion.start
		mapper.start
		projector.start
		traversability.start
	
		############################################################################
		## Start the replay ##
		Vizkit.control log

		Vizkit.display fusion.cloud
#		Vizkit.display mapper.cloud
		Vizkit.display traversability.traversability_map

		begin
			Vizkit.exec
		rescue Interrupt => e
			mapper.stop
			mapper.cleanup
			fusion.stop
			fusion.cleanup
			projector.stop
			projector.cleanup
			traversability.cleanup
			traversability.stop
		end

	end
end
