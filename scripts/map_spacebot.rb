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
#log.transformer_broadcaster.track(false)
#log.transformer_broadcaster.rename("foo")
velodyne_ports.each do |port|
    port.tracked = true
end

## Execute the task 'message_producer::Task' ##
Orocos.run	'slam3d::convert_scan' => 'converter',
			'slam3d::indoor_filter' => 'filter',
			'slam3d::PointcloudMapper' => 'mapper' do

	## Configure the scan converter ##
	converter = Orocos.name_service.get 'converter'
	converter.configure
		
	## Configure the pointcloud filter ##
	filter = Orocos.name_service.get 'filter'
	filter.configure

	## Configure the mapper ##
	mapper = Orocos.name_service.get 'mapper'
	mapper.scan_resolution = 0.1
	mapper.neighbor_radius = 1.0
	mapper.min_translation = 0.5;
	mapper.min_rotation = 0.1
	
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

	## Start the tasks ##
	mapper.start
	filter.start
	converter.start

	Vizkit.control log
#	Vizkit.display filter.cloud_out
	Vizkit.display mapper.cloud
	Vizkit.display mapper.map2robot
	begin
		Vizkit.exec
	rescue Interrupt => e
		mapper.stop
		filter.stop
		converter.stop
		
		mapper.cleanup
		filter.cleanup
		converter.cleanup
	end
  
#  Orocos.watch(converter)
end
