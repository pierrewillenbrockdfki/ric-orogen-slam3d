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
			'slam3d::PointcloudMapper' => 'mapper' do

  ## Get the task context ##
  converter = Orocos.name_service.get 'converter'
  mapper = Orocos.name_service.get 'mapper'

  ## Connect ports with the task ##
  velodyne_ports.each do |port|
      port.connect_to converter.scan, :type => :buffer, :size => 100
  end
  converter.cloud.connect_to mapper.scan, :type => :buffer, :size => 100

  ## Start the tasks ##
  converter.configure
  mapper.configure
  
  mapper.start
  converter.start
  
  Vizkit.control log
  Vizkit.display converter.cloud
  Vizkit.display mapper.cloud
  begin
    Vizkit.exec
  rescue Interrupt => e
    mapper.stop
	converter.stop
    mapper.cleanup
	converter.cleanup
  end
  
#  Orocos.watch(converter)
end
