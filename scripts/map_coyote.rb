require 'orocos'
require 'orocos/log'
require 'vizkit'

include Orocos

log = Orocos::Log::Replay.open("/home/dfki.uni-bremen.de/skasperski/Robotics/samples/20150612_coyote3_rh5")
log.use_sample_time = false

## Initialize orocos ##
Orocos.initialize

# track only needed ports
pcl_ports = log.find_all_output_ports("/base/samples/Pointcloud_m", "pointcloud")
pcl_ports.each do |port|
    port.tracked = true
end

## Execute the task 'message_producer::Task' ##
Orocos.run 'slam3d::PointcloudMapper' => 'mapper' do
  ## Get the task context##
  mapper = Orocos.name_service.get 'mapper'
  # connect ports with the task
  pcl_ports.each do |port|
      port.connect_to mapper.scan, :type => :buffer, :size => 10
  end

  ## Start the tasks ##
  mapper.configure
  mapper.start
  
  Vizkit.control log
  Vizkit.display mapper
  begin
    Vizkit.exec
  rescue Interrupt => e
    mapper.stop
    mapper.cleanup
  end
  
end
