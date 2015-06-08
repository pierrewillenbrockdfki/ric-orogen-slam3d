require 'orocos'
require 'orocos/log'
require 'vizkit'
require 'transformer/runtime'

include Orocos

log = Orocos::Log::Replay.open("/home/dfki.uni-bremen.de/skasperski/Robotics/samples/20130709_velodyne_seekurjr_outdoor_track")
log.use_sample_time = false

## Initialize orocos ##
Orocos.initialize
#Orocos.transformer.load_conf('transforms.rb')

# track only needed ports
velodyne_ports = log.find_all_output_ports("/velodyne_lidar/MultilevelLaserScan", "laser_scans")
log.transformer_broadcaster.track(false)
log.transformer_broadcaster.rename("foo")
velodyne_ports.each do |port|
    port.tracked = true
end

## Execute the task 'message_producer::Task' ##
Orocos.run 'slam3d::Mapper' => 'mapper' do
  ## Get the task context##
  mapper = Orocos.name_service.get 'mapper'
  # connect ports with the task
  velodyne_ports.each do |port|
      port.connect_to mapper.scan, :type => :buffer, :size => 100
  end

  ## Start the tasks ##
#  Orocos.transformer.setup(mapper)
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
  
#  Orocos.watch(mapper)
end

