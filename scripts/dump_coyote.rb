require 'orocos'
require 'orocos/log'
require 'vizkit'
require 'transformer/runtime'

include Orocos

if not ARGV[0]
    puts "add a valid logfile folder as parameter"
    exit
end

log = Orocos::Log::Replay.open(ARGV)
log.use_sample_time = false

## Initialize orocos ##
Orocos.initialize

# track only needed ports
pcl_ports = log.find_all_output_ports("/base/samples/Pointcloud_m", "pointcloud")

## Execute the PCL-Dump Task ##
Orocos.run 'slam3d::dump_pcl' => 'dump' do
  ## Get the task context##
  dump = Orocos.name_service.get 'dump'
  # connect ports with the task
  pcl_ports.each do |port|
      port.connect_to dump.pcl, :type => :buffer, :size => 10
  end

  ## Start the tasks ##
  dump.configure
  dump.start
  
  Vizkit.control log
  begin
    Vizkit.exec
  rescue Interrupt => e
    dump.stop
    dump.cleanup
  end
  
end

