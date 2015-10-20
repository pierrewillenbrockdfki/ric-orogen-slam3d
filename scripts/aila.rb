require 'orocos'
require 'orocos/log'
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
Orocos.run 'slam3d::LineScanConverter' => ['rear_converter', 'front_converter'] do

	############################################################################
	## Configure the scan converter ##
	f_converter = Orocos.name_service.get 'front_converter'
	r_converter = Orocos.name_service.get 'rear_converter'
	scan_ports.each do |port|
		if port.task.name == "aila-control/hokuyo_front"
			port.connect_to f_converter.scan
		elsif port.task.name == "aila-control/hokuyo_rear"
			port.connect_to r_converter.scan			
		end
	end
	f_converter.configure
	f_converter.start
	r_converter.configure
	r_converter.start
	
	############################################################################
	## Start the replay ##
	Vizkit.control log

	Vizkit.display f_converter.cloud
	Vizkit.display r_converter.cloud

	begin
		Vizkit.exec
	rescue Interrupt => e
		converter.stop
		converter.cleanup
	end
end