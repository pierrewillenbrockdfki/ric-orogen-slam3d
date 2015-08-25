require 'orocos'

unless ARGV.length == 2
	puts "Usage: service <TASK> <SERVICE>"
	exit
end

include Orocos
Orocos.initialize

mapper = Orocos.name_service.get ARGV[0]
mapper.send(ARGV[1])
