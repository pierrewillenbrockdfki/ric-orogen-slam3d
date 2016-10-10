require 'orocos'

unless ARGV.length >= 2
	puts "Usage: service <TASK> <SERVICE> <PARAM>"
	exit
end

include Orocos
Orocos.initialize

mapper = Orocos.name_service.get ARGV[0]

if(ARGV.length == 2)
    mapper.send(ARGV[1])
elsif(ARGV.length == 3)
    mapper.send(ARGV[1], ARGV[2])
end
