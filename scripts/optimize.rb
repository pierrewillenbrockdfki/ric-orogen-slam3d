require 'orocos'

include Orocos
Orocos.initialize

mapper = Orocos.name_service.get 'mapper'
mapper.optimize
