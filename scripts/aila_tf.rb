static_transform Eigen::Vector3.new(-0.185, 0.0, 0.11),
                 Eigen::Quaternion.from_angle_axis(3.141592654, Eigen::Vector3.UnitZ),
	             "laser2" => "body"

static_transform Eigen::Vector3.new(0.185, 0.0, 0.11),
	"laser1" => "body"

#dynamic_transform "aila_rover.odometry", "body" => "odometry"
#dynamic_transform "mapper.map2odometry", "odometry" => "world_osg"

dynamic_transform "mapper.map2robot", "body" => "world_osg"
