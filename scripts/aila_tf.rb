static_transform Eigen::Vector3.new(-0.185, 0.0, 0.11),
	"hokuyo_rear" => "body"

static_transform Eigen::Vector3.new(0.185, 0.0, 0.11),
	"hokuyo_front" => "body"

#static_transform Eigen::Vector3.new(0, 0, 0), "body" => "odometry"
dynamic_transform "aila_rover.odometry", "body" => "odometry"
