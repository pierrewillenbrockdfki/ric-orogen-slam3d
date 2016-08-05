#include "PointcloudMapper.hpp"
#include "RockOdometry.hpp"
#include "BaseLogger.hpp"

#include <base/samples/Pointcloud.hpp>
#include <base/samples/RigidBodyState.hpp>

#include <slam3d/BoostMapper.hpp>
#include <slam3d/FileLogger.hpp>
#include <slam3d/G2oSolver.hpp>

#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <pcl/common/transforms.h>

#include <envire/Orocos.hpp>

using namespace slam3d;

PointcloudMapper::PointcloudMapper(std::string const& name)
    : PointcloudMapperBase(name)
{
	mCurrentOdometry = Eigen::Affine3d::Identity();
}

PointcloudMapper::PointcloudMapper(std::string const& name, RTT::ExecutionEngine* engine)
    : PointcloudMapperBase(name, engine)
{
	mCurrentOdometry = Eigen::Affine3d::Identity();
}

PointcloudMapper::~PointcloudMapper()
{
}

bool PointcloudMapper::optimize()
{
	mLogger->message(INFO, "Requested global optimization.");
	try
	{
		boost::unique_lock<boost::shared_mutex> guard(mGraphMutex);
		if(mMapper->optimize())
		{
			sendRobotPose();
			mRebuildMap = true;
			return true;
		}
	}catch (boost::lock_error &e)
	{
		mLogger->message(WARNING, "Could not access the pose graph for optimization! Is another operation still running?");
	}
	return false;
}

bool PointcloudMapper::generate_cloud()
{
	// Publish accumulated cloud
	mLogger->message(INFO, "Requested pointcloud generation.");
	VertexObjectList vertices = mMapper->getVertexObjectsFromSensor(mPclSensor->getName());
	boost::thread projThread(&PointcloudMapper::sendPointcloud, this, vertices);
	return true;
}

bool PointcloudMapper::generate_map()
{
	mLogger->message(INFO, "Requested map generation.");
	if(mRebuildMap)
	{
		VertexObjectList vertices = mMapper->getVertexObjectsFromSensor(mPclSensor->getName());
		boost::thread projThread(&PointcloudMapper::rebuildMap, this, vertices);
	}else
	{
		sendMap();
	}
	return true;
}

bool PointcloudMapper::write_graph()
{
	mMapper->writeGraphToFile("slam3d_graph");
	return true;
}

PointCloud::Ptr PointcloudMapper::buildPointcloud(const VertexObjectList& vertices)
{
	timeval start = mClock->now();
	PointCloud::Ptr accumulated;

	boost::shared_lock<boost::shared_mutex> guard(mGraphMutex);
	accumulated = mPclSensor->getAccumulatedCloud(vertices);

	PointCloud::Ptr cleaned = mPclSensor->removeOutliers(accumulated, mMapOutlierRadius, mMapOutlierNeighbors);
	PointCloud::Ptr downsampled = mPclSensor->downsample(cleaned, mMapResolution);

	timeval finish = mClock->now();
	int duration = finish.tv_sec - start.tv_sec;
	mLogger->message(INFO, (boost::format("Generated Pointcloud from %1% scans in %2% seconds.") % vertices.size() % duration).str());
	return downsampled;
}

void PointcloudMapper::sendPointcloud(const VertexObjectList& vertices)
{
	PointCloud::Ptr accCloud;
	try
	{
		accCloud = buildPointcloud(vertices);
	}catch (boost::lock_error &e)
	{
		mLogger->message(ERROR, "Could not access the pose graph to build Pointcloud!");
		return;
	}
	base::samples::Pointcloud mapCloud;
	for(PointCloud::iterator it = accCloud->begin(); it < accCloud->end(); ++it)
	{
		base::Vector3d vec;
		vec[0] = it->x;
		vec[1] = it->y;
		vec[2] = it->z;
		mapCloud.points.push_back(vec);
	}
	mapCloud.time = base::Time::fromMicroseconds(accCloud->header.stamp);
	_cloud.write(mapCloud);	
}

void PointcloudMapper::handleNewScan(const VertexObject& scan)
{
	addScanToMap(scan);
}

void PointcloudMapper::addScanToMap(const VertexObject& scan)
{
	PointCloudMeasurement::Ptr m = boost::dynamic_pointer_cast<PointCloudMeasurement>(scan.measurement);
	if(!m)
	{
		mLogger->message(WARNING, "Vertex given to addScanToMap is not a Pointcloud!");
		return;
	}
	
	PointCloud::ConstPtr pcl = m->getPointCloud();
	Transform sensor_pose = scan.corrected_pose * m->getSensorPose();
	for(PointCloud::const_iterator it = pcl->begin(); it != pcl->end(); ++it)
	{
		Eigen::Vector3d p = sensor_pose * Eigen::Vector3d(it->x, it->y, it->z);
		if(p[2] >= mGridMinZ && p[2] <= mGridMaxZ)
		{
			envire::MLSGrid::SurfacePatch patch( p[2], 0.1 );
			mMultiLayerMap->update(Eigen::Vector2d(p[0], p[1]) , patch );
		}
	}
}

void PointcloudMapper::rebuildMap(const VertexObjectList& vertices)
{
	mMultiLayerMap->clear();
	boost::shared_lock<boost::shared_mutex> guard(mGraphMutex);
	for(VertexObjectList::const_iterator v = vertices.begin(); v != vertices.end(); ++v)
	{
		addScanToMap(*v);
	}
	mRebuildMap = false;
	sendMap();
}

void PointcloudMapper::sendMap()
{
	// Publish the MLS-Map	
	envire::OrocosEmitter emitter(&mEnvironment, _envire_map);
	emitter.flush();
}	

bool PointcloudMapper::setLog_level(boost::int32_t value)
{
	switch(value)
	{
	case 4:
		mLogger->setLogLevel(FATAL);
		break;
	case 3:
		mLogger->setLogLevel(ERROR);
		break;
	case 2:
		mLogger->setLogLevel(WARNING);
		break;
	case 0:
		mLogger->setLogLevel(DEBUG);
		break;
	default:
		mLogger->setLogLevel(INFO);
	}
	return true;
}

bool PointcloudMapper::configureHook()
{	
	if (! PointcloudMapperBase::configureHook())
		return false;
		
	mClock = new Clock();
	switch(_log_type)
	{
	case 0:
		mLogger = new Logger(*mClock);
		break;
	case 1:
		mLogger = new BaseLogger();
		break;
	case 2:
		mLogger = new FileLogger(*mClock, "slam3d.log");
		break;
	default:
		mLogger = new Logger(*mClock);
		mLogger->message(WARNING, "Invalid logger type, using standard logger.");
	}

	setLog_level(_log_level);

	mLogger->message(INFO, "=== Configure PointCloudMapper ===");

	mPclSensor = new PointCloudSensor("pointcloud", mLogger, Transform::Identity());
	GICPConfiguration conf = _gicp_config.get();
	mPclSensor->setFineConfiguaration(conf);
	mLogger->message(INFO, " = GICP - Parameters =");
	mLogger->message(INFO, (boost::format("correspondence_randomness:    %1%") % conf.correspondence_randomness).str());
	mLogger->message(INFO, (boost::format("euclidean_fitness_epsilon:    %1%") % conf.euclidean_fitness_epsilon).str());
	mLogger->message(INFO, (boost::format("max_correspondence_distance:  %1%") % conf.max_correspondence_distance).str());
	mLogger->message(INFO, (boost::format("max_fitness_score:            %1%") % conf.max_fitness_score).str());
	mLogger->message(INFO, (boost::format("max_sensor_distance:          %1%") % conf.max_sensor_distance).str());
	mLogger->message(INFO, (boost::format("maximum_iterations:           %1%") % conf.maximum_iterations).str());
	mLogger->message(INFO, (boost::format("maximum_optimizer_iterations: %1%") % conf.maximum_optimizer_iterations).str());
	mLogger->message(INFO, (boost::format("orientation_sigma:            %1%") % conf.orientation_sigma).str());
	mLogger->message(INFO, (boost::format("point_cloud_density:          %1%") % conf.point_cloud_density).str());
	mLogger->message(INFO, (boost::format("position_sigma:               %1%") % conf.position_sigma).str());
	mLogger->message(INFO, (boost::format("rotation_epsilon:             %1%") % conf.rotation_epsilon).str());
	mLogger->message(INFO, (boost::format("transformation_epsilon:       %1%") % conf.transformation_epsilon).str());

	conf = _gicp_coarse_config.get();
	mPclSensor->setCoarseConfiguaration(conf);
	mLogger->message(INFO, " = GICP - Coarse Parameters =");
	mLogger->message(INFO, (boost::format("correspondence_randomness:    %1%") % conf.correspondence_randomness).str());
	mLogger->message(INFO, (boost::format("euclidean_fitness_epsilon:    %1%") % conf.euclidean_fitness_epsilon).str());
	mLogger->message(INFO, (boost::format("max_correspondence_distance:  %1%") % conf.max_correspondence_distance).str());
	mLogger->message(INFO, (boost::format("max_fitness_score:            %1%") % conf.max_fitness_score).str());
	mLogger->message(INFO, (boost::format("max_sensor_distance:          %1%") % conf.max_sensor_distance).str());
	mLogger->message(INFO, (boost::format("maximum_iterations:           %1%") % conf.maximum_iterations).str());
	mLogger->message(INFO, (boost::format("maximum_optimizer_iterations: %1%") % conf.maximum_optimizer_iterations).str());
	mLogger->message(INFO, (boost::format("orientation_sigma:            %1%") % conf.orientation_sigma).str());
	mLogger->message(INFO, (boost::format("point_cloud_density:          %1%") % conf.point_cloud_density).str());
	mLogger->message(INFO, (boost::format("position_sigma:               %1%") % conf.position_sigma).str());
	mLogger->message(INFO, (boost::format("rotation_epsilon:             %1%") % conf.rotation_epsilon).str());
	mLogger->message(INFO, (boost::format("transformation_epsilon:       %1%") % conf.transformation_epsilon).str());
	
	mSolver = new G2oSolver(mLogger);
	mMapper = new BoostMapper(mLogger);

	mLogger->message(INFO, " = GraphMapper - Parameters =");
	mLogger->message(INFO, (boost::format("use_odometry:           %1%") % _use_odometry.get()).str());	
	if(_use_odometry.get())
	{
		mOdometry = new RockOdometry(_robot2odometry, mLogger);
		mMapper->setOdometry(mOdometry, _add_odometry_edges.get());
		mLogger->message(INFO, (boost::format("add_odometry_edges:     %1%") % _add_odometry_edges.get()).str());
	}else
	{
		mOdometry = NULL;
	}
	
	double min_translation = _min_translation.get();
	double min_rotation = _min_rotation.get();
	mLogger->message(INFO, (boost::format("min_pose_distance:      %1% / %2%") % min_translation % min_rotation).str());
	mMapper->setMinPoseDistance(min_translation, min_rotation);
	
	double neighbor_radius = _neighbor_radius.get();
	mLogger->message(INFO, (boost::format("neighbor_radius:        %1%") % neighbor_radius).str());
	int max_neighbor_links = _max_neighbor_links.get();
	mLogger->message(INFO, (boost::format("max_neighbor_links:     %1%") % max_neighbor_links).str());
	mMapper->setNeighborRadius(neighbor_radius, max_neighbor_links);
	
	unsigned range = _patch_building_range.get();
	mLogger->message(INFO, (boost::format("patch_building_range:   %1%") % range).str());
	mMapper->setPatchBuildingRange(range);
	
	mScanResolution = _scan_resolution.get();
	mLogger->message(INFO, (boost::format("scan_resolution:        %1%") % mScanResolution).str());
	
	mMapResolution = _map_resolution.get();
	mLogger->message(INFO, (boost::format("map_resolution:         %1%") % mMapResolution).str());
	
	mOptimizationRate = _optimization_rate.get();
	mLogger->message(INFO, (boost::format("optimization_rate:      %1%") % mOptimizationRate).str());
	
	mMapPublishRate = _map_publish_rate.get();
	mLogger->message(INFO, (boost::format("map_publish_rate:       %1%") % mMapPublishRate).str());
	
	mMapOutlierRadius = _map_outlier_radius.get();
	mLogger->message(INFO, (boost::format("map_outlier_radius:     %1%") % mMapOutlierRadius).str());
	
	mMapOutlierNeighbors = _map_outlier_neighbors.get();
	mLogger->message(INFO, (boost::format("map_outlier_neighbors:  %1%") % mMapOutlierNeighbors).str());
	
	mRobotName = _robot_name.get();
	mLogger->message(INFO, (boost::format("robot_name:             %1%") % mRobotName).str());
	
	mRobotFrame = _robot_frame.get();
	mLogger->message(INFO, (boost::format("robot_frame:            %1%") % mRobotFrame).str());
	
	mOdometryFrame = _odometry_frame.get();
	mLogger->message(INFO, (boost::format("odometry_frame:         %1%") % mOdometryFrame).str());

	mMapFrame = _map_frame.get();
	mLogger->message(INFO, (boost::format("map_frame:              %1%") % mMapFrame).str());

	mUseColorsAsViewpoints = _use_colors_as_viewpoints.get();
	mLogger->message(INFO, (boost::format("use_viewpoints:         %1%") % mUseColorsAsViewpoints).str());

	// MLS
	mGridSizeX = _grid_size_x.get();
	mGridSizeY = _grid_size_y.get();
	mGridOffsetX = _grid_offset_x.get();
	mGridOffsetY = _grid_offset_y.get();
	mGridMinZ = _grid_min_z.get();
	mGridMaxZ = _grid_max_z.get();
	mGridResolution = _grid_resolution.get();

	mMapper->registerSensor(mPclSensor);
	mMapper->setSolver(mSolver);
	
	mScansReceived = 0;
	mScansAdded = 0;
	mRebuildMap = false;
	
	// Initialize MLS-Map
	size_t x_size = mGridSizeX / mGridResolution;
	size_t y_size = mGridSizeY / mGridResolution;
	mMultiLayerMap = new envire::MultiLevelSurfaceGrid(x_size, y_size, mGridResolution, mGridResolution, mGridOffsetX, mGridOffsetY);
	mMultiLayerMap->setUniqueId("/slam3d-mls");
	mMultiLayerMap->getConfig() = _grid_mls_config.get();
	
	// Add MLS to Environment
	envire::FrameNode* mls_node = new envire::FrameNode();
	mEnvironment.addChild(mEnvironment.getRootNode(), mls_node);
	mEnvironment.setFrameNode(mMultiLayerMap, mls_node);
	
	return true;
}

bool PointcloudMapper::startHook()
{
	if(!PointcloudMapperBase::startHook())
		return false;
	return true;
}

PointCloud::Ptr PointcloudMapper::createFromRockMessage(const base::samples::Pointcloud& cloud_in)
{
	PointCloud::Ptr cloud_out(new PointCloud);
	cloud_out->header.stamp = cloud_in.time.toMicroseconds();
	
	if(mUseColorsAsViewpoints)
	{
		if(cloud_in.colors.size() != cloud_in.points.size())
		{
			mLogger->message(WARNING, "Color vector from pointcloud has invalid size!");
			mUseColorsAsViewpoints = false;
		}
	}
	
	unsigned numPoints = cloud_in.points.size();
	for(unsigned i = 0; i < numPoints; ++i)
	{
		PointType p;
		p.x = cloud_in.points[i][0];
		p.y = cloud_in.points[i][1];
		p.z = cloud_in.points[i][2];
		
		if(mUseColorsAsViewpoints)
		{
			p.vp_x = cloud_in.colors[i][0];
			p.vp_y = cloud_in.colors[i][1];
			p.vp_z = cloud_in.colors[i][2];
		}
		
		cloud_out->push_back(p);
	}
	return cloud_out;
}

void PointcloudMapper::createFromPcl(PointCloud::ConstPtr pcl_cloud, base::samples::Pointcloud& base_cloud)
{
	base_cloud.time.fromMicroseconds(pcl_cloud->header.stamp);
	for(PointCloud::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++)
	{
		base::Point p;
		p[0] = it->x;
		p[1] = it->y;
		p[2] = it->z;
		base_cloud.points.push_back(p);
	}
}

void PointcloudMapper::sendRobotPose()
{
	// Publish the robot pose in map
	base::samples::RigidBodyState rbs;
	rbs.setTransform(mCurrentPose);
	rbs.invalidateCovariances();
	rbs.sourceFrame = mRobotFrame;
	rbs.targetFrame = mMapFrame;
	rbs.time = mCurrentTime;
	_map2robot.write(rbs);
	
	// Publish the odometry drift
	if(mOdometry)
	{
		Eigen::Affine3d drift = mCurrentPose * mCurrentOdometry.inverse();
		rbs.setTransform(drift);
		rbs.sourceFrame = mOdometryFrame;
		_map2odometry.write(rbs);
	}
}

void PointcloudMapper::scanTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &scan_sample)
{
	++mScansReceived;
	mCurrentTime = ts;
	if(mOdometry)
	{
		try
		{
			mCurrentOdometry = mOdometry->getOdometricPose(ts);
		}
		catch(OdometryException &e)
		{
			mLogger->message(ERROR, e.what());
			return;
		}
	}

	// Get laser pose
	Transform laserPose = Transform::Identity();
	try
	{
		Eigen::Affine3d affine;
		_laser2robot.get(ts, affine, true);
		if((affine.matrix().array() == affine.matrix().array()).all())
		{
			laserPose.linear() = affine.linear();
			laserPose.translation() = affine.translation();
		}
	}catch(std::exception &e)
	{
		mLogger->message(ERROR, e.what());
	}

	// Transform base::samples::Pointcloud --> Pointcloud
	PointCloud::Ptr cloud = createFromRockMessage(scan_sample);
	
	// Downsample and add to map
	PointCloudMeasurement::Ptr measurement;
	try
	{
		if(mScanResolution > 0)
		{
			PointCloud::Ptr downsampled_cloud = mPclSensor->downsample(cloud, mScanResolution);
			mLogger->message(DEBUG, (boost::format("Downsampled cloud has %1% points.") % downsampled_cloud->size()).str());
			measurement = PointCloudMeasurement::Ptr(new PointCloudMeasurement(downsampled_cloud, mRobotName, mPclSensor->getName(), laserPose));
		}else
		{
			measurement = PointCloudMeasurement::Ptr(new PointCloudMeasurement(cloud, mRobotName, mPclSensor->getName(), laserPose));
		}

		if(mMapper->addReading(measurement))
		{
			mScansAdded++;
			handleNewScan(mMapper->getLastVertex());
			if(mOptimizationRate > 0 && (mScansAdded % mOptimizationRate) == 0)
			{
				optimize();
			}
			if(mMapPublishRate > 0 && (mScansAdded % mMapPublishRate) == 0)
			{
				generate_map();
			}
		}
		mCurrentPose = mMapper->getCurrentPose();
		sendRobotPose();
	}catch(std::exception& e)
	{
		mLogger->message(ERROR, (boost::format("Adding scan to map failed: %1%") % e.what()).str());
	}
}

void PointcloudMapper::updateHook()
{
	PointcloudMapperBase::updateHook();
}

void PointcloudMapper::errorHook()
{
	PointcloudMapperBase::errorHook();
}

void PointcloudMapper::stopHook()
{
	PointcloudMapperBase::stopHook();
}

void PointcloudMapper::cleanupHook()
{
	PointcloudMapperBase::cleanupHook();
	delete mMapper;
	delete mPclSensor;
	delete mSolver;
	delete mLogger;
	delete mClock;
}
