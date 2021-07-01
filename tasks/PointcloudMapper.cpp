#include "PointcloudMapper.hpp"
#include "RockOdometry.hpp"
#include "BaseLogger.hpp"
#include "Common.hpp"

#include <base/samples/Pointcloud.hpp>
#include <base/samples/RigidBodyState.hpp>

#include <slam3d/graph/boost/BoostGraph.hpp>
#include <slam3d/core/FileLogger.hpp>
#include <slam3d/solver/g2o/G2oSolver.hpp>

#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

using namespace slam3d;

PointcloudMapper::PointcloudMapper(std::string const& name)
    : PointcloudMapperBase(name)
{
}

PointcloudMapper::PointcloudMapper(std::string const& name, RTT::ExecutionEngine* engine)
    : PointcloudMapperBase(name, engine)
{
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
		if(mGraph->optimize())
		{
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
	VertexObjectList vertices = mGraph->getVerticesFromSensor(mPclSensor->getName());
	boost::thread projThread(&PointcloudMapper::sendPointcloud, this, vertices);
	return true;
}

bool PointcloudMapper::generate_map()
{
	mLogger->message(INFO, "Requested map generation.");
	if(mGraph->optimized())
	{
		VertexObjectList vertices = mGraph->getVerticesFromSensor(mPclSensor->getName());
		boost::thread projThread(&PointcloudMapper::rebuildMap, this, vertices);
	}else
	{
		sendMap();
	}
	return true;
}

bool PointcloudMapper::force_add()
{
	mForceAdd = true;
	return true;
}

bool PointcloudMapper::write_envire()
{
	boost::filesystem::path env_path(_envire_path.value());
	boost::filesystem::create_directories(env_path);
	env_path += "mls_map-";
	env_path += base::Time::now().toString(base::Time::Seconds, "%Y%m%d-%H%M");
	env_path += ".bin";

	std::ofstream ostream;
	ostream.open(env_path.string());
	if(!ostream.is_open())
	{
		mLogger->message(ERROR, (boost::format("Failed to serialize MLS map! Couldn't open file %1%") % env_path.string()).str());
		return false;
	}

	boost::archive::binary_oarchive bin_out(ostream);
	bin_out << mMultiLayerMap.getData();
	ostream.close();
	return true;
}

bool PointcloudMapper::write_graph()
{
	mGraph->writeGraphToFile("slam3d_graph");
	return true;
}

bool PointcloudMapper::write_ply(const std::string& folder)
{
	mLogger->message(INFO, "Write pointcloud to PLY file.");
	VertexObjectList vertices = mGraph->getVerticesFromSensor(mPclSensor->getName());
	PointCloud::Ptr accCloud;
	try
	{
		accCloud = buildPointcloud(vertices);
	}
	catch (boost::lock_error &e)
	{
		mLogger->message(ERROR, "Could not access the pose graph to build Pointcloud!");
		return false;
	}

	boost::filesystem::path ply_path(folder);
	boost::filesystem::create_directories(ply_path);
	ply_path += "pointcloud-";
	ply_path += base::Time::now().toString(base::Time::Seconds, "%Y%m%d-%H%M");
	ply_path += ".ply";

	pcl::PLYWriter ply_writer;
	return ply_writer.write(ply_path.string(), *accCloud) >= 0;
}

PointCloud::Ptr PointcloudMapper::buildPointcloud(const VertexObjectList& vertices)
{
	timeval start = mClock->now();
	boost::shared_lock<boost::shared_mutex> guard(mGraphMutex);
	PointCloud::Ptr cloud = mPclSensor->buildMap(vertices);
	timeval finish = mClock->now();
	int duration = finish.tv_sec - start.tv_sec;
	mLogger->message(INFO, (boost::format("Generated Pointcloud from %1% scans in %2% seconds.") % vertices.size() % duration).str());
	return cloud;
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
	createFromPcl(accCloud, mapCloud);
	_cloud.write(mapCloud);
}

PointCloudMeasurement::Ptr PointcloudMapper::castToPointcloud(Measurement::Ptr m)
{
	PointCloudMeasurement::Ptr pcm = boost::dynamic_pointer_cast<PointCloudMeasurement>(m);
	if(!pcm)
	{
		throw BadMeasurementType();
	}
	return pcm;
}

void PointcloudMapper::handleNewScan(const VertexObject& scan)
{
	addScanToMap(castToPointcloud(scan.measurement), scan.corrected_pose);
}

void PointcloudMapper::addScanToMap(PointCloudMeasurement::Ptr scan, const Transform& pose)
{	
	PointCloud::ConstPtr pcl = scan->getPointCloud();
	boost::unique_lock<boost::shared_mutex> guard(mMapMutex);
	mMultiLayerMap.getData().mergePointCloud(*pcl, pose * scan->getSensorPose());
}

void PointcloudMapper::clearMap()
{
	boost::unique_lock<boost::shared_mutex> guard(mMapMutex);
	mMultiLayerMap.getData().clear();
}

void PointcloudMapper::rebuildMap(const VertexObjectList& vertices)
{
	timeval start = mClock->now();
	clearMap();

	boost::shared_lock<boost::shared_mutex> guard(mGraphMutex);
	for(VertexObjectList::const_iterator v = vertices.begin(); v != vertices.end(); ++v)
	{
		addScanToMap(castToPointcloud(v->measurement), v->corrected_pose);
	}
	timeval finish = mClock->now();
	int duration = finish.tv_sec - start.tv_sec;
	mLogger->message(INFO, (boost::format("Completely rebuild map from %1% scans in %2% seconds.") % vertices.size() % duration).str());

	sendMap();
}

void PointcloudMapper::sendMap()
{
	// Publish the MLS-Map
	mMultiLayerMap.setTime(mCurrentTime);
	_mls.write(mMultiLayerMap.asSpatioTemporal());
	mScansReceived = 0;
}

bool PointcloudMapper::loadPLYMap(const std::string& path)
{
	PointCloud::Ptr pcl_cloud(new PointCloud());
	pcl::PLYReader ply_reader;
	if(ply_reader.read(path, *pcl_cloud) >= 0)
	{
		Transform pc_tr(pcl_cloud->sensor_orientation_.cast<ScalarType>());
		pc_tr.translation() = pcl_cloud->sensor_origin_.block(0,0,3,1).cast<ScalarType>();
		PointCloudMeasurement::Ptr initial_map(new PointCloudMeasurement(pcl_cloud, _robot_name, mPclSensor->getName(), pc_tr));
		try
		{
			VertexObject root_node = mGraph->getVertex(0);
			mMapper->addExternalMeasurement(initial_map, root_node.measurement->getUniqueId(), TransformWithCovariance(), "ply-loader");
			addScanToMap(initial_map, Transform::Identity());
			return true;
		}
		catch(std::exception& e)
		{
			mLogger->message(ERROR, (boost::format("Adding initial point cloud failed: %1%") % e.what()).str());
		}
	}else
	{
		mLogger->message(ERROR, (boost::format("Failed to load a-priori PLY map %1%") % path).str());
	}
	return false;
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

	// Create all internal objects
	mSolver = new G2oSolver(mLogger);
	mPatchSolver = new G2oSolver(mLogger);

	mPclSensor = new PointCloudSensor("LaserScanner", mLogger);
	mPclSensor->setPatchSolver(mPatchSolver);

	mGraph = new BoostGraph(mLogger);
	mGraph->setSolver(mSolver, _optimization_rate);
	mGraph->fixNext();

	mMapper = new Mapper(mGraph, mLogger);
	mMapper->registerSensor(mPclSensor);

	// Read and set parameters
	mLogger->message(INFO, "=== PointCloudSensor - Parameters ===");
	mPclSensor->setMinPoseDistance(_min_translation, _min_rotation);
	mPclSensor->setNeighborRadius(_neighbor_radius, _max_neighbor_links);
	mPclSensor->setPatchBuildingRange(_patch_building_range);
	mPclSensor->setMapResolution(_map_resolution);
	mPclSensor->setMapOutlierRemoval(_map_outlier_radius, _map_outlier_neighbors);
	mPclSensor->setLinkPrevious(_sequential_icp);	
	mPclSensor->setMinLoopLength(_min_loop_length);
	mPclSensor->setRegistrationParameters(_gicp_config, false);
	mPclSensor->setRegistrationParameters(_gicp_coarse_config, true);

	mLogger->message(INFO, " === Mapper - Parameters ===");
	mLogger->message(INFO, (boost::format("use_odometry:           %1%") % _use_odometry.get()).str());	
	if(_use_odometry.get())
	{
		mOdometry = new RockOdometry("RockOdometry", mGraph, mSolver, mLogger, _robot2odometry);
		_robot2odometry.registerUpdateCallback(boost::bind(&PointcloudMapper::transformerCallback, this, _1));
		mLogger->message(INFO, (boost::format("add_odometry_edges:     %1%") % _add_odometry_edges).str());
		if(_add_odometry_edges)
		{
			mMapper->registerPoseSensor(mOdometry);
		}
		base::Vector3d vec3d = _gravity_reference.get();
		if(vec3d.norm() > 0)
		{
			mOdometry->setGravityReference(vec3d);
		}
	}else
	{
		mOdometry = NULL;
	}
	mCurrentOdometry = Eigen::Affine3d::Identity();
	
	mLogger->message(INFO, (boost::format("scan_resolution:        %1%") % _scan_resolution).str());	
	mLogger->message(INFO, (boost::format("map_publish_rate:       %1%") % _map_publish_rate).str());
	mLogger->message(INFO, (boost::format("optimization_rate:      %1%") % _optimization_rate).str());
	
	mLogger->message(INFO, (boost::format("robot_name:             %1%") % _robot_name).str());
	mLogger->message(INFO, (boost::format("laser_frame:            %1%") % _laser_frame.get()).str());
	mLogger->message(INFO, (boost::format("robot_frame:            %1%") % _robot_frame).str());
	mLogger->message(INFO, (boost::format("odometry_frame:         %1%") % _odometry_frame).str());
	mLogger->message(INFO, (boost::format("map_frame:              %1%") % _map_frame).str());
	
	mScansAdded = 0;
	mScansReceived = 0;
	mForceAdd = false;
	
	// Initialize MLS-Map
	mGridConf = _grid_config.get();
	mLogger->message(INFO, " = Grid - Parameters =");
	mLogger->message(INFO, (boost::format("min_x: %1%") % mGridConf.min_x).str());
	mLogger->message(INFO, (boost::format("max_x: %1%") % mGridConf.max_x).str());
	mLogger->message(INFO, (boost::format("min_y: %1%") % mGridConf.min_y).str());
	mLogger->message(INFO, (boost::format("max_y: %1%") % mGridConf.max_y).str());
	mLogger->message(INFO, (boost::format("min_z: %1%") % mGridConf.min_z).str());
	mLogger->message(INFO, (boost::format("max_z: %1%") % mGridConf.max_z).str());
	mLogger->message(INFO, (boost::format("resolution: %1%") % mGridConf.resolution).str());
	
	size_t x_size = (mGridConf.max_x - mGridConf.min_x) / mGridConf.resolution;
	size_t y_size = (mGridConf.max_y - mGridConf.min_y) / mGridConf.resolution;
	
	mMultiLayerMap.setData(maps::grid::MLSMapSloped(maps::grid::Vector2ui(x_size, y_size), Eigen::Vector2d(mGridConf.resolution, mGridConf.resolution), _grid_mls_config.value()));
	mMultiLayerMap.getData().getId() = "/slam3d-mls";
	mMultiLayerMap.getData().translate(Eigen::Vector3d(mGridConf.min_x, mGridConf.min_y, 0));
	mMultiLayerMap.setFrame(_map_frame.value());

	// load a-priori map file
	if(!_apriori_ply_map.value().empty() && loadPLYMap(_apriori_ply_map.value()))
	{
		mScansAdded++;
	}
	
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
	cloud_out->reserve(cloud_in.points.size());
	
	unsigned numPoints = cloud_in.points.size();
	for(unsigned i = 0; i < numPoints; ++i)
	{
		PointType p;
		p.x = cloud_in.points[i][0];
		p.y = cloud_in.points[i][1];
		p.z = cloud_in.points[i][2];
		cloud_out->push_back(p);
	}
    
	if(mScansAdded == 0 && _initial_patch_radius > 0)
	{
		mPclSensor->fillGroundPlane(cloud_out, _initial_patch_radius);
	}
	return cloud_out;
}

void PointcloudMapper::createFromPcl(PointCloud::ConstPtr pcl_cloud, base::samples::Pointcloud& base_cloud)
{
	base_cloud.time.fromMicroseconds(pcl_cloud->header.stamp);
	base_cloud.points.reserve(pcl_cloud->size());
	for(PointCloud::const_iterator it = pcl_cloud->begin(); it < pcl_cloud->end(); it++)
	{
		base::Point p;
		p[0] = it->x;
		p[1] = it->y;
		p[2] = it->z;
		base_cloud.points.push_back(p);
	}
}

void PointcloudMapper::transformerCallback(const base::Time &time)
{
	try
	{
		mCurrentOdometry = mOdometry->getPose(time);

		// Send the current pose
		base::samples::RigidBodyState rbs;
		rbs.invalidateCovariances();
		rbs.time = time;
		rbs.sourceFrame = _robot_frame;
		rbs.targetFrame = _map_frame;
		rbs.setTransform(mCurrentDrift * mCurrentOdometry);
		_robot2map.write(rbs);
	}
	catch(InvalidPose &e)
	{
		mLogger->message(ERROR, e.what());
	}
}

void PointcloudMapper::updateHook()
{
	try
	{
		PointcloudMapperBase::updateHook();
	}catch(std::exception &e)
	{
		mLogger->message(ERROR, e.what());
		return;
	}

	base::samples::Pointcloud scan_sample;
	while(_scan.read(scan_sample, false) == RTT::NewData)
	{
		// Get laser pose
		Transform laserPose = Transform::Identity();
		try
		{
			Eigen::Affine3d affine;
			if(!_laser2robot.get(scan_sample.time, affine, false) || !affine.matrix().allFinite())
			{
				mLogger->message(ERROR, (boost::format("Failed to receive a valid transform from '%1%' to '%2%'!")
					% _laser_frame % _robot_frame).str());
				continue;
			}
			laserPose.linear() = affine.linear();
			laserPose.translation() = affine.translation();
		}
		catch(std::exception &e)
		{
			mLogger->message(ERROR, e.what());
			continue;
		}

		// Transform base::samples::Pointcloud --> Pointcloud
		PointCloud::Ptr cloud = createFromRockMessage(scan_sample);
		
		// Downsample and add to map
		PointCloudMeasurement::Ptr measurement;
		try
		{
			if(_scan_resolution > 0)
			{
				PointCloud::Ptr downsampled_cloud = mPclSensor->downsample(cloud, _scan_resolution);
				mLogger->message(DEBUG, (boost::format("Downsampled cloud has %1% points.") % downsampled_cloud->size()).str());
				measurement = PointCloudMeasurement::Ptr(new PointCloudMeasurement(downsampled_cloud, _robot_name, mPclSensor->getName(), laserPose));
			}else
			{
				measurement = PointCloudMeasurement::Ptr(new PointCloudMeasurement(cloud, _robot_name, mPclSensor->getName(), laserPose));
			}
			
			mScansReceived++;

			if(mPclSensor->addMeasurement(measurement, mOdometry->getPose(measurement->getTimestamp())))
			{
				mScansAdded++;
				mForceAdd = false;
				mCurrentDrift = mMapper->getCurrentPose() * mOdometry->getPose().inverse();
				mPclSensor->linkLastToNeighbors();
				handleNewScan(mMapper->getLastVertex());
				if(_map_publish_rate > 0 && (mScansAdded % _map_publish_rate) == 0)
				{
					generate_map();
				}
			}else
			{
				if((_map_update_rate > 0) && (mScansReceived >= _map_update_rate))
				{
					addScanToMap(measurement, mMapper->getCurrentPose());
					sendMap();
				}
			}
			mCurrentTime = scan_sample.time;
			
			// Send the calculated transform
			base::samples::RigidBodyState rbs;
			rbs.invalidateCovariances();
			rbs.targetFrame = _map_frame;
			rbs.time = mCurrentTime;
			
			if(mOdometry)
			{
				rbs.sourceFrame = _odometry_frame;
				rbs.setTransform(mCurrentDrift);
				_odometry2map.write(rbs);
			}else
			{
				rbs.sourceFrame = _robot_frame;
				rbs.setTransform(mMapper->getCurrentPose());
				_robot2map.write(rbs);
			}
		}catch(std::exception& e)
		{
			mLogger->message(ERROR, (boost::format("Adding scan to map failed: %1%") % e.what()).str());
		}
	}
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
	delete mGraph;
	delete mPclSensor;
	if(mOdometry)
		delete mOdometry;
	delete mSolver;
	delete mPatchSolver;
	delete mLogger;
	delete mClock;
}
