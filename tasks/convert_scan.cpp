/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "convert_scan.hpp"

#include <velodyne_lidar/MultilevelLaserScan.h>
#include <velodyne_lidar/pointcloudConvertHelper.hpp>

#include <base/Logging.hpp>
#include <base/samples/Pointcloud.hpp>

using namespace slam3d;

convert_scan::convert_scan(std::string const& name, TaskCore::TaskState initial_state)
    : convert_scanBase(name, initial_state)
{
}

convert_scan::convert_scan(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : convert_scanBase(name, engine, initial_state)
{
}

convert_scan::~convert_scan()
{
}

bool convert_scan::configureHook()
{
	if (! convert_scanBase::configureHook())
		return false;
	return true;
}

bool convert_scan::startHook()
{
	if (! convert_scanBase::startHook())
		return false;
	return true;
}

void convert_scan::updateHook()
{
	convert_scanBase::updateHook();
	
	// Get scans from input port
	velodyne_lidar::MultilevelLaserScan scan;
	while(_scan.read(scan, false) == RTT::NewData)
	{
		// Convert to PointCloud
		std::vector<Eigen::Vector3d> points;
		velodyne_lidar::ConvertHelper::convertScanToPointCloud(scan, points);
		if(points.size() == 0)
		{
			LOG_ERROR("Convertion to pointcloud returned no points!");
			return;
		}else
		{
			LOG_DEBUG("Converted to pointcloud with %d points.", points.size());
		}
		
		base::samples::Pointcloud cloud;
		for(std::vector<Eigen::Vector3d>::iterator it = points.begin(); it < points.end(); ++it)
		{
			cloud.points.push_back(*it);
	//		base::Vector3d vec;
	//		vec[0] = it->x;
	//		vec[1] = it->y;
	//		vec[2] = it->z;
	//		mapCloud.points.push_back(vec);
		}
	//	cloud.time = base::Time::fromMicroseconds(accCloud->header.stamp);
		cloud.time = scan.time;
		_cloud.write(cloud);
	}
/*	
	slam::PointCloud::Ptr cloud(new slam::PointCloud);
	cloud->header.stamp = scan.time.toMicroseconds();
	for(std::vector<Eigen::Vector3d>::iterator it = points.begin(); it < points.end(); ++it)
	{
		slam::PointType p;
		p.x = (*it)[0];
		p.y = (*it)[1];
		p.z = (*it)[2];
		cloud->push_back(p);
	}
*/
}

void convert_scan::errorHook()
{
	convert_scanBase::errorHook();
}

void convert_scan::stopHook()
{
	convert_scanBase::stopHook();
}

void convert_scan::cleanupHook()
{
	convert_scanBase::cleanupHook();
}
