/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ScanConverter.hpp"

#include <velodyne_lidar/MultilevelLaserScan.h>
#include <velodyne_lidar/pointcloudConvertHelper.hpp>

#include <base/Logging.hpp>
#include <base/samples/Pointcloud.hpp>

using namespace slam3d;

ScanConverter::ScanConverter(std::string const& name, TaskCore::TaskState initial_state)
    : ScanConverterBase(name, initial_state)
{
}

ScanConverter::ScanConverter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : ScanConverterBase(name, engine, initial_state)
{
}

ScanConverter::~ScanConverter()
{
}

bool ScanConverter::configureHook()
{
	if (! ScanConverterBase::configureHook())
		return false;
	return true;
}

bool ScanConverter::startHook()
{
	if (! ScanConverterBase::startHook())
		return false;
	return true;
}

void ScanConverter::updateHook()
{
	ScanConverterBase::updateHook();
	
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
		}
		cloud.time = scan.time;
		_cloud.write(cloud);
	}
}

void ScanConverter::errorHook()
{
	ScanConverterBase::errorHook();
}

void ScanConverter::stopHook()
{
	ScanConverterBase::stopHook();
}

void ScanConverter::cleanupHook()
{
	ScanConverterBase::cleanupHook();
}
