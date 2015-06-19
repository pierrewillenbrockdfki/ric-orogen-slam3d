/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "dump_pcl.hpp"

#include <base/Logging.hpp>
#include <velodyne_lidar/MultilevelLaserScan.h>
#include <velodyne_lidar/pointcloudConvertHelper.hpp>

#include <stdio.h>

using namespace slam3d;

dump_pcl::dump_pcl(std::string const& name, TaskCore::TaskState initial_state)
    : dump_pclBase(name, initial_state)
{
}

dump_pcl::dump_pcl(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : dump_pclBase(name, engine, initial_state)
{
}

dump_pcl::~dump_pcl()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See dump_pcl.hpp for more detailed
// documentation about them.

bool dump_pcl::configureHook()
{
    if (! dump_pclBase::configureHook())
        return false;
    return true;
}
bool dump_pcl::startHook()
{
    if (! dump_pclBase::startHook())
        return false;
		
    mScanNumber = 0;
    return true;
}

void dump_pcl::handlePointcloud(const std::vector<Eigen::Vector3d>& points, std::vector<float> remission)
{
	int num = points.size();
	int pointer = 0;
	float* data = new float[num*4];
	
	for(int point = 0; point < num; ++point)
	{
		data[pointer] = (float)(points.at(point)[0]); pointer++;
		data[pointer] = (float)(points.at(point)[1]); pointer++;
		data[pointer] = (float)(points.at(point)[2]); pointer++;
		data[pointer] = remission.at(point);          pointer++;
	}
	
	try
	{
		char filename[20];
		sprintf(filename, "%06d.bin", mScanNumber);
		FILE* stream = fopen(filename, "wb");
		fwrite(data, sizeof(float), 4*num, stream);
		fclose(stream);
		mScanNumber++;
	}catch(std::exception &e)
	{
		LOG_ERROR("Error writing file: %s", e.what());
	}
	delete data;
}

void dump_pcl::updateHook()
{
    dump_pclBase::updateHook();
	
	// Read the scan from the port
	velodyne_lidar::MultilevelLaserScan scan;
	while(_scan.read(scan, false) == RTT::NewData)
	{
		// Convert to PointCloud
		std::vector<Eigen::Vector3d> points;
		std::vector<float> remission;
		velodyne_lidar::ConvertHelper::convertScanToPointCloud(scan, points, Eigen::Affine3d::Identity(), true, 0, &remission);
		if(points.size() == 0)
		{
			LOG_ERROR("Convertion to pointcloud returned no points!");
			continue;
		}else
		{
			LOG_INFO("Converted to pointcloud with %d points.", points.size());
		}
		
		if(points.size() != remission.size())
		{
			LOG_ERROR("Got %d points but %d remissions!", points.size(), remission.size());
			continue;
		}
		
		handlePointcloud(points, remission);
	}
	
	base::samples::Pointcloud cloud;
	while(_pcl.read(cloud, false) == RTT::NewData)
	{
		std::vector<Eigen::Vector3d> points;
		std::vector<float> remission;
		for(std::vector<base::Vector3d>::const_iterator it = cloud.points.begin(); it < cloud.points.end(); ++it)
		{
			Eigen::Vector3d p;
			p[0] = (*it)[0];
			p[1] = (*it)[1];
			p[2] = (*it)[2];
			points.push_back(p);
			remission.push_back(1.0);
		}
		handlePointcloud(points, remission);
	}
}
void dump_pcl::errorHook()
{
    dump_pclBase::errorHook();
}
void dump_pcl::stopHook()
{
    dump_pclBase::stopHook();
}
void dump_pcl::cleanupHook()
{
    dump_pclBase::cleanupHook();
}
