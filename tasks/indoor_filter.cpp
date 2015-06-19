#include "indoor_filter.hpp"

#include <base/Logging.hpp>
#include <base/samples/Pointcloud.hpp>

using namespace slam3d;

indoor_filter::indoor_filter(std::string const& name, TaskCore::TaskState initial_state)
 : indoor_filterBase(name, initial_state)
{
}

indoor_filter::indoor_filter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
 : indoor_filterBase(name, engine, initial_state)
{
}

indoor_filter::~indoor_filter()
{
}

bool indoor_filter::configureHook()
{
	if (! indoor_filterBase::configureHook())
		return false;
	return true;
}

bool indoor_filter::startHook()
{
	if (! indoor_filterBase::startHook())
		return false;
	return true;
}

void indoor_filter::updateHook()
{
	indoor_filterBase::updateHook();
	
	// Read the scan from the port
	base::samples::Pointcloud cloud;
	while(_cloud_in.read(cloud, false) == RTT::NewData)
	{
		base::samples::Pointcloud filtered_cloud;
		for(std::vector<base::Vector3d>::const_iterator it = cloud.points.begin(); it < cloud.points.end(); ++it)
		{
			double sq_dist = ((*it)[0] * (*it)[0]) + ((*it)[1] * (*it)[1]) + ((*it)[2] * (*it)[2]);
			if((*it)[2] < 0.5 && sq_dist < 100)
				filtered_cloud.points.push_back(*it);
		}
		filtered_cloud.time = cloud.time;
		_cloud_out.write(filtered_cloud);
	}
}

void indoor_filter::errorHook()
{
	indoor_filterBase::errorHook();
}

void indoor_filter::stopHook()
{
	indoor_filterBase::stopHook();
}

void indoor_filter::cleanupHook()
{
	indoor_filterBase::cleanupHook();
}
