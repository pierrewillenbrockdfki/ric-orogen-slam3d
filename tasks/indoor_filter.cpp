#include "indoor_filter.hpp"

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
