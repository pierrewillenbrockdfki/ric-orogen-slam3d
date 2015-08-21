/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DistributedPointcloudMapper.hpp"

using namespace slam3d;

DistributedPointcloudMapper::DistributedPointcloudMapper(std::string const& name)
    : DistributedPointcloudMapperBase(name)
{
}

DistributedPointcloudMapper::DistributedPointcloudMapper(std::string const& name, RTT::ExecutionEngine* engine)
    : DistributedPointcloudMapperBase(name, engine)
{
}

DistributedPointcloudMapper::~DistributedPointcloudMapper()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DistributedPointcloudMapper.hpp for more detailed
// documentation about them.

bool DistributedPointcloudMapper::configureHook()
{
    if (! DistributedPointcloudMapperBase::configureHook())
        return false;
    return true;
}
bool DistributedPointcloudMapper::startHook()
{
    if (! DistributedPointcloudMapperBase::startHook())
        return false;
    return true;
}
void DistributedPointcloudMapper::updateHook()
{
    DistributedPointcloudMapperBase::updateHook();
}
void DistributedPointcloudMapper::errorHook()
{
    DistributedPointcloudMapperBase::errorHook();
}
void DistributedPointcloudMapper::stopHook()
{
    DistributedPointcloudMapperBase::stopHook();
}
void DistributedPointcloudMapper::cleanupHook()
{
    DistributedPointcloudMapperBase::cleanupHook();
}
