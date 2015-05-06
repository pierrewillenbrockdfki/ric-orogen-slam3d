/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Mapper.hpp"

using namespace slam3d;

Mapper::Mapper(std::string const& name)
    : MapperBase(name)
{
}

Mapper::Mapper(std::string const& name, RTT::ExecutionEngine* engine)
    : MapperBase(name, engine)
{
}

Mapper::~Mapper()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Mapper.hpp for more detailed
// documentation about them.

bool Mapper::configureHook()
{
    if (! MapperBase::configureHook())
        return false;
    return true;
}
bool Mapper::startHook()
{
    if (! MapperBase::startHook())
        return false;
    return true;
}
void Mapper::updateHook()
{
    MapperBase::updateHook();
}
void Mapper::errorHook()
{
    MapperBase::errorHook();
}
void Mapper::stopHook()
{
    MapperBase::stopHook();
}
void Mapper::cleanupHook()
{
    MapperBase::cleanupHook();
}
