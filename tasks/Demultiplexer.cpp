/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Demultiplexer.hpp"

using namespace slam3d;

Demultiplexer::Demultiplexer(std::string const& name)
    : DemultiplexerBase(name)
{
}

Demultiplexer::Demultiplexer(std::string const& name, RTT::ExecutionEngine* engine)
    : DemultiplexerBase(name, engine)
{
}

Demultiplexer::~Demultiplexer()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Demultiplexer.hpp for more detailed
// documentation about them.

bool Demultiplexer::configureHook()
{
    if (! DemultiplexerBase::configureHook())
        return false;
    return true;
}
bool Demultiplexer::startHook()
{
    if (! DemultiplexerBase::startHook())
        return false;
    return true;
}
void Demultiplexer::updateHook()
{
    DemultiplexerBase::updateHook();
}
void Demultiplexer::errorHook()
{
    DemultiplexerBase::errorHook();
}
void Demultiplexer::stopHook()
{
    DemultiplexerBase::stopHook();
}
void Demultiplexer::cleanupHook()
{
    DemultiplexerBase::cleanupHook();
}
