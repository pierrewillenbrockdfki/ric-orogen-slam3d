/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Multiplexer.hpp"

using namespace slam3d;

Multiplexer::Multiplexer(std::string const& name)
    : MultiplexerBase(name)
{
}

Multiplexer::Multiplexer(std::string const& name, RTT::ExecutionEngine* engine)
    : MultiplexerBase(name, engine)
{
}

Multiplexer::~Multiplexer()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Multiplexer.hpp for more detailed
// documentation about them.

bool Multiplexer::configureHook()
{
    if (! MultiplexerBase::configureHook())
        return false;
    return true;
}
bool Multiplexer::startHook()
{
    if (! MultiplexerBase::startHook())
        return false;
    return true;
}
void Multiplexer::updateHook()
{
    MultiplexerBase::updateHook();
}
void Multiplexer::errorHook()
{
    MultiplexerBase::errorHook();
}
void Multiplexer::stopHook()
{
    MultiplexerBase::stopHook();
}
void Multiplexer::cleanupHook()
{
    MultiplexerBase::cleanupHook();
}
