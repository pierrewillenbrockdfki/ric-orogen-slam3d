#include "TraversabilityMapProjector.hpp"

using namespace slam3d;

TraversabilityMapProjector::TraversabilityMapProjector(std::string const& name, TaskCore::TaskState initial_state)
    : TraversabilityMapProjectorBase(name, initial_state)
{
}

TraversabilityMapProjector::TraversabilityMapProjector(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TraversabilityMapProjectorBase(name, engine, initial_state)
{
}

TraversabilityMapProjector::~TraversabilityMapProjector()
{
}

bool TraversabilityMapProjector::configureHook()
{
	if (! TraversabilityMapProjectorBase::configureHook())
		return false;

	// Read parameters
	mSizeX = _size_x.get();
	mSizeY = _size_y.get();
	mOffsetX = _offset_x.get();
	mOffsetY = _offset_y.get();
	mMinZ = _min_z.get();
	mMaxZ = _max_z.get();
	mResolution = _resolution.get();

	mGrid = new envire::TraversabilityGrid(mSizeX, mSizeY, mResolution, mResolution, mOffsetX, mOffsetY, "slam3d-grid");

	// Add grid to environment
	envire::FrameNode* grid_node = new envire::FrameNode();
	mEnvironment.addChild(mEnvironment.getRootNode(), grid_node);
	mEnvironment.setFrameNode(mGrid, grid_node);
	return true;
}

bool TraversabilityMapProjector::startHook()
{
	if (! TraversabilityMapProjectorBase::startHook())
		return false;
	return true;
}

void TraversabilityMapProjector::updateHook()
{
	TraversabilityMapProjectorBase::updateHook();

	// Read point cloud from the port
	base::samples::Pointcloud cloud;
	while(_cloud.read(cloud, false) == RTT::NewData)
	{
		// Project the pointcloud to the grid
	}

	// Publish the Grid-Map
	EnvireEvents* events = new EnvireEvents;
	mEnvironment.pullEvents(*events, true);
	_envire_map.write(EnvirePointer(events));
}

void TraversabilityMapProjector::errorHook()
{
	TraversabilityMapProjectorBase::errorHook();
}

void TraversabilityMapProjector::stopHook()
{
	TraversabilityMapProjectorBase::stopHook();
}

void TraversabilityMapProjector::cleanupHook()
{
	TraversabilityMapProjectorBase::cleanupHook();
}

