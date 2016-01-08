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

	// Initialize envire stuff
	mPointcloud = new envire::Pointcloud();
	size_t x_size = mSizeX / mResolution;
	size_t y_size = mSizeY / mResolution;
	mGrid = new envire::TraversabilityGrid(x_size, y_size, mResolution, mResolution, mOffsetX, mOffsetY, "slam3d-grid");
	mProjection = new envire::TraversabilityProjection();

	// Add grid to environment
	envire::FrameNode* grid_node = new envire::FrameNode();
	mEnvironment.addChild(mEnvironment.getRootNode(), grid_node);
	mEnvironment.setFrameNode(mGrid, grid_node);
	
	// Add projection operator
	if(!mEnvironment.addInput(mProjection, mPointcloud))
	{
		LOG_ERROR("Failed to add Input to Envire!");
		return false;
	}
	if(!mEnvironment.addOutput(mProjection, mGrid))
	{
		LOG_ERROR("Failed to add Output to Envire!");
		return false;
	}
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
		mPointcloud->vertices.clear();
		for(std::vector<base::Vector3d>::const_iterator it = cloud.points.begin(); it < cloud.points.end(); ++it)
		{
			mPointcloud->vertices.push_back(*it);
		}
		mProjection->updateAll();
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

