#include "PointcloudMapper2D.hpp"

#include <envire/Core.hpp>

using namespace slam3d;

PointcloudMapper2D::PointcloudMapper2D(std::string const& name)
    : PointcloudMapper2DBase(name)
{
}

PointcloudMapper2D::PointcloudMapper2D(std::string const& name, RTT::ExecutionEngine* engine)
    : PointcloudMapper2DBase(name, engine)
{
}

PointcloudMapper2D::~PointcloudMapper2D()
{
}

bool PointcloudMapper2D::generate_map()
{
	// Generate the Grid-Map
	
	// Publish the Grid-Map
	EnvireEvents* events = new EnvireEvents;
	mEnvironment.pullEvents(*events, true);
	_envire_map.write(EnvirePointer(events));
	return true;
}

bool PointcloudMapper2D::configureHook()
{
	if (! PointcloudMapper2DBase::configureHook())
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
	size_t x_size = mSizeX / mResolution;
	size_t y_size = mSizeY / mResolution;
	mGrid = new envire::TraversabilityGrid(x_size, y_size, mResolution, mResolution, mOffsetX, mOffsetY, "slam3d-grid");

	// Add grid to environment
	envire::FrameNode* grid_node = new envire::FrameNode();
	mEnvironment.addChild(mEnvironment.getRootNode(), grid_node);
	mEnvironment.setFrameNode(mGrid, grid_node);
	return true;
}

bool PointcloudMapper2D::startHook()
{
	if (! PointcloudMapper2DBase::startHook())
		return false;
	return true;
}

void PointcloudMapper2D::updateHook()
{
	PointcloudMapper2DBase::updateHook();
}

void PointcloudMapper2D::errorHook()
{
	PointcloudMapper2DBase::errorHook();
}

void PointcloudMapper2D::stopHook()
{
	PointcloudMapper2DBase::stopHook();
}

void PointcloudMapper2D::cleanupHook()
{
	PointcloudMapper2DBase::cleanupHook();
}
