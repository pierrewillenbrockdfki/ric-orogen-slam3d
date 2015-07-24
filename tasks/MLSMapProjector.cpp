/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MLSMapProjector.hpp"

#include <base/Logging.hpp>
#include <envire/core/Serialization.hpp>

using namespace slam3d;

MLSMapProjector::MLSMapProjector(std::string const& name, TaskCore::TaskState initial_state)
    : MLSMapProjectorBase(name, initial_state)
{
}

MLSMapProjector::MLSMapProjector(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : MLSMapProjectorBase(name, engine, initial_state)
{
}

MLSMapProjector::~MLSMapProjector()
{
}

bool MLSMapProjector::configureHook()
{
	if (! MLSMapProjectorBase::configureHook())
		return false;

	// Read parameters
	mSizeX = _size_x.get();
	mSizeY = _size_y.get();
	mOffsetX = _offset_x.get();
	mOffsetY = _offset_y.get();
	mMinZ = _min_z.get();
	mMaxZ = _max_z.get();
	mResolution = _resolution.get();
	
	size_t x_size = mSizeX / mResolution;
	size_t y_size = mSizeY / mResolution;
	mMultiLayerMap = new envire::MultiLevelSurfaceGrid(x_size, y_size, mResolution, mResolution, mOffsetX, mOffsetY);
	mMultiLayerMap->setUniqueId("/slam3d-mls");
	mPointcloud = new envire::Pointcloud();
	mProjection = new envire::MLSProjection();

	// add pointcloud to environment
	envire::FrameNode* cloud_node = new envire::FrameNode();
	mEnvironment.addChild(mEnvironment.getRootNode(), cloud_node);
	mEnvironment.setFrameNode(mPointcloud, cloud_node);
	
	// add mls to environment
	envire::FrameNode* mls_node = new envire::FrameNode();
	mEnvironment.addChild(mEnvironment.getRootNode(), mls_node);
	mEnvironment.setFrameNode(mMultiLayerMap, mls_node);
	
	mProjection->setAreaOfInterest(mOffsetX, mOffsetX+mSizeX, mOffsetY, mOffsetY+mSizeY, mMinZ, mMaxZ);
	if(!mEnvironment.addInput(mProjection, mPointcloud))
	{
		std::cerr << "Failed to add Input!" << std::endl;
		return false;
	}
	if(!mEnvironment.addOutput(mProjection, mMultiLayerMap))
	{
		std::cerr << "Failed to add Output!" << std::endl;
		return false;
	}


	return true;
}

bool MLSMapProjector::startHook()
{
    if (! MLSMapProjectorBase::startHook())
        return false;
    return true;
}

void MLSMapProjector::updateHook()
{
    MLSMapProjectorBase::updateHook();
	
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
	
	// TEST
	EnvireEvents* events = new EnvireEvents;
	mEnvironment.pullEvents(*events, true);
	_envire_map.write(EnvirePointer(events));
	
	// Publish the MLS-Map
/*	envire::BinaryEvent bin_event;
	if(mBinarySerialization.serializeBinaryEvent(mMultiLayerMap, bin_event))
	{
		std::cout << "Created event of type " << bin_event.type << std::endl;
		EnvireEvents* events = new EnvireEvents;
		events->push_back(bin_event);
		_envire_map.write(EnvirePointer(events));
	}else
	{
		std::cerr << "Serialization failed!" << std::endl;
	}*/
}

void MLSMapProjector::errorHook()
{
    MLSMapProjectorBase::errorHook();
}

void MLSMapProjector::stopHook()
{
    MLSMapProjectorBase::stopHook();
}

void MLSMapProjector::cleanupHook()
{
    MLSMapProjectorBase::cleanupHook();
}
