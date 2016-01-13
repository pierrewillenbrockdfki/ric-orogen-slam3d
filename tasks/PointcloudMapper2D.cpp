#include "PointcloudMapper2D.hpp"

#include <envire/Core.hpp>
#include <envire/Orocos.hpp>

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
	// Setup a temporary array
	size_t size_x = mGrid->getCellSizeX();
	size_t size_y = mGrid->getCellSizeY();
	size_t arr_size = size_x * size_y;
	int occ[arr_size];
	int hit[arr_size];
	memset(occ, 0, arr_size);
	memset(hit, 0, arr_size);	
	
	// Project pointclouds to temporary array
	mLogger->message(INFO, "Requested traversability-grid generation.");
	VertexList vertices = mMapper->getVerticesFromSensor(mPclSensor->getName());
	int count = 0;
	int valid = 0;
	for(VertexList::const_iterator it = vertices.begin(); it < vertices.end(); it++)
	{
		PointCloudMeasurement* pcl = dynamic_cast<PointCloudMeasurement*>((*it)->measurement);
		if(!pcl)
		{
			mLogger->message(ERROR, "Measurement from PCL-Sensor is not a point cloud!");
			return false;
		}
		PointCloud::ConstPtr cloud = pcl->getPointCloud();
		for(PointCloud::const_iterator it = cloud->begin(); it < cloud->end(); it++)
		{
			size_t grid_x, grid_y;
			if(mGrid->toGrid(Eigen::Vector3d(it->x, it->y, 0), grid_x, grid_y))
			{
				occ[(grid_y * size_x) + grid_x] += 1;
				hit[(grid_y * size_x) + grid_x] += 1;
				valid++;
			}
			count++;
		}
	}
	std::cout << "Projected " << valid << " out of " << count << " points to the grid." << std::endl;
	
	// Write temporary array to traversability map
	size_t index = 0;
	for(size_t y = 0; y < size_y; y++)
	{
		for(size_t x = 0; x < size_x; x++, index++)
		{
			int k = 0;
			double p = 1;
			if(hit[index] > 0)
			{
				if((occ[index] / hit[index]) > 0.5)
				{
					k = 1;
				}
//				p = 2.0 * std::abs(0.5 - (occ[index] / hit[index]));
			}
//			std::cout << "Set (" << x << "," << y << ") to class '" << k << "' with p = " << p <<  std::endl;
			mGrid->setTraversabilityAndProbability(k, p, x, y);
		}
	}
	
	
	envire::OrocosEmitter emitter(&mEnvironment, _envire_map);
//	emitter.setTime(mapTime);
	emitter.flush();
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
	mGrid->setTraversabilityClass(1, envire::TraversabilityClass(1.0));
    mGrid->setTraversabilityClass(0, envire::TraversabilityClass(0.0));

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
