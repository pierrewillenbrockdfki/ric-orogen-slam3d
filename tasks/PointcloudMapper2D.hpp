#ifndef SLAM3D_POINTCLOUDMAPPER2D_TASK_HPP
#define SLAM3D_POINTCLOUDMAPPER2D_TASK_HPP

#include "slam3d/PointcloudMapper2DBase.hpp"

#include <envire/maps/TraversabilityGrid.hpp>

namespace slam3d
{
	typedef uint16_t grid_t;

	class PointcloudMapper2D : public PointcloudMapper2DBase
	{
	friend class PointcloudMapper2DBase;

	protected:

		// Envire stuff for Traversability-Grid
		typedef std::vector<envire::BinaryEvent> EnvireEvents;
		typedef RTT::extras::ReadOnlyPointer<EnvireEvents> EnvirePointer;

		// Internal methods
		virtual void addScanToMap(const VertexObject& scan);
		virtual void rebuildMap(const VertexObjectList& vertices);
		virtual void sendMap();

		// Members
		envire::TraversabilityGrid* mGrid;
		grid_t* mOccCells;
		grid_t* mHitCells;

		// Parameters
		double mSizeX;
		double mSizeY;
		double mOffsetX;
		double mOffsetY;
		double mMinZ;
		double mMaxZ;
		double mResolution;
		
	public:
		PointcloudMapper2D(std::string const& name = "slam3d::PointcloudMapper2D");
		PointcloudMapper2D(std::string const& name, RTT::ExecutionEngine* engine);
		~PointcloudMapper2D();
		
		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif

