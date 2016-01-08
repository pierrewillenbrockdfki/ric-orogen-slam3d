#ifndef SLAM3D_TRAVERSABILITYMAPPROJECTOR_TASK_HPP
#define SLAM3D_TRAVERSABILITYMAPPROJECTOR_TASK_HPP

#include "slam3d/TraversabilityMapProjectorBase.hpp"

#include <envire/operators/TraversabilityProjection.hpp>

namespace slam3d
{
	class TraversabilityMapProjector : public TraversabilityMapProjectorBase
	{
	friend class TraversabilityMapProjectorBase;

	protected:
		typedef std::vector<envire::BinaryEvent> EnvireEvents;
		typedef RTT::extras::ReadOnlyPointer<EnvireEvents> EnvirePointer;

	private:
		envire::Environment mEnvironment;
		envire::Pointcloud* mPointcloud;
		envire::TraversabilityGrid* mGrid;
		envire::TraversabilityProjection* mProjection;
		envire::BinarySerialization mBinarySerialization;

		double mSizeX;
		double mSizeY;
		double mOffsetX;
		double mOffsetY;
		double mMinZ;
		double mMaxZ;
		double mResolution;

	public:
		TraversabilityMapProjector(std::string const& name = "slam3d::TraversabilityMapProjector", TaskCore::TaskState initial_state = Stopped);
		TraversabilityMapProjector(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);
		~TraversabilityMapProjector();

		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif

