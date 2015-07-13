/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SLAM3D_MLS_RENDERER_TASK_HPP
#define SLAM3D_MLS_RENDERER_TASK_HPP

#include "slam3d/mls_rendererBase.hpp"

#include <envire/Orocos.hpp>
#include <envire/operators/MLSProjection.hpp>

namespace slam3d
{
	class mls_renderer : public mls_rendererBase
	{
	friend class mls_rendererBase;
	
	protected:
		typedef std::vector<envire::BinaryEvent> EnvireEvents;
		typedef RTT::extras::ReadOnlyPointer<EnvireEvents> EnvirePointer;
	
	protected:
		envire::Environment mEnvironment;
		envire::Pointcloud* mPointcloud;
		envire::MultiLevelSurfaceGrid* mMultiLayerMap;
		envire::MLSProjection* mProjection;
		envire::BinarySerialization mBinarySerialization;

	public:
		mls_renderer(std::string const& name = "slam3d::mls_renderer", TaskCore::TaskState initial_state = Stopped);
		mls_renderer(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);
		~mls_renderer();
		
		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif

