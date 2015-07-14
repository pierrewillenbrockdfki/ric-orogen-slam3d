/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SLAM3D_INDOOR_FILTER_TASK_HPP
#define SLAM3D_INDOOR_FILTER_TASK_HPP

#include "slam3d/indoor_filterBase.hpp"

namespace slam3d
{
	class indoor_filter : public indoor_filterBase
	{
	friend class indoor_filterBase;
	
	protected:
		double mMinHeight;
		double mMaxHeight;
		double mSqMinDistance;
		double mSqMaxDistance;

	public:
		indoor_filter(std::string const& name = "slam3d::indoor_filter", TaskCore::TaskState initial_state = Stopped);
		indoor_filter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);
		~indoor_filter();
		
		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif

