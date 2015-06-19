/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SLAM3D_CONVERT_SCAN_TASK_HPP
#define SLAM3D_CONVERT_SCAN_TASK_HPP

#include "slam3d/convert_scanBase.hpp"

namespace slam3d
{
	class convert_scan : public convert_scanBase
	{
	friend class convert_scanBase;
	
	protected:



	public:
		convert_scan(std::string const& name = "slam3d::convert_scan", TaskCore::TaskState initial_state = Stopped);
		convert_scan(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);
		~convert_scan();
		
		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif

