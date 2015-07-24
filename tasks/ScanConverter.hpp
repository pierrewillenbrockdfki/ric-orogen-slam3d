/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SLAM3D_SCANCONVERTER_TASK_HPP
#define SLAM3D_SCANCONVERTER_TASK_HPP

#include "slam3d/ScanConverterBase.hpp"

namespace slam3d
{
	class ScanConverter : public ScanConverterBase
	{
	friend class ScanConverterBase;
	
	protected:



	public:
		ScanConverter(std::string const& name = "slam3d::ScanConverter", TaskCore::TaskState initial_state = Stopped);
		ScanConverter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);
		~ScanConverter();
		
		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif

