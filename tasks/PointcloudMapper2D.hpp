#ifndef SLAM3D_POINTCLOUDMAPPER2D_TASK_HPP
#define SLAM3D_POINTCLOUDMAPPER2D_TASK_HPP

#include "slam3d/PointcloudMapper2DBase.hpp"

namespace slam3d
{
	class PointcloudMapper2D : public PointcloudMapper2DBase
	{
	friend class PointcloudMapper2DBase;

	protected:



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

