#include <slam3d/include/Logger.hpp>
#include <slam3d/include/Clock.hpp>
#include <slam3d/include/Odometry.hpp>

#include <base/Logging.hpp>

namespace slam3d
{
	class BaseLogger : public slam::Logger
	{
	public:
		BaseLogger() : slam::Logger(slam::Clock()){}
		~BaseLogger(){}
		
		virtual void message(slam::LOG_LEVEL lvl, const std::string& msg)
		{
			switch(lvl)
			{
				case slam::DEBUG:
					LOG_DEBUG("%s", msg.c_str());
					break;
				case slam::INFO:
					LOG_INFO("%s", msg.c_str());
					break;
				case slam::WARNING:
					LOG_WARN("%s", msg.c_str());
					break;
				case slam::ERROR:
					LOG_ERROR("%s", msg.c_str());
					break;
				case slam::FATAL:
					LOG_FATAL("%s", msg.c_str());
					break;
			}
		}
	};
	
	class RockOdometry : public slam::Odometry
	{
	public:
		RockOdometry() : slam::Odometry() {}
		~RockOdometry() {}
		
		// Rock transformer cannot be queried for a specific timestamp.
		// So we have to rely on current pose to match last received measurement.
		slam::Transform getOdometricPose(timeval stamp)
		{
			return mCurrentPose;
		}
		
		slam::TransformWithCovariance getRelativePose(timeval last, timeval next)
		{
			slam::TransformWithCovariance twc;
			twc.transform = slam::Transform::Identity();
			twc.covariance = slam::Covariance::Identity();
			return twc;
		}
		
		void setCurrentPose(const slam::Transform& pose)
		{
			mCurrentPose = pose;
		}
		
	private:
		slam::Transform mCurrentPose;
	};
}