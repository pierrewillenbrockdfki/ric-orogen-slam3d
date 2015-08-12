#include <slam3d/include/Logger.hpp>
#include <slam3d/include/Clock.hpp>
#include <slam3d/include/Odometry.hpp>

#include <base/samples/RigidBodyState.hpp>
#include <base/Logging.hpp>

namespace slam3d
{
	long timevaldiff(const timeval& start, const timeval& end)
	{
		long msec;
		msec=(end.tv_sec - start.tv_sec)*1000;
		msec+=(end.tv_usec - start.tv_usec)/1000;
		if(msec > 0)
			return msec;
		else
			return -msec;
	}
	
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
		RockOdometry(slam::Logger* logger, double tolerance = 100) : slam::Odometry(logger)
		{
			mCurrentPose = slam::Transform::Identity();
			mTolerance = tolerance; // milliseconds

		}
		~RockOdometry() {}
		
		// Rock transformer cannot be queried for a specific timestamp.
		// So we have to rely on current pose to match last received measurement.
		slam::Transform getOdometricPose(timeval stamp)
		{
			long diff = timevaldiff(stamp, mCurrentTime);
			if(diff > mTolerance)
			{
				mLogger->message(slam::ERROR, (boost::format("Odometry data stamp differs from requested stamp by %1% ms.")%diff).str());
				throw slam::OdometryException();
			}
			return mCurrentPose;
		}
		
		slam::TransformWithCovariance getRelativePose(timeval last, timeval next)
		{
			slam::TransformWithCovariance twc;
			twc.transform = slam::Transform::Identity();
			twc.covariance = slam::Covariance::Identity();
			return twc;
		}
		
		void setCurrentPose(const base::samples::RigidBodyState& pose)
		{
			Eigen::Affine3d odom_aff = pose.getTransform();
			if((odom_aff.matrix().array() == odom_aff.matrix().array()).all())
			{
				mCurrentPose.linear() = odom_aff.linear();
				mCurrentPose.translation() = odom_aff.translation();
				mCurrentTime = pose.time.toTimeval();
			}else
			{
				mLogger->message(slam::ERROR, "Odometry sample contained invalid data!");
				throw slam::OdometryException();
			}
		}
		
	private:
		slam::Transform mCurrentPose;
		timeval mCurrentTime;
		long mTolerance;
	};
}