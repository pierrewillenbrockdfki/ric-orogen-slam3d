#include <slam3d/Logger.hpp>
#include <slam3d/Clock.hpp>
#include <slam3d/Odometry.hpp>

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
	
	class BaseLogger : public slam3d::Logger
	{
	public:
		BaseLogger() : slam3d::Logger(slam3d::Clock()){}
		~BaseLogger(){}
		
		virtual void message(slam3d::LOG_LEVEL lvl, const std::string& msg)
		{
			switch(lvl)
			{
				case slam3d::DEBUG:
					LOG_DEBUG("%s", msg.c_str());
					break;
				case slam3d::INFO:
					LOG_INFO("%s", msg.c_str());
					break;
				case slam3d::WARNING:
					LOG_WARN("%s", msg.c_str());
					break;
				case slam3d::ERROR:
					LOG_ERROR("%s", msg.c_str());
					break;
				case slam3d::FATAL:
					LOG_FATAL("%s", msg.c_str());
					break;
			}
		}
	};
	
	class RockOdometry : public slam3d::Odometry
	{
	public:
		RockOdometry(slam3d::Logger* logger, double tolerance = 100) : slam3d::Odometry(logger)
		{
			mCurrentPose = slam3d::Transform::Identity();
			mTolerance = tolerance; // milliseconds

		}
		~RockOdometry() {}
		
		// Rock transformer cannot be queried for a specific timestamp.
		// So we have to rely on current pose to match last received measurement.
		slam3d::Transform getOdometricPose(timeval stamp)
		{
			long diff = timevaldiff(stamp, mCurrentTime);
			if(diff > mTolerance)
			{
				mLogger->message(slam3d::ERROR, (boost::format("Odometry data stamp differs from requested stamp by %1% ms.")%diff).str());
				throw slam3d::OdometryException();
			}
			return mCurrentPose;
		}
		
		slam3d::TransformWithCovariance getRelativePose(timeval last, timeval next)
		{
			slam3d::TransformWithCovariance twc;
			twc.transform = slam3d::Transform::Identity();
			twc.covariance = slam3d::Covariance::Identity();
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
				mLogger->message(slam3d::ERROR, "Odometry sample contained invalid data!");
				throw slam3d::OdometryException();
			}
		}
		
	private:
		slam3d::Transform mCurrentPose;
		timeval mCurrentTime;
		long mTolerance;
	};
}