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
	
	class BaseLogger : public Logger
	{
	public:
		BaseLogger() : Logger(Clock()){}
		~BaseLogger(){}
		
		virtual void message(LOG_LEVEL lvl, const std::string& msg)
		{
			switch(lvl)
			{
				case DEBUG:
					LOG_DEBUG("%s", msg.c_str());
					break;
				case INFO:
					LOG_INFO("%s", msg.c_str());
					break;
				case WARNING:
					LOG_WARN("%s", msg.c_str());
					break;
				case ERROR:
					LOG_ERROR("%s", msg.c_str());
					break;
				case FATAL:
					LOG_FATAL("%s", msg.c_str());
					break;
			}
		}
	};
	
	class RockOdometry : public Odometry
	{
	public:
		RockOdometry(transformer::Transformation& tf, Logger* logger)
		: Odometry(logger), mTransformation(tf){}

		~RockOdometry() {}
		
		Transform getOdometricPose(timeval stamp)
		{
			base::Time ts = base::Time::fromSeconds(stamp.tv_sec, stamp.tv_usec);
			Eigen::Affine3d affine = getOdometricPose(ts);
			
			Transform tf;
			if((affine.matrix().array() == affine.matrix().array()).all())
			{
				tf.linear() = affine.linear();
				tf.translation() = affine.translation();
			}else
			{
				mLogger->message(ERROR, "Odometry sample contained invalid data!");
				throw OdometryException();
			}
			return tf;
		}
		
		Eigen::Affine3d getOdometricPose(base::Time t)
		{
			Eigen::Affine3d odom;
			bool res;
			try
			{
				res = mTransformation.get(t, odom, false);
			}catch(std::exception& e)
			{
				mLogger->message(ERROR, e.what());
				throw OdometryException();
			}
			
			if(!res)
			{
				transformer::TransformationStatus s = mTransformation.getStatus();
				mLogger->message(ERROR, (boost::format("Transformation not available! [success: %1%, no chain: %2%, no sample: %3%, interpolation failed: %4%]") % s.generated_transformations % s.failed_no_chain % s.failed_no_sample % s.failed_interpolation_impossible).str());
				throw OdometryException();
			}
			
			return odom;
		}
		
		TransformWithCovariance getRelativePose(timeval last, timeval next)
		{
			TransformWithCovariance twc;
			twc.transform = Transform::Identity();
			twc.covariance = Covariance::Identity();
			return twc;
		}

	private:
		transformer::Transformation& mTransformation;
	};
}
