#ifndef SLAM3D_ROCK_ODOMETRY_HPP
#define SLAM3D_ROCK_ODOMETRY_HPP

#include <slam3d/PoseSensor.hpp>
#include <transformer/Transformer.hpp>

namespace slam3d
{
	class RockOdometry : public PoseSensor
	{
	public:
		RockOdometry(const std::string& name, Graph* graph, Logger* logger, transformer::Transformation& tf);
		~RockOdometry();

		void handleNewVertex(IdType vertex);
		
		Transform getPose(timeval stamp);
		Eigen::Affine3d getPose(base::Time t);
		Covariance<6> calculateCovariance(const Transform &tf);

	private:
		transformer::Transformation& mTransformation;
		Transform mCurrentOdometricPose;
		Transform mLastOdometricPose;
		IdType mLastVertex;
	};
}

#endif
