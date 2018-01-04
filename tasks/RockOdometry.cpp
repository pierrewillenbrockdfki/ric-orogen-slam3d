#include "RockOdometry.hpp"

#include <slam3d/Graph.hpp>
#include <boost/format.hpp>
using namespace slam3d;

RockOdometry::RockOdometry(const std::string& name, Graph* graph, Logger* logger, transformer::Transformation& tf)
 : PoseSensor(name, graph, logger), mTransformation(tf)
{
	mLastVertex = 0;
	mLastOdometricPose = Transform::Identity();
	mCurrentOdometricPose = Transform::Identity();
}

RockOdometry::~RockOdometry()
{
}

void RockOdometry::handleNewVertex(IdType vertex)
{
	if(mLastVertex > 0)
	{
		Transform tf = mLastOdometricPose.inverse() * mCurrentOdometricPose;
		Covariance cov = calculateCovariance(tf);
		mGraph->addConstraint(mLastVertex, vertex, tf, cov, mName, "odometry");
		mGraph->setCorrectedPose(vertex, mGraph->getCurrentPose() * tf);
	}
	mLastVertex = vertex;
	mLastOdometricPose = mCurrentOdometricPose;
}

Transform RockOdometry::getPose(timeval stamp)
{
	base::Time ts = base::Time::fromSeconds(stamp.tv_sec, stamp.tv_usec);
	getPose(ts);
	return mCurrentOdometricPose;
}

Eigen::Affine3d RockOdometry::getPose(base::Time t)
{
	Eigen::Affine3d affine;
	bool res;
	try
	{
		res = mTransformation.get(t, affine, false);
	}catch(std::exception& e)
	{
		throw InvalidPose(e.what());
	}
	
	if(!res)
	{
		transformer::TransformationStatus s = mTransformation.getStatus();
		throw InvalidPose(
			(boost::format("Transformation not available! [success: %1%, no chain: %2%, no sample: %3%, interpolation failed: %4%]")
			% s.generated_transformations
			% s.failed_no_chain
			% s.failed_no_sample
			% s.failed_interpolation_impossible).str());
	}
	
	if((affine.matrix().array() == affine.matrix().array()).all())
	{
		mCurrentOdometricPose.linear() = affine.linear();
		mCurrentOdometricPose.translation() = affine.translation();
	}else
	{
		throw InvalidPose("Odometry sample contained invalid data.");
	}
	
	return affine;
}

Covariance RockOdometry::calculateCovariance(const Transform &tf)
{
	return Covariance::Identity() * 100;
}
