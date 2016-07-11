#ifndef SLAM3D_CONFIGURATION_HPP
#define SLAM3D_CONFIGURATION_HPP

namespace slam3d
{
	struct OctoMapConfiguration
	{
		double occupancyThres;
		double probHit;
		double probMiss;
		double clampingThresMin;
		double clampingThresMax;
		double rangeMax;
		
		OctoMapConfiguration() : occupancyThres(0.5),
		                         probHit(0.7),
		                         probMiss(0.4),
		                         clampingThresMin(0.1192),
		                         clampingThresMax(0.971),
		                         rangeMax(-1){}
	};
}

#endif