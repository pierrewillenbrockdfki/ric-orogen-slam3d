#ifndef SLAM3D_OCTREEMAPPER_TASK_HPP
#define SLAM3D_OCTREEMAPPER_TASK_HPP

#include <slam3d/OcTreeMapperBase.hpp>
#include <octomap/OcTree.h>

#include "OctoMapConfiguration.hpp"

namespace slam3d
{
	class OcTreeMapper : public OcTreeMapperBase
	{
	friend class OcTreeMapperBase;
	protected:

		// Operations
		virtual bool remove_dynamic_objects();
		
		// Overloaded methods
		virtual void addScanToMap(const VertexObject& scan);
		virtual void rebuildMap(const VertexObjectList& vertices);
		virtual void sendMap();
		
		// Internal methods
		void buildMLS();

		// Members
		octomap::OcTree* mOcTree;
		OctoMapConfiguration mConfiguration;
		
	public:
		OcTreeMapper(std::string const& name = "slam3d::OcTreeMapper");
		OcTreeMapper(std::string const& name, RTT::ExecutionEngine* engine);
		~OcTreeMapper();

		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif

