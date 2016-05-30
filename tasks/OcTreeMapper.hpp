/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SLAM3D_OCTREEMAPPER_TASK_HPP
#define SLAM3D_OCTREEMAPPER_TASK_HPP

#include <slam3d/OcTreeMapperBase.hpp>
#include <octomap/OcTree.h>

namespace slam3d
{
	class OcTreeMapper : public OcTreeMapperBase
	{
	friend class OcTreeMapperBase;
	protected:

		// Operations
		virtual bool generate_map();

		// Internal methods
        void addScanToOctoMap(const VertexObject& scan);
		void buildOcTree(const VertexObjectList& vertices);
		void buildMLS();

		// Members
		octomap::OcTree* mOcTree;
		
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

