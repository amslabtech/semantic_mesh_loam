#include"semantic_mesh_loam/pc_to_mesh.h"

int main(int argc, char** argv){

	ros::init(argc, argv, "pc_to_mesh");

	ros::NodeHandle node;
	ros::NodeHandle privateNode("~");

	semloam::PcToMesh pctomesh;

	if( pctomesh.setup(node, privateNode) ){

		pctomesh.spin();

	}

	return 0;
}
