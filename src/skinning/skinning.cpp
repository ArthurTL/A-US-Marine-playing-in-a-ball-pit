#include "skinning.hpp"

namespace cgp
{
	void normalize_weights(buffer<buffer<float>>& weights)
	{
		size_t const N = weights.size();
		for (size_t k = 0; k < N; ++k) {
			float s = 0.0f;
			for(float w : weights[k]) s += w;
			assert_cgp_no_msg(s>1e-5f);
			for(float& w : weights[k]) w /= s;
		}
	}


	// Linear Blend Skinning
	void skinning_LBS_compute(
		buffer<vec3>& position_skinned,  // position to deform
		buffer<vec3>& normal_skinned,    // normal to deform
		buffer<affine_rt> const& skeleton_current,    // rigid transforms for skeleton joints in current pose
		buffer<affine_rt> const& skeleton_rest_pose,  // rigid transforms of skeleton joints in rest pose
		buffer<vec3> const& position_rest_pose,       // vertex positions of the mesh in rest pose
		buffer<vec3> const& normal_rest_pose,         // normal coordinates of the mesh in rest pose
		rig_structure const& rig)                     // information of the skinning weights (joints and weights associated to a vertex)
	{
		size_t const N_vertex = position_rest_pose.size();
		size_t const N_joint = skeleton_current.size();

		// Sanity check on sizes of buffers
		assert_cgp_no_msg(position_skinned.size()==N_vertex);
		assert_cgp_no_msg(normal_skinned.size()==N_vertex);
		assert_cgp_no_msg(normal_rest_pose.size()==N_vertex);
		assert_cgp_no_msg(skeleton_rest_pose.size()==N_joint);
		assert_cgp_no_msg(rig.joint.size()==N_vertex);
		assert_cgp_no_msg(rig.weight.size()==N_vertex);
		
		// To do
		//   Compute the Linear Blend Skinning ...
		for (int i = 0; i < N_vertex; ++i) {
			vec3 newPos = vec3(0.f, 0.f, 0.f);
			vec3 newNormal = vec3(0.f, 0.f, 0.f);
			quaternion q_p = quaternion();
			quaternion qe_p = quaternion();
			for (int w_i = 0; w_i < rig.weight[i].size();++w_i) {

		
			
				affine_rt T_inv = inverse(skeleton_rest_pose[rig.joint[i][w_i]]);
				affine_rt M = skeleton_current[rig.joint[i][w_i]] * T_inv;
				newPos += M * position_rest_pose[i] * rig.weight[i][w_i];
				newNormal += M * normal_rest_pose[i] * rig.weight[i][w_i];


			/*
				q_p += skeleton_current[rig.joint[i][w_i]].rotation.quat()* rig.weight[i][w_i];
				vec3 t = skeleton_current[rig.joint[i][w_i]].translation;
				qe_p += quaternion(t.x, t.y, t.z, 0) * skeleton_current[rig.joint[i][w_i]].rotation.quat() * rig.weight[i][w_i] / 2.f;*/

			}
			/*
			quaternion q0 = normalize(q_p);
			quaternion qt = 2 * qe_p * conjugate(q0) / normalize(q_p);
			position_skinned[i] = rotation_transform::from_quaternion(q0).matrix() * position_rest_pose[i] + vec3(qt.x, qt.y, qt.z) ;*/

			position_skinned[i] = newPos;
			normal_skinned[i] = newNormal;

		}


	}

}