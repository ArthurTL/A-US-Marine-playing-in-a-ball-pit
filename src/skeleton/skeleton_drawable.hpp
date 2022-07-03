#pragma once

#include "cgp/cgp.hpp"

namespace cgp
{
	struct skeleton_drawable
	{
		skeleton_drawable();
		skeleton_drawable(buffer<affine_rt> const& skeleton, buffer<int> const& parent_index);
		void clear();
		void update(buffer<affine_rt> const& skeleton, buffer<int> const& parent_index);

		float size_frame = 0.05f;
		float size_sphere = 0.01f;
		bool display_segments = true;
		bool display_joint_frame = true;
		bool display_joint_sphere = true;

		segments_drawable segments;
		mesh_drawable joint_frame;
		mesh_drawable joint_sphere;
		buffer<affine_rt> data;
	};

	template <typename SCENE>
	void draw(skeleton_drawable const& skeleton, SCENE const& scene);
}


namespace cgp
{
	template <typename SCENE>
	void draw(skeleton_drawable const& skeleton, SCENE const& scene)
	{
		if(skeleton.display_segments)
			draw(skeleton.segments, scene);
		
		size_t const N = skeleton.data.size();

		if(skeleton.display_joint_frame)
		{
			mesh_drawable joint_frame_temp = skeleton.joint_frame;
			joint_frame_temp.transform.scaling = skeleton.size_frame;
			for (size_t k = 0; k < N; ++k)
			{
				joint_frame_temp.transform.translation = skeleton.data[k].translation;
				joint_frame_temp.transform.rotation = skeleton.data[k].rotation;
				draw(joint_frame_temp, scene);
			}
		}
		
		if (skeleton.display_joint_sphere)
		{
			mesh_drawable joint_sphere_temp = skeleton.joint_sphere;
			joint_sphere_temp.transform.scaling = skeleton.size_sphere;
			for (size_t k = 0; k < N; ++k)
			{
				joint_sphere_temp.transform.translation = skeleton.data[k].translation;
				draw(joint_sphere_temp, scene);
			}
		}

		
	}
}