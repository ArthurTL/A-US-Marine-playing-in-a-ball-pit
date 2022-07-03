#pragma once

#include "cgp/cgp.hpp"
#include "../skinning/skinning.hpp"
#include "../skeleton/skeleton.hpp"


void load_animation_bend_zx(cgp::buffer<cgp::buffer<cgp::affine_rt>>& animation_skeleton, cgp::buffer<float>& animation_time, cgp::buffer<int> const& parent_index);
void load_animation_bend_z(cgp::buffer<cgp::buffer<cgp::affine_rt>>& animation_skeleton, cgp::buffer<float>& animation_time, cgp::buffer<int> const& parent_index);
void load_animation_twist_x(cgp::buffer<cgp::buffer<cgp::affine_rt>>& animation_skeleton, cgp::buffer<float>& animation_time, cgp::buffer<int> const& parent_index);

void load_cylinder(cgp::skeleton_animation_structure& skeleton_data, cgp::rig_structure& rig, cgp::mesh& shape);
void load_rectangle(cgp::skeleton_animation_structure& skeleton_data, cgp::rig_structure& rig, cgp::mesh& shape);


void load_skinning_data(std::string const& directory, cgp::skeleton_animation_structure& skeleton_data, cgp::rig_structure& rig, cgp::mesh& shape, GLuint& texture_id);
void load_skinning_anim(std::string const& directory, cgp::skeleton_animation_structure& skeleton_data);