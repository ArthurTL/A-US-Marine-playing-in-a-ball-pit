#pragma once

#include "cgp/cgp.hpp"

#include "simulation/simulation.hpp"

#include "skeleton/skeleton.hpp"
#include "skeleton/skeleton_drawable.hpp"
#include "skinning/skinning.hpp"



// The element of the GUI that are not already stored in other structures
struct gui_parameters {
	bool display_frame = false;
	bool add_sphere    = true;
	bool lock_gravity = true;
};


struct visual_shapes_parameters
{
	cgp::mesh_drawable surface_skinned;
	cgp::mesh_drawable surface_rest_pose;
	cgp::skeleton_drawable skeleton_current;
	cgp::skeleton_drawable skeleton_rest_pose;
};

struct skinning_current_data
{
	cgp::buffer<cgp::vec3> position_rest_pose;
	cgp::buffer<cgp::vec3> position_skinned;
	cgp::buffer<cgp::vec3> velocity_skinned;
	cgp::buffer<cgp::vec3> normal_rest_pose;
	cgp::buffer<cgp::vec3> normal_skinned;

	cgp::buffer<cgp::affine_rt> temp_local;
	cgp::buffer<cgp::affine_rt> skeleton_current;
	cgp::buffer<cgp::affine_rt> skeleton_rest_pose;
};

// The structure of the custom scene
struct scene_structure {
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //

	cgp::mesh_drawable global_frame;          // The standard global frame
	cgp::scene_environment_basic environment; // Standard environment controler
	gui_parameters gui;                       // Standard GUI element storage
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	cgp::timer_event_periodic timer;
	cgp::mesh shape;
	GLuint texture_id;

	std::vector<particle_structure> particles;
	int N = 0;
	cgp::grid_3D<cgp::buffer<int>> cells = cgp::grid_3D<cgp::buffer<int>>::from_vector(cgp::buffer<cgp::buffer<int>>(GRID_SIZE * GRID_SIZE * GRID_SIZE), GRID_SIZE, GRID_SIZE, GRID_SIZE);
	cgp::mesh_drawable sphere;
	cgp::segments_drawable cube_wireframe;
	cgp::vec3  g = { 0,-9.81f,0 };

	cgp::timer_interval timer_skeleton;
	visual_shapes_parameters visual_data;
	cgp::skeleton_animation_structure skeleton_data;
	cgp::rig_structure rig;
	skinning_current_data skinning_data;
	cgp::int3 move_dir = { 0,0,0 };
	cgp::vec3 offset = { 0,0,0 };
	cgp::rotation_transform rotation = cgp::rotation_transform::between_vector(normalize(cgp::vec3(1, 1, 0)), normalize(cgp::vec3(-1, 0, -1)));
	cgp::vec3 front_vec = { 0,0,-1.f };
	float move_speed = 0.0f;
	float rot_speed = 0.f;
	float angle = 0.f;
	bool walking = false;
	// ****************************** //
	// Functions
	// ****************************** //

	void initialize();  // Standard initialization to be called before the animation loop
	void display();     // The frame display to be called within the animation loop
	void display_gui(); // The display of the GUI, also called within the animation loop

	void emit_particle();
	void simulation_step(float dt);
	void sphere_display();


	void compute_deformation();
	void move();
	void walk(bool b);
	void change_rot(float speed);
	void update_anim();
	void update_new_content(cgp::mesh const& shape, GLuint texture_id);
};





