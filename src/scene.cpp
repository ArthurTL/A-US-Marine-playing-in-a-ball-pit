#include "scene.hpp"
#include "loader/skinning_loader.hpp"

using namespace cgp;


void scene_structure::display()
{
	timer.update();
	environment.light = environment.camera.position();


	// Create a new particle if needed
	emit_particle();

	// Call the simulation of the particle system
	float const dt = 0.01f * timer.scale;
	simulate(particles, dt, g,cells, skinning_data.position_skinned, skinning_data.velocity_skinned);

	// Display the result
	sphere_display();


	if (gui.display_frame)
		draw(global_frame, environment);

	// Display the character
	timer_skeleton.update();
	if (move_speed>0.f &&!walking) {
		load_skinning_anim("assets/marine/anim_walk/", skeleton_data);
		update_anim();
		walking = true;
	}

	if (move_speed<0.01f && walking) {
		load_skinning_anim("assets/marine/anim_idle/", skeleton_data);
		update_anim();
		walking = false;
	}
	compute_deformation();
	draw(visual_data.surface_skinned, environment);
}


void scene_structure::sphere_display()
{
	// Display the particles as spheres
	size_t const N = particles.size();
	for (size_t k = 0; k < N; ++k)
	{
		particle_structure const& particle = particles[k];
		sphere.shading.color = particle.c;
		sphere.transform.translation = particle.p;
		sphere.transform.scaling = particle.r;
		draw(sphere, environment);
	}

	// Display the box in which the particles should stay
	draw(cube_wireframe, environment);
}




void scene_structure::initialize()
{

	// Common element of the scene
	// ************************************ //
	global_frame.initialize(mesh_primitive_frame(), "Frame");
	environment.camera.look_at({ 3.0f,2.0f,2.0f }, { 0,1,0 }, { 0,1,0 });
	timer.event_period = 0.5f;

	// Edges of the containing cube
	//  Note: this data structure is set for display purpose - don't use it to compute some information on the cube - it would be un-necessarily complex
	buffer<vec3> cube_wireframe_data = { {-1,0,-1},{1,0,-1}, {1,0,-1},{1,2,-1}, {1,2,-1},{-1,2,-1}, {-1,2,-1},{-1,0,-1},
		{-1,0,1} ,{1,0,1},  {1,0,1}, {1,2,1},  {1,2,1}, {-1,2,1},  {-1,2,1}, {-1,0,1},
		{-1,0,-1},{-1,0,1}, {1,0,-1},{1,0,1}, {1,2,-1},{1,2,1},   {-1,2,-1},{-1,2,1} };
	cube_wireframe.initialize(cube_wireframe_data, "cube wireframe");

	sphere.initialize(mesh_primitive_sphere(), "Sphere");

	
	texture_id = mesh_drawable::default_texture;
	load_skinning_data("assets/marine/", skeleton_data, rig, shape, texture_id);
	load_skinning_anim("assets/marine/anim_idle/", skeleton_data);
	normalize_weights(rig.weight);
	float const scaling = 0.007f;
	for (auto& p : shape.position) p *= scaling;
	skeleton_data.scale(scaling);
	update_new_content(shape, texture_id);
}

void scene_structure::emit_particle()
{
	// Emit particle with random velocity
	//  Assume first that all particles have the same radius and mass
	static buffer<vec3> const color_lut = { {1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1} };
	if (timer.event && gui.add_sphere) {
		float const theta = rand_interval(0, 2 * Pi);
		vec3 const v = vec3( std::cos(theta), 1.0f, std::sin(theta));

		particle_structure particle;
		particle.p = { 0,1.f,-0.4f };
		particle.r = 0.07f;
		particle.c = color_lut[int(rand_interval() * color_lut.size())];
		particle.v = v;
		particle.m = 1.0f; 
		particle.index = N;
		N++;
		particles.push_back(particle);
	}
}


void scene_structure::display_gui()
{
	ImGui::Checkbox("Frame", &gui.display_frame);
	ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
	ImGui::SliderFloat("Time to add new sphere", &timer.event_period, 0.05f, 2.0f, "%.2f s");
	ImGui::Checkbox("Add sphere", &gui.add_sphere);
	ImGui::Checkbox("Lock gravity", &gui.lock_gravity);
}

void scene_structure::update_new_content(mesh const& shape, GLuint texture_id)
{
	visual_data.surface_skinned.clear();
	visual_data.surface_skinned.initialize(shape, "Skinned surface");
	visual_data.surface_skinned.texture = texture_id;

	visual_data.surface_rest_pose.clear();
	visual_data.surface_rest_pose.initialize(shape, "Rest pose");
	visual_data.surface_rest_pose.texture = texture_id;

	skinning_data.position_rest_pose = shape.position;
	skinning_data.position_skinned = skinning_data.position_rest_pose;
	skinning_data.velocity_skinned.resize(skinning_data.position_skinned.size());
	skinning_data.velocity_skinned.fill(vec3(0,0,0));
	skinning_data.normal_rest_pose = shape.normal;
	skinning_data.normal_skinned = skinning_data.normal_rest_pose;

	skinning_data.skeleton_current = skeleton_data.rest_pose_global();
	skinning_data.skeleton_rest_pose = skinning_data.skeleton_current;

	visual_data.skeleton_current.clear();
	visual_data.skeleton_current = skeleton_drawable(skinning_data.skeleton_current, skeleton_data.parent_index);

	visual_data.skeleton_rest_pose.clear();
	visual_data.skeleton_rest_pose = skeleton_drawable(skinning_data.skeleton_rest_pose, skeleton_data.parent_index);

	timer_skeleton.t_min = skeleton_data.animation_time[0];
	timer_skeleton.t_max = skeleton_data.animation_time[skeleton_data.animation_time.size() - 1];
	timer_skeleton.t = skeleton_data.animation_time[0];
}

void scene_structure::compute_deformation()
{
	float const t = timer_skeleton.t;
	skinning_data.velocity_skinned = skinning_data.position_skinned;
	skinning_data.temp_local = skeleton_data.evaluate_local(t);
	move();
	skinning_data.skeleton_current = skeleton_data.evaluate_global(skinning_data.temp_local,t);

	visual_data.skeleton_current.update(skinning_data.skeleton_current, skeleton_data.parent_index);

	// Compute skinning deformation
	skinning_LBS_compute(skinning_data.position_skinned, skinning_data.normal_skinned,
		skinning_data.skeleton_current, skinning_data.skeleton_rest_pose,
		skinning_data.position_rest_pose, skinning_data.normal_rest_pose,
		rig);
	visual_data.surface_skinned.update_position(skinning_data.position_skinned);
	visual_data.surface_skinned.update_normal(skinning_data.normal_skinned);
	skinning_data.velocity_skinned -= skinning_data.position_skinned;
	skinning_data.velocity_skinned /= (t > 0.001 ? -t:-100000.f);
	
}

void scene_structure::move() {
	
	//Pivoting
	rotation = rotation * rotation_transform::between_vector(normalize(vec3(1, 0, 0)), normalize(vec3(1, rot_speed, 0)));
	skinning_data.temp_local[0].rotation = rotation;

	//Walking forward
	front_vec = rotation_transform::between_vector(normalize(vec3(0, 0, -1)), normalize(vec3(-rot_speed, 0, -1))).matrix()*front_vec  ;
	offset = vec3(clamp(offset.x+front_vec.x * move_speed,-0.9f,0.9f),offset.y, clamp(offset.z + front_vec.z * move_speed, -0.9f, 0.9f));
	skinning_data.temp_local[0].translation += offset;
}

void scene_structure::walk(bool b) {
	move_speed = b ? 0.025f : 0.f;
}

void scene_structure::change_rot(float speed) {
	rot_speed = speed;
}

void scene_structure::update_anim() {
	timer_skeleton.t_min = skeleton_data.animation_time[0];
	timer_skeleton.t_max = skeleton_data.animation_time[skeleton_data.animation_time.size() - 1];
	timer_skeleton.t = skeleton_data.animation_time[0];
	float const scaling = 0.007f;
	for (auto& p : shape.position) p *= scaling;
	skeleton_data.scale(scaling);
}