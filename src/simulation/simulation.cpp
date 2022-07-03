#include "simulation.hpp"

using namespace cgp;


void simulate(std::vector<particle_structure>& particles, float dt, vec3 g, cgp::grid_3D<cgp::buffer<int>>& cells, cgp::buffer<cgp::vec3> position_rest_pose, cgp::buffer<cgp::vec3> skeleton_velocity)
{
	//vec3 const g = { 0,0,-9.81f };
	size_t const N = particles.size();
	//cgp::buffer<cgp::buffer<int>> empty = cgp::buffer<cgp::buffer<int>>(GRID_SIZE * GRID_SIZE * GRID_SIZE);
	//cells = grid_3D<cgp::buffer<int>>::from_vector(empty, GRID_SIZE, GRID_SIZE, GRID_SIZE);
	vec3 avg_pos(0,0,0);
	for (vec3 pos : position_rest_pose) {
		avg_pos += pos;
	}
	avg_pos /= position_rest_pose.size();
	avg_pos = (avg_pos + vec3(1.f, 0, 1.f)) / 2.f;
	int3 body_cell = int3(std::max(0, std::min((int)floor(avg_pos.x * GRID_SIZE), GRID_SIZE - 1)),
		std::max(0, std::min((int)floor(avg_pos.y * GRID_SIZE), GRID_SIZE - 1)),
		std::max(0, std::min((int)floor(avg_pos.z * GRID_SIZE), GRID_SIZE - 1)));
	
	for (size_t k = 0; k < N; ++k)
	{
		particle_structure& particle = particles[k];

		vec3 const f = particle.m * g;
		rotation_transform;
		particle.v = (1 - 0.9f * dt) * particle.v + dt * f;
		particle.p = particle.p + dt * particle.v;
		associateCell(cells, particle);
	}
	for (size_t k = 0; k < N; ++k)
	{

		if (particles[k].cell.x >= body_cell.x - 1 && particles[k].cell.x <= body_cell.x + 1 && particles[k].cell.z >= body_cell.z - 1 && particles[k].cell.z <= body_cell.z + 1)
			handleBodyCollision(particles[k], position_rest_pose, skeleton_velocity);
	    handleParticlesCollision(particles, k, cells);
		handlePlaneCollision(particles[k],vec3(0.f, 1.f, -1.f));
		handlePlaneCollision(particles[k], vec3(0.f, 1.f, 1.f));
		handlePlaneCollision(particles[k], vec3(1.f, 1.f, 0.f));
		handlePlaneCollision(particles[k], vec3(-1.f, 1.f, 0.f));
		handlePlaneCollision(particles[k], vec3(0.f, 0.f, 0.f));
		handlePlaneCollision(particles[k], vec3(0.f, 2.f, 0.f));
	    
	}
	//std::cout << NB_COLLISION_CHECK << std::endl;
	if (particles.size() == 50) {
		std::cout << "50" << std::endl;
	}
	if (particles.size() == 100) {
		std::cout << "100" << std::endl;
	}
	if (particles.size() == 150) {
		std::cout << "150" << std::endl;
	}
	if (particles.size() ==200) {
		std::cout << "200" << std::endl;
	}
	if (particles.size() == 250) {
		std::cout << "250" << std::endl;
	}
	if (particles.size() == 300) {
		std::cout << "300" << std::endl;
	}
	if (particles.size() == 350) {
		std::cout << "350" << std::endl;
	}
	if (particles.size() == 400) {
		std::cout << "400" << std::endl;
	}
	
	
}

void handlePlaneCollision(particle_structure& particle, vec3 a) {
	vec3 const n = -(a -vec3(0.f,1.f,0.f));
	float detection = dot(particle.p - a, n);
	if (detection <= particle.r) {
		vec3 const v = particle.v;
		vec3 const v_orth = dot(v, n) * n;
		vec3 const v_col = v - v_orth;
		
		particle.v = 1.f * v_col - 0.75f * v_orth;
		particle.p = particle.p +  (particle.r-detection)/2 * n;
	}
}

void handleBodyCollision(particle_structure& particle, cgp::buffer<cgp::vec3> skeleton_pos, cgp::buffer<cgp::vec3> skeleton_velocity) {
	for (int i = 0; i < skeleton_pos.size();i++) {
		vec3 s = skeleton_pos[i];
		vec3 v_s = skeleton_velocity[i];
		vec3 const dist = particle.p - s;
		if (norm(dist) < 0.02f + particle.r) {
			
			vec3 const n = normalize(dist);
			vec3 const v = particle.v;
			//std::cout <<"Ball : "<<  v << std::endl;
			//std::cout <<"Leg : "<<  v_s << std::endl;
			vec3 const v_orth = dot(v, n) * n;
			vec3 const v_col = v - v_orth;


			particle.v = 1.f * v + 1.f * 1.99f  * dot(v_s - v, n) * n;
			

			
			//particle.v = 1.f * v_col - 1.f * v_orth;
			particle.p = particle.p + (particle.r - (norm(dist) - 0.02f)) / 2 * n;
			
		}
	}
}

void handleParticlesCollision(std::vector<particle_structure>& particles, int k,const cgp::grid_3D<cgp::buffer<int>>& cells) {
	particle_structure&p1 = particles[k];
	
	/*for (int j = 0; j < particles.size(); ++j) {
		if (j != k) {
			particle_structure& p2 = particles[j];
			particleCollision(p1, p2, j, k);
		}
	}*/
	
	int3 c = p1.cell;

	for (int dx = -1; dx < 2; ++dx) {
		for (int dy = -1; dy < 2; ++dy) {
			for (int dz = -1; dz < 2; ++dz) {
				if (c.x + dx >= 0 && c.x + dx < GRID_SIZE  && c.y + dy >= 0 && c.y + dy <GRID_SIZE  && c.z + dz >= 0 && c.z + dz < GRID_SIZE ) {
					for (int j : cells(p1.cell + int3(dx, dy, dz))) {
						if (j != k) {
							particle_structure& p2 = particles[j];
							particleCollision(p1, p2, j, k);
						}

					}

				}
			}
		}
	}
	
		


	
			
	
}

void associateCell(cgp::grid_3D<cgp::buffer<int>>& cells, particle_structure& particle)
{
	
	vec3 p = (particle.p + vec3(1.f, 0, 1.f)) / 2.f;
	int3 cell_id = int3(std::max(0,std::min((int)floor(p.x * GRID_SIZE),GRID_SIZE-1)), 
		std::max(0, std::min((int)floor(p.y * GRID_SIZE), GRID_SIZE - 1)),
		std::max(0, std::min((int)floor(p.z * GRID_SIZE), GRID_SIZE - 1)));
	
	if (is_equal(particle.cell, int3(-1,-1,-1))) {   //initialization
		particle.cell = cell_id;
		cells(cell_id).push_back(particle.index);
	}
	else if (!is_equal(particle.cell, cell_id)) {
		const buffer<int>& prev_cell = cells(particle.cell);
		buffer<int> new_cell;
		for (int i : prev_cell) {
			if (i != particle.index)
				new_cell.push_back(i);
		};

		cells(particle.cell) = new_cell;
		particle.cell = cell_id;
		cells(cell_id).push_back(particle.index);
		
	}
	
	
}

void particleCollision(particle_structure& p1, particle_structure& p2, int i,int k) {
	assert_cgp_no_msg(i != k);
	float const dist = norm(p1.p - p2.p);
	NB_COLLISION_CHECK++;
	if (dist <= p1.r + p2.r) {

		vec3 u = (p1.p - p2.p) / dist;
		vec3 const v1 = p1.v;
		vec3 const v2 = p2.v;
		p1.v = 1.f * v1 + 0.95f * (2 * p2.m / (p1.m + p2.m)) * dot(v2 - v1, u) * u;
		p2.v = 1.f * v2 - 0.95f * (2 * p1.m / (p1.m + p2.m)) * dot(v2 - v1, u) * u;

		float d = p1.r + p2.r - dist;
		p1.p += d / 2.f * u;
		p2.p -= d / 2.f * u;
	}
}