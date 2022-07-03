#pragma once

#include "cgp/cgp.hpp"



struct particle_structure
{
    cgp::vec3 p; // Position
    cgp::vec3 v; // Speed

    cgp::vec3 c; // Color
    float r;     // Radius
    float m;     // mass
    int index;
    cgp::int3 cell = cgp::int3(-1,-1,-1);
};

static const int GRID_SIZE = 12;
static int NB_COLLISION_CHECK = 0;

void simulate(std::vector<particle_structure>& particles, float dt, cgp::vec3 g,cgp::grid_3D<cgp::buffer<int>>& cells, cgp::buffer<cgp::vec3> position_rest_pose, cgp::buffer<cgp::vec3> skeleton_velocity);
void handlePlaneCollision(particle_structure& particle, cgp::vec3 a);
void handleParticlesCollision(std::vector<particle_structure>& particles,int k, const cgp::grid_3D<cgp::buffer<int>>& cells);
void handleBodyCollision(particle_structure& particle, cgp::buffer<cgp::vec3>skeleton_pos, cgp::buffer<cgp::vec3> skeleton_velocity);
void associateCell(cgp::grid_3D<cgp::buffer<int>>& cells, particle_structure& p);
void particleCollision(particle_structure& p1, particle_structure& p2, int i, int k);
