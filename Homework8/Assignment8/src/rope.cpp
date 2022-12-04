#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        if ((end - start).norm2() == 0. || num_nodes < 2)
            throw std::runtime_error("invalid arguments");
        Vector2D interval = (end - start) / (double)(num_nodes - 1);
        masses.push_back(new Mass(start, node_mass, false));
        for (int i = 1; i < num_nodes; ++i) {
            masses.push_back(new Mass(start + (double)i * interval, node_mass, false));
            springs.push_back(new Spring(masses[i-1], masses[i], k));
        }
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    Rope::~Rope()
    {
        for (auto &ptr : masses) {
            delete ptr;
            ptr = nullptr;
        }
        for (auto &ptr : springs) {
            delete ptr;
            ptr = nullptr;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Mass *a = s->m1;
            Mass *b = s->m2;
            Vector2D ab = b->position - a->position;
            a->forces += s->k * ab / ab.norm() * (ab.norm() - s->rest_length);
            b->forces += -(s->k * ab / ab.norm() * (ab.norm() - s->rest_length));
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity;
                Vector2D a = m->forces / m->mass;

                // explicit method (not stable)
                // m->position = m->position + m->velocity * delta_t;
                // m->velocity = m->velocity + a * delta_t;

                // semi-implicit method
                m->velocity = m->velocity + a * delta_t;
                m->position = m->position + m->velocity * delta_t;

                // TODO (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        const double damping = 0.00005;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                Vector2D a = gravity / m->mass;
                // TODO (Part 3.1): Set the new position of the rope mass
                // m->position += m->position - m->last_position + a * delta_t * delta_t;
                // TODO (Part 4): Add global Verlet damping
                m->position += (1. - damping) * (m->position - m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
            }
        }

        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D center = (s->m1->position + s->m2->position) / 2.;
            Mass *a = s->m1;
            Mass *b = s->m2;
            Vector2D ab = b->position - a->position;
            Vector2D dir = ab.unit();
            if (!b->pinned)
                b->position = center + dir * s->rest_length / 2.;
            if (!a->pinned)
                a->position = center - dir * s->rest_length / 2.;
        }
    }
}
