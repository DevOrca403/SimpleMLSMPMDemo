#include <array>

#include <Eigen/Dense>

#include "function.h"
#include "grid.h"
#include "particle_system.h"
#include "particle_update.h"
#include "particle_visualizer.h"
#include "pg_transfer.h"

// material properties
const float E = 0.8e4f;          // Young's Modulus
const float nu = 0.40f;         // Poisson ratio
const float time_delta = 0.0005f;

// Lame parameters
const float lame_mu = E / (2 * (1 + nu));
const float lame_lambda = E * nu / ((1 + nu) * (1 - 2 * nu));

int main() {
    particle_visualizer pv;
    if(!pv.initialize()) {
        abort();
    }
    if(!pv.load_shader()) {
        abort();
    }
    pv.init_buffer();

    grid g;
    particle_system ps;
    ps.add_block(50, 50, 20, 20, 20, 20, {0.0f, 0.0f, -2.0f}, g.get_cell_space());

    // buffers for interpolation
    std::vector<std::array<Vector3f, 3>> weights(ps.size());
    std::vector<Vector3i> base(ps.size());
    std::vector<Vector3f> fracture(ps.size());

    // buffer for rendering
    std::vector<float> render_buffer_pos(ps.size() * 2);
    std::vector<float> render_buffer_value(ps.size());

    // main Loop
    while (!pv.should_close()) {
        g.reset();

        get_neighbor_base(&(*ps.position.begin()), g.get_cell_space(), &(*base.begin()), ps.size());
        get_neighbor_fracture(&(*ps.position.begin()), &(*base.begin()), g.get_cell_space(), &(*fracture.begin()), ps.size());
        get_weight(&(*fracture.begin()), &(*weights.begin()), ps.size());

        //P2G
        p2g_momentum_n_mass(&(*ps.velocity.begin()), &(*weights.begin()), &(*base.begin()), &(*fracture.begin()), g, ps.size());
        p2g_affine_n_force(&(*ps.stress.begin()), &(*ps.affine_velocity.begin()), time_delta, &(*weights.begin()), &(*base.begin()), &(*fracture.begin()), g, ps.size());

        g.update_velocity();
        g.apply_nodal_condition(time_delta);

        //G2P
        g2p_velocity(g, &(*weights.begin()), &(*base.begin()), &(*fracture.begin()), &(*ps.velocity.begin()), ps.size());
        g2p_affine_velocity(g, &(*weights.begin()), &(*base.begin()), &(*fracture.begin()), &(*ps.affine_velocity.begin()), ps.size());

        update_position(&(*ps.velocity.begin()), time_delta, &(*ps.position.begin()), ps.size());
        update_deformation_gradient(&(*ps.affine_velocity.begin()), time_delta, &(*ps.deformation_gradient.begin()), ps.size());
        update_stress(&(*ps.deformation_gradient.begin()), lame_mu, lame_lambda, &(*ps.stress.begin()), ps.size());

        //for (int i = 0; i < ps.size(); ++i) {
        //    render_buffer_pos[2 * i] = ps.position[i].x() * 2.0f - 1.0f;
        //    render_buffer_pos[2 * i + 1] = ps.position[i].y() * 2.0f - 1.0f;
        //    render_buffer_pos[2 * i + 2] = ps.position[i].z() * 2.0f - 1.0f;

        //    render_buffer_value[i] = ps.stress[i].squaredNorm()/700.0f;
        //}

        //pv.render(render_buffer_pos.begin(), render_buffer_pos.end(), render_buffer_value.begin());
    }
}