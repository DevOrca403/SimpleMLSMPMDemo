#include "particle_system.h"

void particle_system::add_particle(const Vector2f& _pos, const Vector2f& _vel) {
    position.push_back(_pos);
    velocity.push_back(_vel);

    affine_velocity.emplace_back(Matrix2f::Zero());
    deformation_gradient.emplace_back(Matrix2f::Identity());
    stress.emplace_back(Matrix2f::Zero());
}

void particle_system::add_four_particles_in_cell(int _x, int _y, float _cell_space, const Vector2f& _vel) {
    static const std::array<float, 4> X = {0.25f, 0.75f, 0.25f, 0.75f};
    static const std::array<float, 4> Y = {0.25f, 0.25f, 0.75f, 0.75f};
    for (int i = 0; i < 4; ++i) {
        Vector2f pos = _cell_space * Vector2f(static_cast<float>(_x) + X[i], static_cast<float>(_y) + Y[i]);
        add_particle(pos, _vel);
    }
}

void particle_system::add_block(int _pos_x, int _pos_y, int _size_x, int _size_y, const Vector2f& _vel, float _cell_space) {
    _pos_x -= _size_x / 2;
    _pos_y -= _size_y / 2;
    for (int x = 0; x < _size_x; ++x) {
        for (int y = 0; y < _size_y; ++y) {
            add_four_particles_in_cell(_pos_x + x, _pos_y + y, _cell_space, _vel);
        }
    }
}

size_t particle_system::size() const {
    return position.size();
}