#include "particle_system.h"

void particle_system::add_particle(const Vector3f& _pos, const Vector3f& _vel = Vector3f::Zero()) {
    position.push_back(_pos);
    velocity.push_back(_vel);

    affine_velocity.emplace_back(Matrix3f::Zero());
    deformation_gradient.emplace_back(Matrix3f::Identity());
    stress.emplace_back(Matrix3f::Zero());
}

void particle_system::add_four_particles_in_cell(int _x, int _y, int _z, float _cell_space, const Vector3f & _vel = Vector3f::Zero()) {
    static const std::array<float, 8> X = { 0.25f, 0.25f, 0.25f, 0.25f, 0.75f, 0.75f, 0.75f, 0.75f };
    static const std::array<float, 8> Y = { 0.25f, 0.75f, 0.25f, 0.75f, 0.25f, 0.75f, 0.25f, 0.75f };
    static const std::array<float, 8> Z = { 0.25f, 0.25f, 0.75f, 0.75f, 0.25f, 0.25f, 0.75f, 0.75f };
    for (int i = 0; i < 8; ++i) {
        Vector3f pos = _cell_space * Vector3f(static_cast<float>(_x) + X[i], static_cast<float>(_y) + Y[i], static_cast<float>(_z) + Z[i]);
        add_particle(pos, _vel);
    }
}

void particle_system::add_block(int _pos_x, int _pos_y, int _pos_z, int _size_x, int _size_y, int _size_z, const Vector3f & _vel, float _cell_space) {
    _pos_x -= _size_x / 2;
    _pos_y -= _size_y / 2;
    for (int x = 0; x < _size_x; ++x) {
        for (int y = 0; y < _size_y; ++y) {
            for (int z = 0; z < _size_z; ++z) {
                add_four_particles_in_cell(_pos_x + x, _pos_y + y, _pos_z + z, _cell_space, _vel);
            }
        }
    }
}

size_t particle_system::size() const {
    return position.size();
}