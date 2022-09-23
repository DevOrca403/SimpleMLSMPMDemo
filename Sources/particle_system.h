#ifndef ORCAMLSMPMDEMO_PARTICLE_SYSTEM_H
#define ORCAMLSMPMDEMO_PARTICLE_SYSTEM_H

#include <vector>
#include <array>

#include <Eigen/Dense>


using namespace Eigen;

struct particle_system {
    std::vector<Vector3f> position;
    std::vector<Vector3f> velocity;
    std::vector<Matrix3f> affine_velocity;

    std::vector<Matrix3f> deformation_gradient;
    std::vector<Matrix3f> stress;

    static constexpr float particle_mass = 1.0f;
    static constexpr float vol0 = 1.0f;

    void add_particle(const Vector3f& _pos, const Vector3f& _vel);
    void add_four_particles_in_cell(int _x, int _y, int _z, float _cell_space, const Vector3f& _vel);
    void add_block(int _pos_x, int _pos_y, int _pos_z, int _size_x, int _size_y, int _size_z, const Vector3f& _vel, float _cell_space);

    [[nodiscard]] size_t size() const;
    [[nodiscard]] static constexpr float mass();
    [[nodiscard]] static constexpr float initial_volume();
};

constexpr float particle_system::mass() {
    return particle_mass;
}

constexpr float particle_system::initial_volume() {
    return vol0;
}

#endif //ORCAMLSMPMDEMO_PARTICLE_SYSTEM_H
