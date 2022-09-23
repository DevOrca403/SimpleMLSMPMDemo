#ifndef ORCAMLSMPMDEMO_PG_TRANSFER_H
#define ORCAMLSMPMDEMO_PG_TRANSFER_H

#include <Eigen/Dense>

#include "grid.h"
#include "particle_system.h"

using namespace Eigen;

void p2g_momentum_n_mass(Vector3f* _p_vel_begin,
                  std::array<Vector3f, 3>* _weights_begin,
                  Vector3i* _base_begin,
                  Vector3f* _fracture_begin,
                  grid& _g,
                  size_t _len) {
    float p_mass = particle_system::mass();
    for (int loop = 0; loop < _len; ++loop, ++_p_vel_begin, ++_weights_begin, ++_base_begin, ++_fracture_begin) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++)
                {
                    float weight = (*_weights_begin)[i][0] * (*_weights_begin)[j][1] * (*_weights_begin)[k][2];
                    _g.mass()[(*_base_begin)[0] + i][(*_base_begin)[1] + j][(*_base_begin)[2] + k] += weight * p_mass;
                    _g.momentum()[(*_base_begin)[0] + i][(*_base_begin)[1] + j][(*_base_begin)[2] + k] += (
                        (*_weights_begin)[i][0] * (*_weights_begin)[j][1] * (*_weights_begin)[k][2] * (p_mass * (*_p_vel_begin)));
                }
                
            }
        }
    }
}

void p2g_affine_n_force(Matrix3f* _p_stress_begin,
                        Matrix3f* _affine_vel_begin,
                        float _time_delta,
                        std::array<Vector3f, 3>* _weights_begin,
                        Vector3i* _base_begin,
                        Vector3f* _fracture_begin,
                        grid& _g,
                        size_t _len) {
    float p_mass = particle_system::mass();
    float D = 0.25f * _g.get_cell_space() * _g.get_cell_space();
    for (int loop = 0; loop < _len; ++loop, ++_p_stress_begin, ++_affine_vel_begin, ++_weights_begin, ++_base_begin, ++_fracture_begin) {
        Matrix3f stress_term = -(_time_delta * particle_system::initial_volume()) * (*_p_stress_begin) * (1.0f / D);
        Matrix3f pure_affine_term = (p_mass) * (*_affine_vel_begin);

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    Vector3f _r = (Vector3f(i, j, k) - (*_fracture_begin)) * _g.get_cell_space();
                    _g.momentum()[(*_base_begin)[0] + i][(*_base_begin)[1] + j][(*_base_begin)[2] + k] += (
                           (*_weights_begin)[i][0] * (*_weights_begin)[j][1] * (*_weights_begin)[k][2] * (stress_term + pure_affine_term) * _r);

                }
            }
        }
    }
}

void g2p_velocity(const grid& _g,
                  std::array<Vector3f, 3>* _weights_begin,
                  Vector3i* _base_begin,
                  Vector3f* _fracture_begin,
                  Vector3f* _p_vel_dest,
                  size_t _len) {
    for (int loop = 0; loop < _len; ++loop, ++_weights_begin, ++_base_begin, ++_fracture_begin, ++_p_vel_dest) {
        (*_p_vel_dest) = Vector3f::Zero();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++)
                {
                    float weight = (*_weights_begin)[i][0] * (*_weights_begin)[j][1] * (*_weights_begin)[k][2];
                    (*_p_vel_dest) += weight * _g.velocity()[(*_base_begin)[0] + i][(*_base_begin)[1] + j][(*_base_begin)[2] + k];
                }
            }
        }
    }
}

void g2p_affine_velocity(const grid& _g,
                         std::array<Vector3f, 3>* _weights_begin,
                         Vector3i* _base_begin,
                         Vector3f* _fracture_begin,
                         Matrix3f* _p_affine_dest,
                         size_t _len) {
    for (int loop = 0; loop < _len; ++loop, ++_weights_begin, ++_base_begin, ++_fracture_begin, ++_p_affine_dest) {
        (*_p_affine_dest) = Matrix3f::Zero();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    Vector3f _r = (Vector3f(i, j) - (*_fracture_begin)) * _g.get_cell_space();
                    Vector3f grid_v = _g.velocity()[(*_base_begin)[0] + i][(*_base_begin)[1] + j][(*_base_begin)[2] + k];
                    float weight = (*_weights_begin)[i][0] * (*_weights_begin)[j][1] * (*_weights_begin)[k][2];

                    float D_inv = (3.0f / (_g.get_cell_space() * _g.get_cell_space()));
                    (*_p_affine_dest) += D_inv * weight * grid_v * _r.transpose();
                }
            }
        }
    }
}
#endif //ORCAMLSMPMDEMO_PG_TRANSFER_H
