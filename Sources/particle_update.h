#ifndef ORCAMLSMPMDEMO_PARTICLE_UPDATE_H
#define ORCAMLSMPMDEMO_PARTICLE_UPDATE_H

#include <Eigen/Dense>

using namespace Eigen;

/**
 * @brief Update MLS-MPM stress * volume. It doesn't return pure stress because of optimization.
 *
 * @param _deform_grad_begin First of deformation gradient
 * @param _lame_mu Lame constant mu
 * @param _lame_lambda Lame constant lambda
 * @param _stress_dest First destination of stress * volume updates
 * @param _len Length of inputs (i.e. number of particles)
 */
void update_stress(Matrix3f* _deform_grad_begin,
                   float _lame_mu,
                   float _lame_lambda, 
                   Matrix3f* _stress_dest,
                   size_t _len) {
    for (int loop = 0; loop < _len; ++loop, ++_deform_grad_begin, ++_stress_dest) {
        const Matrix3f& F = (*_deform_grad_begin);
        float J = (*_deform_grad_begin).determinant();

        // Polar SVD for co-rotated model
        Matrix3f F_rotate;
        {
            JacobiSVD<Matrix3f> svd(F,ComputeFullU | ComputeFullV);
            F_rotate = svd.matrixU() * svd.matrixV().transpose();
        }
        // Cauchy stress times volume
        Matrix3f stress =
                (2.0f * _lame_mu * (F - F_rotate) * (F).transpose()) + (_lame_lambda * (J - 1) * J * Matrix3f::Identity());
        (*_stress_dest) = stress;
    }
}
/**
 * @brief Do particle advection.
 *
 * @param _p_vel_begin First of velocities
 * @param _time_delta Time delta
 * @param _p_pos_dest First of previous particle positions and destination of the updated.
 * @param _len Length of inputs (i.e. number of particles)
 */
void update_position(Vector3f* _p_vel_begin,
                     float _time_delta, 
                     Vector3f * _p_pos_dest,
                     size_t _len) {
    for (int loop = 0; loop < _len; ++loop, ++_p_vel_begin, ++_p_pos_dest) {
        (*_p_pos_dest) += (*_p_vel_begin) * _time_delta;
    }
}
/**
 * @brief Update MLS-MPM deformation gradients (=F).
 * @param _p_affine_begin First of affine velocities
 * @param _time_delta Time delta
 * @param _deform_grad_dest First of previous deformation gradients and destination of the updated
 * @param _len Length of inputs (i.e. number of particles)
 */
void update_deformation_gradient(Matrix3f* _p_affine_begin,

                                 float _time_delta, Matrix3f* _deform_grad_dest,
                                 size_t _len) {
    for (int loop = 0; loop < _len; ++loop, ++_p_affine_begin, ++_deform_grad_dest) {
        (*_deform_grad_dest) *= (Matrix3f::Identity() + _time_delta * (*_p_affine_begin));
    }
}

#endif //ORCAMLSMPMDEMO_PARTICLE_UPDATE_H
