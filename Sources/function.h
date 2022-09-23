#ifndef ORCAMLSMPMDEMO_FUNCTION_H
#define ORCAMLSMPMDEMO_FUNCTION_H

#include <array>

#include <Eigen/Dense>

using namespace Eigen;
/**
 * @brief Calculate neighbor bases which mean local origin for b-spline.
 *
 * @param _pos_begin First of particle positions
 * @param _cell_space Cell space
 * @param _base_dest First destination of neighbor bases
 * @param _len Length of inputs (i.e. number of particles)
 */
void get_neighbor_base(Vector3f* _pos_begin,
                       float _cell_space,
                       Vector3i* _base_dest,
                       size_t _len) {
    float inv_cell_space = 1.0f / _cell_space;
    for(int loop = 0; loop < _len; ++loop, ++_pos_begin, ++_base_dest) {
        Vector3i base_coord = ((*_pos_begin) * inv_cell_space - Vector3f::Constant(0.5f)).cast<int>();
        (*_base_dest) = base_coord;
    }
}
/**
 * @brief Calculate neighbor fractures which mean distances between local origin and particle position.
 *
 * @param _pos_begin First of particle positions
 * @param _base_begin First of local origins
 * @param _cell_space Cell space
 * @param _fracture_dest First destination of neighbor fractures
 * @param _len Length of inputs (i.e. number of particles)
 */
void get_neighbor_fracture(Vector3f* _pos_begin,
                           Vector3i* _base_begin,
                           float _cell_space,
                           Vector3f* _fracture_dest,
                           size_t _len) {
    float inv_cell_space = 1.0f / _cell_space;
    for (int loop = 0; loop < _len; ++loop, ++_pos_begin, ++_base_begin, ++_fracture_dest) {
        Vector3f fracture_coord = ((*_pos_begin)) * inv_cell_space - (*_base_begin).cast<float>();
        (*_fracture_dest) = fracture_coord;
    }
}
/**
 * @brief Calculate B-spline weights for PG transfer.
 *
 * @param _fracture_begin First of distances between local origin and particle position(= neighbor fracture).
 * @param _weight_dest First destination of weights. It contains 3 * 2 dimension vectors.
 * @param _len Length of inputs (i.e. number of particles)
 */
void get_weight(Vector3f* _fracture_begin,
                std::array<Vector3f, 3>* _weight_dest,
                size_t _len) {
    for (int loop = 0; loop < _len; ++loop, ++_fracture_begin, ++_weight_dest) {
        (*_weight_dest) = {
                0.5f * (Vector3f::Constant(1.5f) - (*_fracture_begin)).array().square().matrix(),
                Vector3f::Constant(0.75f) - ((*_fracture_begin) - Vector3f::Constant(1.0f)).array().square().matrix(),
                0.5f * ((*_fracture_begin) - Vector3f::Constant(0.5)).array().square().matrix()
        };
    }
}

#endif //ORCAMLSMPMDEMO_FUNCTION_H
