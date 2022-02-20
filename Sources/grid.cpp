#include "grid.h"

#define ASSERT_GRID_ACCESS_VALIDATION \
assert(0 <= _cell_idx[0] && _cell_idx[0] < GRID_RESOLUTION);\
assert(0 <= _cell_idx[1] && _cell_idx[1] < GRID_RESOLUTION);

#define ASSERT_GRID_ACCESS_VALIDATION1 \
assert(1 <= _cell_idx[0] && _cell_idx[0] < GRID_RESOLUTION - 2);\
assert(1 <= _cell_idx[1] && _cell_idx[1] < GRID_RESOLUTION - 2);

grid::NodalData<Eigen::Vector2f>& grid::momentum() const {
    assert(m_momentum);
    return (*m_momentum);
}

grid::NodalData<float>& grid::mass() const {
    assert(m_mass);
    return (*m_mass);
}

grid::NodalData<Eigen::Vector2f>& grid::velocity() const {
    assert(m_velocity);
    return (*m_velocity);
}

void grid::reset() {
    for(int i = 0; i < GRID_RESOLUTION; ++i) {
        for(int j = 0; j < GRID_RESOLUTION; ++j) {
            (*m_momentum)[i][j] = Eigen::Vector2f::Zero();
            (*m_mass)[i][j] = 0.0f;
            (*m_velocity)[i][j] = Eigen::Vector2f::Zero();
        }
    }
}

grid::grid() {
    m_momentum = std::make_unique<NodalData<Eigen::Vector2f>>();
    m_mass = std::make_unique<NodalData<float>>();
    m_velocity = std::make_unique<NodalData<Eigen::Vector2f>>();
}

void grid::apply_nodal_condition(float _time_delta) {
    static const Eigen::Vector2f left_limits = {0.0f, std::numeric_limits<float>::lowest()};
    static const Eigen::Vector2f right_limits = {0.0f, std::numeric_limits<float>::max()};
    static const Eigen::Vector2f bottom_limits = {std::numeric_limits<float>::lowest(), 0.0f};
    static const Eigen::Vector2f top_limits = {std::numeric_limits<float>::max(), 0.0f};
    for (int _x = 0; _x < GRID_RESOLUTION; ++_x) {
        for (int _y = 0; _y < GRID_RESOLUTION; ++_y) {
            if (_x < GRID_BOUNDARY_THICKNESS) {
                (*m_velocity)[_x][_y] = (*m_velocity)[_x][_y].cwiseMax(left_limits);
            }
            else if (GRID_RESOLUTION - GRID_BOUNDARY_THICKNESS < _x) {
                (*m_velocity)[_x][_y] = (*m_velocity)[_x][_y].cwiseMin(right_limits);
            }
            else if (_y < GRID_BOUNDARY_THICKNESS) {
                (*m_velocity)[_x][_y] = (*m_velocity)[_x][_y].cwiseMax(bottom_limits);
            }
            else if (GRID_RESOLUTION - GRID_BOUNDARY_THICKNESS < _y) {
                (*m_velocity)[_x][_y] = (*m_velocity)[_x][_y].cwiseMin(top_limits);
            }
            else {
                // apply body force
                (*m_velocity)[_x][_y] += _time_delta * Eigen::Vector2f{0.0f, -5.0f};
            }
        }
    }
}

void grid::update_velocity() {
    for(int x = 0; x < GRID_RESOLUTION; ++x) {
        for (int y = 0; y < GRID_RESOLUTION; ++y) {
            float m = (*m_mass)[x][y];
            if(m > 0.0f) {
                (*m_velocity)[x][y] = (*m_momentum)[x][y] / m;
            }
        }
    }
}


