#include "grid.h"

#define ASSERT_GRID_ACCESS_VALIDATION \
assert(0 <= _cell_idx[0] && _cell_idx[0] < GRID_RESOLUTION);\
assert(0 <= _cell_idx[1] && _cell_idx[1] < GRID_RESOLUTION);

#define ASSERT_GRID_ACCESS_VALIDATION1 \
assert(1 <= _cell_idx[0] && _cell_idx[0] < GRID_RESOLUTION - 2);\
assert(1 <= _cell_idx[1] && _cell_idx[1] < GRID_RESOLUTION - 2);

grid::NodalData<Eigen::Vector3f>& grid::momentum() const {
    assert(m_momentum);
    return (*m_momentum);
}

grid::NodalData<float>& grid::mass() const {
    assert(m_mass);
    return (*m_mass);
}

grid::NodalData<Eigen::Vector3f>& grid::velocity() const {
    assert(m_velocity);
    return (*m_velocity);
}

void grid::reset() {
    for(int i = 0; i < GRID_RESOLUTION; ++i) {
        for(int j = 0; j < GRID_RESOLUTION; ++j) {
            for (int k = 0; k < GRID_RESOLUTION; ++k)
            {
                (*m_momentum)[i][j][k] = Eigen::Vector3f::Zero();
                (*m_mass)[i][j][k] = 0.0f;
                (*m_velocity)[i][j][k] = Eigen::Vector3f::Zero();
            }
        }
    }
}

grid::grid() {
    m_momentum = std::make_unique<NodalData<Eigen::Vector3f>>();
    m_mass = std::make_unique<NodalData<float>>();
    m_velocity = std::make_unique<NodalData<Eigen::Vector3f>>();
}

void grid::apply_nodal_condition(float _time_delta) {
    static constexpr float lowest = std::numeric_limits<float>::lowest();
    static constexpr float max = std::numeric_limits<float>::max();    
    // x axis
    static const Eigen::Vector3f left_limits   = { 0.0f  , lowest, lowest };
    static const Eigen::Vector3f right_limits  = { 0.0f  , max   , max    };

    // y axis
    static const Eigen::Vector3f front_limits  = { lowest, 0.0f  , lowest };
    static const Eigen::Vector3f back_limits   = { max   , 0.0f  , max    };

    // z axis
    static const Eigen::Vector3f bottom_limits = { lowest, lowest, 0.0f   };
    static const Eigen::Vector3f top_limits    = { max   , max   , 0.0f   };

    for (int _x = 0; _x < GRID_RESOLUTION; ++_x) {
        for (int _y = 0; _y < GRID_RESOLUTION; ++_y) {
            for (int _z = 0; _z < GRID_RESOLUTION; ++_z)
            {
                // x axis
                if (_x < GRID_BOUNDARY_THICKNESS)
                {
                    (*m_velocity)[_x][_y][_z] = (*m_velocity)[_x][_y][_z].cwiseMax(left_limits);
                }
                else if (GRID_RESOLUTION - GRID_BOUNDARY_THICKNESS < _x)
                {
                    (*m_velocity)[_x][_y][_z] = (*m_velocity)[_x][_y][_z].cwiseMin(right_limits);
                }

                // z axis 
                else if (_y < GRID_BOUNDARY_THICKNESS)
                {
                    (*m_velocity)[_x][_y][_z] = (*m_velocity)[_x][_y][_z].cwiseMax(front_limits);
                }
                else if (GRID_RESOLUTION - GRID_BOUNDARY_THICKNESS < _y)
                {
                    (*m_velocity)[_x][_y][_z] = (*m_velocity)[_x][_y][_z].cwiseMin(back_limits);
                }

                // z axis 
                else if (_z < GRID_BOUNDARY_THICKNESS)
                {
                    (*m_velocity)[_x][_y][_z] = (*m_velocity)[_x][_y][_z].cwiseMax(bottom_limits);
                }
                else if (GRID_RESOLUTION - GRID_BOUNDARY_THICKNESS < _z)
                {
                    (*m_velocity)[_x][_y][_z] = (*m_velocity)[_x][_y][_z].cwiseMin(top_limits);
                }

                else
                {
                    // apply body force
                    (*m_velocity)[_x][_y][_z] += _time_delta * Eigen::Vector3f{ 0.0f, 0.0f, -0.5f };
                }
            }
        }
    }
}

void grid::update_velocity() {
    for(int x = 0; x < GRID_RESOLUTION; ++x) {
        for (int y = 0; y < GRID_RESOLUTION; ++y) {
            for (int z = 0; z < GRID_RESOLUTION; ++z)
            {
                float m = (*m_mass)[x][y][z];
                if(m > 0.0f) {
                    (*m_velocity)[x][y][z] = (*m_momentum)[x][y][z] / m;
                }
            }
        }
    }
}


