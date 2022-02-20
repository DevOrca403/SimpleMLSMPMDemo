#ifndef ORCAMPMDEMO_GRID_H
#define ORCAMPMDEMO_GRID_H

#define GRID_RESOLUTION 100
#define GRID_BOUNDARY_THICKNESS 5

#include <array>
#include <memory>
#include <cassert>
#include <limits>

#include <Eigen/Dense>

class grid {
public:
    template<class DataType>
    using NodalData = std::array<std::array<DataType, GRID_RESOLUTION>, GRID_RESOLUTION>;
    template<class DataType>
    using UPtrNodalData = std::unique_ptr<NodalData<DataType>>;

    [[nodiscard]] NodalData<Eigen::Vector2f>& momentum() const;

    [[nodiscard]] NodalData<float>& mass() const;

    [[nodiscard]] NodalData <Eigen::Vector2f>& velocity() const;

    void reset();

    void apply_nodal_condition(float _time_delta);

    void update_velocity();

    grid();

    [[nodiscard]] static constexpr int get_resolution() ;
    [[nodiscard]] static constexpr float get_cell_space() ;
private:
    UPtrNodalData<Eigen::Vector2f> m_momentum;
    UPtrNodalData<float> m_mass;
    UPtrNodalData<Eigen::Vector2f> m_velocity;
};

constexpr int grid::get_resolution() {
    return GRID_RESOLUTION;
}

constexpr float grid::get_cell_space() {
    return 1.0f/GRID_RESOLUTION;
}

#endif //ORCAMPMDEMO_GRID_H
