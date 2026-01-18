//
// Created by robolab on 12/5/24.
//

#ifndef BETA_ROBOTICA_CLASS_PRIVATE_HUNGARIAN_H
#define BETA_ROBOTICA_CLASS_PRIVATE_HUNGARIAN_H

#include <Eigen/Geometry>
#include "munkres.hpp"
#include "common_types.h"

namespace rc
{
    /**
     * @brief Wrapper class for the Hungarian (Munkres) Algorithm.
     * Solves the assignment problem to match observed features with map features.
     */
    class Hungarian
    {
        public:
            /**
            * @brief Hungarian function to work with nominal corners.
            * Computes a cost matrix based on Euclidean distance and finds the optimal assignment that
            *  minimizes the total cost.
            * @param measurement_corners Corners detected by the robot sensors.
            * @param nominal_corners Corners from the map.
            * @param max_corner_diff Maximum allowed distance to consider a match valid (gating).
            * @return Match Vector of tuples {Measured Corner, Nominal Corner, Error Distance}.
            */
            Match match(const Corners &measurement_corners, const Corners &nominal_corners, double max_corner_diff = std::numeric_limits<double>::max());

            // aux methods
            /** @brief Computes Euclidean distance between two QPointF. */
            double euclidean_distance(const QPointF &p1, const QPointF &p2);

            /** @brief Computes Euclidean distance between the coordinates of two Corner objects. */
            double euclidean_distance(const Corner &c1, const Corner &c2);
    };
} // rc

#endif //BETA_ROBOTICA_CLASS_PRIVATE_HUNGARIAN_H
