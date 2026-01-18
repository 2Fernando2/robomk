//
// Created by pbustos on 27/10/25.
//

#ifndef RANSACLINEDETECTOR_H
#define RANSACLINEDETECTOR_H

#include <vector>
#include <Eigen/Geometry>
#include "common_types.h"

namespace rc {

/**  */
class RansacLineDetector
{
    public:
        struct Params
        {
            double distance_threshold = 50.0;   ///< Max distance (mm) from line to consider a point an inlier.
            int min_points_per_line = 30;       ///< Min inliers required to accept a line.
            int max_iterations = 100;           ///< Max RANSAC iterations per attempt.
            int max_lines = 10;                 ///< Max number of lines to extract.
            double min_line_length = 300.0;     ///< Min length (mm) of accepted lines.
            double max_line_separation = 200.0; ///< Max gap allowed to merge collinear segments.
            // Constructor to initialize default values
            Params() : distance_threshold(50.0), min_points_per_line(30), max_iterations(100),
                  max_lines(10), min_line_length(300.0) {}
        };

        /**
         * @brief Main method to extract line segments from points.
         * Iteratively finds the best line model, removes inliers, and repeats.
         * @param points Input 2D points.
         * @param params RANSAC parameters.
         * @return Vector of detected LineSegment objects.
         */
        static std::vector<LineSegment> detect_lines(const std::vector<Eigen::Vector2d>& points, const Params& params = Params());
};

} // rc

#endif //RANSACLINEDETECTOR_H
