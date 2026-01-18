//
// Created by pbustos on 21/11/25.
//

#ifndef ROBUST_ROOM_CENTER_ESTIMATOR_H
#define ROBUST_ROOM_CENTER_ESTIMATOR_H

#include <Eigen/Dense>
#include <vector>
#include <optional>
#include <Lidar3D.h>

namespace rc
{
    /**
     * @brief Robust estimator to find the geometric center of a room from Lidar data.
     * Uses Convex Hull and Orient Bounding Box (OBB) techniques to handle outliners and furniture.
     */
    class PointcloudCenterEstimator
    {
    public:
        /**
         *  @brief Configuration parameters for the estimator.
         */
        struct Config
        {
            int num_sectors = 360;               ///< Angular sectors for initial filtering (10° sectors).
            double max_range = 30000.0;          ///< Max Lidar range (meters).
            double min_range = 200;              ///< Min range to ignore robot hardware.
            double outlier_std_threshold = 2500; ///< Threshold for statistical outlier removal.
            double obb_fit_tolerance = 100;      ///< Tolerance for fitting the Bounding box.
            size_t min_valid_points = 20;        ///< Minimum points required to attempt estimation.

            Config(){}
        };

        using Point2D = Eigen::Vector2d;

        /**
         * @brief Constructor.
         * @param config Configuration struct (nominal).
         */
        explicit PointcloudCenterEstimator(const Config &config = Config{});

        /**
         * @brief Estimates the center from a vector of Eigen::Vector2d.
         * @param points Input 2D points.
         * @return Optional containing the center point, or nullopt if estimation failed.
         */
        std::optional<Point2D> estimate(const std::vector<Point2D>& points);

        /**
         * @brief Overload to accept RoboComp Lidar type.
         * @param points Input points in RoboComp format.
         * @return Optional containing the center point.
         */
        std::optional<Point2D> estimate(const RoboCompLidar3D::TPoints& points);

    private:
        Config config_;

        /** @brief Filters points by range and density.  */
        std::vector<Point2D> filterPoints(const std::vector<Point2D>& points);

        /** @brief Extracts points likely belonging to walls (boundary). */
        std::vector<Point2D> extractBoundaryPoints(const std::vector<Point2D>& points);

        /** @brief Checks if a point is a local distance maximum (candidate corner/wall). */
        bool isLocalMaximum(const Point2D& candidate, const std::vector<Point2D>& neighbors, double threshold);

        /** @brief Removes statistical outliers based on mean distance to neighbors. */
        std::vector<Point2D> removeStatisticalOutliers(const std::vector<Point2D>& points);

        /** @brief Calculates simple centroid (mean) of points. */
        Point2D calculateRobustCentroid(const std::vector<Point2D>& points);

        /** @brief Computes the Convex Hull of the point set. */
        std::vector<Point2D> computeConvexHull(const std::vector<Point2D>& points);

        struct OBB
        {
            Point2D center{};
            double width = 0.0;
            double height = 0.0;
            double rotation = 0.0; // radians
        };

        /** @brief Computes the Minimum Area Oriented Bounding Box for the points. */
        OBB computeOBB(const std::vector<Point2D>& hull);
    };


};
#endif // ROBUST_ROOM_CENTER_ESTIMATOR_H