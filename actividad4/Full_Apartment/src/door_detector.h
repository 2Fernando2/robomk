//
// Created by pbustos on 11/11/25.
//

#ifndef DOORDETECTOR_H
#define DOORDETECTOR_H

#include <expected>

#include "common_types.h"
#include <Lidar3D.h>
#include <QGraphicsScene>

/**
 * @brief Class responsible for processing Lidar data to find door apertures.
 * Detects discontinuities (peaks) in the laser scan and pairs them to identify doors.
 */
class DoorDetector
{
    public:
        DoorDetector() = default;
        ~DoorDetector() = default;

        /**
         * @brief Main method to detect doors from a raw Lidar scan.
         * @param points Raw point cloud.
         * @param robot_scene Optional graphics scene to draw debug info.
         * @return Vector of detected Door objects.
         */
        Doors detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *robot_scene = nullptr);

        /**
         * @brief Pre-processes Lidar points (filtering noise via distance) before detection.
         * @param points Input points.
         * @param scene Optional scene for debug drawing.
         * @return Filtered point cloud.
         */
        RoboCompLidar3D::TPoints filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene);

        /**
         * @brief Returns the last computed set of doors from cache.
         */
        [[nodiscard]] Doors doors() const { return doors_cache; }

        /**
        * @brief Attempts to retrieve the single "current" target door if available.
        * @return Expected Door object or error string.
        */
        [[nodiscard]] std::expected<Door, std::string> get_current_door() const;

        /** @brief Helper to draw detected peaks (door jamb candidates) on a scene. */
        void draw_peaks(Peaks &peaks, QGraphicsScene *scene);

        /** @brief Helper to draw detected door objects on a scene. */
        void draw_doors(Doors &doors, QGraphicsScene *scene);

    private:
        Doors doors_cache;
        // Threshold constants
        const float PEAKS_THRESHOLD = 1000.f;       ///< Min depth jump to consider a peak.
        const float MIN_PEAK_THRESHOLD = 200.f;     ///< Min obstacle width.
        const float MIN_DOOR_THRESHOLD = 800.f;     ///< Min width for a door.
        const float MAX_DOOR_THRESHOLD = 1200.f;    ///< Max width for a door.

};

#endif //DOORDETECTOR_H
