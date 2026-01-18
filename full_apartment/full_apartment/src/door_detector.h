//
// Created by pbustos on 11/11/25.
//

#ifndef DOORDETECTOR_H
#define DOORDETECTOR_H

#include <expected>

#include "common_types.h"
#include <Lidar3D.h>
#include <QGraphicsScene>

class DoorDetector
{
    public:
        DoorDetector() = default;
        ~DoorDetector() = default;

        Doors detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *robot_scene = nullptr);
        RoboCompLidar3D::TPoints filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene);
        [[nodiscard]] Doors doors() const { return doors_cache; }
        [[nodiscard]] std::expected<Door, std::string> get_current_door() const;

        void draw_peaks(Peaks &peaks, QGraphicsScene *scene);
        void draw_doors(Doors &doors, QGraphicsScene *scene);

    private:
        Doors doors_cache;
        const float PEAKS_THRESHOLD = 1000.f;
        const float MIN_PEAK_THRESHOLD = 200.f;
        const float MIN_DOOR_THRESHOLD = 800.f;
        const float MAX_DOOR_THRESHOLD = 1200.f;

};

#endif //DOORDETECTOR_H
