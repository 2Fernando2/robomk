//
// Created by pbustos on 11/11/25.
//

#ifndef DOORDETECTOR_H
#define DOORDETECTOR_H

#include "common_types.h"
#include <Lidar3D.h>
#include <QGraphicsScene>

class DoorDetector
{
    public:
        DoorDetector() = default;
        ~DoorDetector() = default;

        Doors detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene = nullptr);
        std::tuple<RoboCompLidar3D::TPoints, Doors> filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene);
        [[nodiscard]] Doors doors() const { return doors_cache; };

        void draw_peaks(Peaks &peaks, QGraphicsScene *scene);
        void draw_doors(Doors &doors, QGraphicsScene *scene);

    private:
        Doors doors_cache;
        const float PEAKS_THRESHOLD = 1000.f;
        const float MIN_PEAK_THRESHOLD = 300.f;
        const float MIN_DOOR_THRESHOLD = 800.f;
        const float MAX_DOOR_THRESHOLD = 1200.f;

};

#endif //DOORDETECTOR_H
