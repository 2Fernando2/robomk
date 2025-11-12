//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <cppitertools/sliding_window.hpp>
#include <QGraphicsItem>


Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    // compute peaks in lidar data
    RoboCompLidar3D::TPoints peaks;
    for (const auto &p : points | iter::sliding_window(2))
    {
        const auto &p1 = p[0];
        const auto &p2 = p[1];
        auto den = (p2.phi - p1.phi);
        den = (den == 0) ? 1 : den;
        auto derived = (p2.distance2d - p1.distance2d) / den;
        if (derived > 1000.f) peaks.push_back(p1);
    }

    // compute doors in peaks data
    Doors doors;
    for (const auto &c : iter::combinations(peaks, 2))
    {
        const auto &p1 = c[0];
        const auto &p2 = c[1];
        auto dist = std::sqrt(std::pow((p2.x - p1.x), 2) + std::pow((p2.y - p1.y), 2));
        // if (MIN_DOOR_THRESHOLD < dist && dist < MAX_DOOR_THRESHOLD) doors.push_back()
    }

    // filter lidar points
    

    return {};
}

// Method to use the Doors vector to filter out the LiDAR points that como from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);
    if(doors.empty()) return points;

    // for each door, check if the distance from the robot to each lidar point is smaller than the distance from the robot to the door
    RoboCompLidar3D::TPoints filtered;
    for(const auto &d : doors)
    {
        const float dist_to_door = d.center().norm();
        // Check if the angular range wraps around the -π/+π boundary
        const bool angle_wraps = d.p2_angle < d.p1_angle;
        for(const auto &p : points)
        {
            // Determine if point is within the door's angular range
            bool point_in_angular_range;
            if (angle_wraps)
            {
                // If the range wraps around, point is in range if it's > p1_angle OR < p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) or (p.phi < d.p2_angle);
            }
            else
            {
                // Normal case: point is in range if it's between p1_angle and p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) and (p.phi < d.p2_angle);
            }

            // Filter out points that are through the door (in angular range and farther than door)
            if(point_in_angular_range and p.distance2d >= dist_to_door)
                continue;

            //qInfo() << __FUNCTION__ << "Point angle: " << p.phi << " Door angles: " << d.p1_angle << ", " << d.p2_angle << " Point distance: " << p.distance2d << " Door distance: " << dist_to_door;
            filtered.emplace_back(p);
        }
    }
    return filtered;
}
