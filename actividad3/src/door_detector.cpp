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
        auto difference = abs((p2.distance2d - p1.distance2d));
        auto closest = p1.distance2d < p2.distance2d ? p1 : p2;
        if (difference > 1000.f) peaks.push_back(closest);
    }

    /*//no-maximum suppression filter
    for (auto i = 0; i < peaks.size(); i++)
    {
        for (auto j = i + 1; j < peaks.size(); j++)
        {
            auto dist = std::sqrt(std::pow((peaks[j].x - peaks[i].x), 2) + std::pow((peaks[j].y - peaks[i].y), 2));
            if (dist < MIN_PEAK_THRESHOLD) peaks.erase(peaks.begin() + j);
        }
    }*/

    draw_peaks(peaks, scene);

    if (peaks.empty()) return {};

    // compute doors in peaks data
    Doors doors;
    for (const auto &c : iter::combinations(peaks, 2))
    {
        const auto &p1 = c[0];
        const auto &p2 = c[1];
        auto dist = std::sqrt(std::pow((p2.x - p1.x), 2) + std::pow((p2.y - p1.y), 2));
        if (MIN_DOOR_THRESHOLD < dist and dist < MAX_DOOR_THRESHOLD)
            doors.push_back(Door(Eigen::Vector2f(p1.x, p1.y), p1.phi,Eigen::Vector2f(p2.x, p2.y), p2.phi));
    }
    draw_doors(doors, scene);

    return doors;
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



void DoorDetector::draw_peaks(RoboCompLidar3D::TPoints &peaks, QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

    // remove all items drawn in the previous iteration
    for(auto i: items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    for (const auto& peak : peaks)
    {
        auto item = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::red), QBrush(QColorConstants::Svg::red));
        item->setPos(peak.x, peak.y);
        items.push_back(item);
    }
}

void DoorDetector::draw_doors(Doors &doors, QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

    // remove all items drawn in the previous iteration
    for(auto i: items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();


    for (const auto& door : doors)
    {
        auto item = scene->addEllipse(-100, -100, 200, 200, QColor(QColorConstants::Svg::grey), QBrush(QColor(QColorConstants::Svg::grey)));
        item->setPos(door.p1.x(), door.p1.y());
        items.push_back(item);

        item = scene->addEllipse(-100, -100, 200, 200, QColor(QColorConstants::Svg::grey), QBrush(QColor(QColorConstants::Svg::grey)));
        item->setPos(door.p2.x(), door.p2.y());
        items.push_back(item);

        auto line = scene->addLine(door.p1.x(), door.p1.y(), door.p2.x(), door.p2.y(),QPen(QColor(QColorConstants::Svg::grey), 30));
        items.push_back(line);
    }
}