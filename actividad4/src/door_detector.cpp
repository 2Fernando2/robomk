//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <cppitertools/sliding_window.hpp>
#include <QGraphicsItem>
#include <opencv2/core.hpp>


Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    // compute peaks in lidar data
    Peaks peaks;
    for (const auto &p : points | iter::sliding_window(2))
    {
        const auto &p1 = p[0];
        const auto &p2 = p[1];
        auto difference = abs((p2.distance2d - p1.distance2d));
        auto closest = p1.distance2d < p2.distance2d ? p1 : p2;
        if (difference > 1000.f) peaks.push_back(std::make_tuple(Eigen::Vector2f(closest.x,closest.y), closest.phi));
    }
    if (peaks.empty()) return {};

    // non-maximum suppression of peaks: remove peaks closer than 500mm
    Peaks nms_peaks;
    for (const auto &[p, a] : peaks)
        if (const bool too_close = std::ranges::any_of(nms_peaks, [&p](const auto &p2) { return (p - std::get<0>(p2)).norm() < 500.f; }); not too_close)
            nms_peaks.emplace_back(p, a);
    peaks = nms_peaks;

    if (nms_peaks.empty()) return {};
    draw_peaks(nms_peaks, scene);

    // compute doors in peaks data
    Doors doors;
    for (const auto &c : iter::combinations(nms_peaks, 2))
    {
        const auto &p1 = c[0];
        const auto &p2 = c[1];
        auto dist = std::sqrt(std::pow(std::get<0>(p2).x()-std::get<0>(p1).x(), 2) + std::pow(std::get<0>(p2).y()-std::get<0>(p1).y(), 2));
        if (MIN_DOOR_THRESHOLD < dist && dist < MAX_DOOR_THRESHOLD)
        {
            doors.push_back(Door(std::get<0>(p1), std::get<1>(p1), std::get<0>(p2), std::get<1>(p2)));  //caso normal
            // if (std::get<1>(p1) > std::get<1>(p2))
            // {
            //     doors.push_back(Door(std::get<0>(p2), std::get<1>(p2), std::get<0>(p1), std::get<1>(p1)));  //caso especial
            //     qInfo() << "Caso especial";
            // }
            // else
            // {
            //     doors.push_back(Door(std::get<0>(p1), std::get<1>(p1), std::get<0>(p2), std::get<1>(p2)));  //caso normal
            //     // qInfo() << "Caso normal";
            // }
        }
    }
    draw_doors(doors, scene);

    // for (const auto door : doors)
    // {
    //     qInfo() << door.p1_angle << door.p2_angle;
    // }
    // qInfo() << "=============================";
    return doors;
}

// Method to use the Doors vector to filter out the LiDAR points that como from a room outside the current one
std::tuple<RoboCompLidar3D::TPoints, Doors> DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);
    if(doors.empty()) return {points, Doors{}};

    // for each door, check if the distance from the robot to each lidar point is smaller than the distance from the robot to the door
    RoboCompLidar3D::TPoints filtered;
    float offset = 0.0;
    int door_index = 0;
    bool erased = false;
    for(const auto &p : points)
    {
        erased = false;
        door_index = 0;
        for(const auto &d : doors)
        {
            if (erased)
                continue;


            const float dist_to_door = d.center().norm();
            // Check if the angular range wraps around the -π/+π boundary
            const bool angle_wraps = d.p2_angle < d.p1_angle;
            // Determine if point is within the door's angular range
            bool point_in_angular_range;
            if (angle_wraps)
            {
                // Caso especial
                // If the range wraps around, point is in range if it's > p1_angle OR < p2_angle
                point_in_angular_range = (p.phi > d.p1_angle + offset) or (p.phi < d.p2_angle - offset);
            }
            else
            {
                // Normal case: point is in range if it's between p1_angle and p2_angle
                point_in_angular_range = (p.phi > d.p1_angle - offset) and (p.phi < d.p2_angle + offset);
            }

            // Filter out points that are through the door (in angular range and farther than door)
            if(point_in_angular_range and p.distance2d >= dist_to_door)
            {
                // qInfo() << p.phi << d.p1_angle << d.p2_angle << "||" << p.distance2d << ">=" << dist_to_door;
                // qInfo() << "==========================================================";
                erased = true;
                continue;
            }

            //debuggin
            if ( -M_PI_2/4 < p.phi and p.phi < M_PI_2/4)
            {
                qInfo() << __FUNCTION__ << "Point angle: " << p.phi << " Door angles: " << d.p1_angle << ", " << d.p2_angle << " Point distance: " << p.distance2d << " Door distance: " << dist_to_door << door_index;
            }
            filtered.emplace_back(p);
            door_index++;
        }
    }
    return {filtered, doors};
}



void DoorDetector::draw_peaks(Peaks &peaks, QGraphicsScene *scene)
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
        item->setPos(std::get<0>(peak).x(), std::get<0>(peak).y());
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