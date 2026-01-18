//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"
#include <expected>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <cppitertools/enumerate.hpp>
#include <QGraphicsItem>
#include <opencv2/core.hpp>

Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *robot_scene)
{
    if (points.empty()) return {};

    // compute peaks in lidar data
    Peaks peaks;
    for (const auto &p : points | iter::sliding_window(2))
    {
        const auto &p1 = p[0]; const auto &p2 = p[1];
        if (const float diff = abs(p2.distance2d - p1.distance2d); diff > MIN_PEAK_THRESHOLD){
            const auto min = std::ranges::min_element(p, [](auto &pa, auto &pb){return pa.distance2d < pb.distance2d;});
            peaks.emplace_back(Eigen::Vector2f(min->x, min->y), min->phi);
        }
    }

    // non-maximum suppression of peaks: remove peaks closer than 500mm
    Peaks nms_peaks;
    for (const auto &[p, a] : peaks)
        if (const bool too_close = std::ranges::any_of(nms_peaks, [&p](const auto &p2) { return (p - std::get<0>(p2)).norm() < 500.f; }); not too_close)
            nms_peaks.emplace_back(p, a);
    peaks = nms_peaks;

    if (nms_peaks.empty()) return {};
    // compute doors in peaks data
    Doors doors;
    for (const auto &p : peaks | iter::combinations(2)) {
        const auto &[p0,a0] = p[0]; const auto &[p1,a1] = p[1];
        const float gap = (p1-p0).norm();
        if (gap < MAX_DOOR_THRESHOLD and gap > MIN_DOOR_THRESHOLD)
            doors.emplace_back(p0, a0, p1, a1);
    }
    draw_peaks(peaks, robot_scene);
    draw_doors(doors, robot_scene);
    doors_cache = doors;
    return doors;
}

// Method to use the Doors vector to filter out the LiDAR points that como from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);
    if(doors.empty()) return points;

    // for each door, check if the distance from the robot to each lidar point is smaller than the distance from the robot to the door
    RoboCompLidar3D::TPoints filtered;
    float offset = 0.2f;
    for(const auto &p : points)
    {
        bool erased = false;
        for(const auto &[d_index, d] : doors | iter::enumerate) {
            const float dist_to_door = d.center().norm();
            // Check if the angular range wraps around the -π/+π boundary
            const bool angle_wraps = d.p2_angle < d.p1_angle;
            // Determine if point is within the door's angular range
            bool point_in_angular_range;
            if (angle_wraps) {
                // If the range wraps around, point is in range if it's > p1_angle OR < p2_angle
                point_in_angular_range = (p.phi > (d.p1_angle - offset)) or (p.phi < (d.p2_angle + offset));
            } else {
                // Normal case: point is in range if it's between p1_angle and p2_angle
                point_in_angular_range = (p.phi > (d.p1_angle - offset)) and (p.phi < (d.p2_angle + offset));
            }

            // Filter out points that are through the door (in angular range and farther than door)
            if (point_in_angular_range and p.distance2d >= dist_to_door) {
                erased = true;
                break;
            }

            //if (d_index == doors.size()-1)
                //filtered.emplace_back(p);

            //debuggin
            /*if ( -M_PI_2/4 < p.phi and p.phi < M_PI_2/4){qInfo() << __FUNCTION__ << "Point angle: " << p.phi << " Door angles: " << d.p1_angle << ", " << d.p2_angle << " Point distance: " << p.distance2d << " Door distance: " << dist_to_door << door_index;}*/
        }
        if(!erased)
            filtered.emplace_back(p);
    }
    return filtered;
}

std::expected<Door, std::string> DoorDetector::get_current_door() const
{
    if (doors_cache.empty())
        return std::unexpected<std::string>{"No doors detected"};
    return doors_cache[0];
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