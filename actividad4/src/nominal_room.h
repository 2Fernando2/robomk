#pragma once
#include <QPointF>
#include <QRectF>
#include <Eigen/Dense>
#include <vector>
#include "src/common_types.h"
#include <cppitertools/sliding_window.hpp>

  struct NominalRoom
        {
            float width; //  mm
            float length;
            Doors doors;
            explicit NominalRoom(const float width_=10000.f, const float length_=5000.f) :
                width(width_), length(length_)
            {};
            void set_doors(const Doors &doors_) {doors = doors_;}
            [[nodiscard]] Doors get_doors() const {return doors;}
            [[nodiscard]] Walls get_walls() const {
                Walls walls;
                // Compute walls by pair of corners
                auto room_corners = corners();
                room_corners.push_back(room_corners.front()); // duplicate first element so can be paired with last
                int wall_index = 0;
                for (const auto &corner_pair: room_corners | iter::sliding_window(2)){
                    const auto &p1_ = std::get<0>(corner_pair[0]);
                    const auto &p2_ = std::get<0>(corner_pair[1]);
                    Eigen::Vector2f p1(p1_.x(), p1_.y());
                    Eigen::Vector2f p2(p2_.x(), p2_.y());
                    auto line = Eigen::ParametrizedLine<float, 2>::Through(p1, p2);
                    walls.push_back(std::make_tuple(line, wall_index++, corner_pair[0], corner_pair[1]));
                }
                return walls;
            }
            [[nodiscard]] Wall get_closest_wall_to_point(const Eigen::Vector2f &p) const {
                const auto walls = get_walls();
                auto closest_wall = std::ranges::min_element(walls,
                        [&p](const Wall &w1, const Wall &w2){
                            return std::get<0>(w1).distance(p) < std::get<0>(w2).distance(p);
                        });
                return *closest_wall;
            }
            [[nodiscard]] Eigen::Vector2f get_projection_of_point_on_closest_wall(const Eigen::Vector2f &p) const {
                const Wall w = get_closest_wall_to_point(p);
                    return std::get<0>(w).projection(p);
            }
            [[nodiscard]] Corners corners() const
            {
                // compute corners from width and length
                return {
                    {QPointF{-width/2.f, -length/2.f}, 0.f, 0.f},
                    {QPointF{width/2.f, -length/2.f}, 0.f, 0.f},
                    {QPointF{width/2.f, length/2.f}, 0.f, 0.f},
                    {QPointF{-width/2.f, length/2.f}, 0.f, 0.f}
                };
            }
            [[nodiscard]] QRectF rect() const
            {
                return QRectF{-width/2.f, -length/2.f, width, length};
            }
            [[nodiscard]] Corners transform_corners_to(const Eigen::Affine2f &transform) const  // for room to robot pass the inverse of robot_pose
            {
                Corners transformed_corners;
                for(const auto &[p, _, __] : corners())
                {
                    auto ep = Eigen::Vector2d{p.x(), p.y()};
                    Eigen::Vector2d tp = transform.cast<double>() * ep;
                    transformed_corners.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, 0.f, 0.f);
                }
                return transformed_corners;
            }
        };