//
// Created by pbustos on 9/12/24.
//

#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <vector>
#include <QPointF>
#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <QLineF>
#include <Eigen/src/Geometry/ParametrizedLine.h>


/**
 * @brief Represents a 2D line segment detected detected from sensor data.
 */
struct LineSegment
{
    Eigen::Vector2d start;            ///< Start point of the segment.
    Eigen::Vector2d end;              ///< End point of the segment.
    Eigen::Vector2d direction;        ///< Normalized direction vector of the line.
    std::vector<int> inlier_indices;  ///< Indices of Lidar points belonging to this line.
    int num_inliers;                  ///< Count of inliers.
    double score;                     ///< Quality score of the line fit.

    /** @brief Converts the segment to a Qt Line Object. */
    [[nodiscard]] QLineF toQLineF() const { return QLineF{QPointF{start.x(), start.y()}, QPointF{end.x(), end.y()}};}

    /** @brief Converts the segment to an Eigen ParametrizedLine. */
    [[nodiscard]] Eigen::ParametrizedLine<double, 2> toEigenLine() const { return Eigen::ParametrizedLine<double, 2>(start, direction);};

    /** @brief Calculates the midpoint of the segment. */
    [[nodiscard]] Eigen::Vector2d midpoint() const { return (start + end) / 2.0; }

    /** @brief Calculates the Euclidean length of the segment. */
    [[nodiscard]] double length() const { return (end - start).norm(); }

    /**
     * @brief Computes the general line equation coefficients (Ax + By + C = 0).
     * @return Vector3d containing {A, B, C}.
     */
    [[nodiscard]] Eigen::Vector3d to_general_form() const
    {
        const auto line = toQLineF();
        // A = y1 - y2
        auto A = line.p1().y() - line.p2().y();
        // B = x2 - x1
        auto B = line.p2().x() - line.p1().x();
        // C = -Ax1 - By1
        // Using p1 to solve for C. You could also use p2.
        auto C = -A * line.p1().x() - B * line.p1().y();
        return{ A, B, C };
    };
};
using Lines = std::vector<LineSegment>;
using Par_lines = std::vector<std::pair<LineSegment, LineSegment>>;
using Corner = std::tuple<QPointF, double, long>; // corner point, angle wrt coordinate axes, timestamp
using Corners =  std::vector<Corner>;
using All_Corners = std::vector<std::tuple<QPointF, QPointF, QPointF, QPointF>>;
using Features = std::tuple<Lines, Par_lines, Corners, All_Corners>;
using Center = std::pair<QPointF, int>;  // center of a polygon and number of votes
using Match = std::vector<std::tuple<Corner, Corner, double>>;  //  measurement - nominal - error Both must be in the same reference system
using Peaks = std::vector<std::tuple<Eigen::Vector2f, float>>; // 2D points representing peaks with angle wrt robot frame

/**
* @brief Represents a door aperture consisting of two jamb points (peaks).
* Stores both local and global coordinates and topological connectivity info.
*/
struct Door
{
    Eigen::Vector2f p1;                     ///< First door jamb (local coords).
    float p1_angle;                         ///< Angle of p1 relative to robot center.
    Eigen::Vector2f p2;                     ///< Second door jamb (local coords).
    float p2_angle;                         ///< Angle of p2 relative to robot center.
    Eigen::Vector2f p1_global, p2_global;   ///< Doors jambs in global (room) coords.
    bool visited = false;                   ///< Flag indicating if the robot has crossed this door.
    int connects_to_room = -1;              ///< index of the room this door connects to
    int connects_to_door = -1;              ///< index of the door in the connected room
    Eigen::Affine2f door_pose_in_room;      ///< door pose in the room frame Y+ points into the room

    Door() = default;

    /**
     * @brief Constructor that orders points based on the smallest angular span.
     * Ensures p1 and p2 are ordered consistently.
     * @param point1 First candidate point.
     * @param angle1 Angle of first point.
     * @param point2 Second candidate point.
     * @param angle2 Angle of second point.
     */
    Door(Eigen::Vector2f point1, const float angle1, Eigen::Vector2f point2, const float angle2)
    {
        // Calculate angular difference both ways
        float diff_forward = angle2 - angle1;
        float diff_backward = angle1 - angle2;

        // Normalize differences to [0, 2π)
        if (diff_forward < 0) diff_forward += 2 * M_PI;
        if (diff_backward < 0) diff_backward += 2 * M_PI;

        // Choose ordering that gives smaller angular span
        if (diff_forward <= diff_backward)
        {
            p1 = point1; p1_angle = angle1;
            p2 = point2; p2_angle = angle2;
        } else
        {
            p1 = point2; p1_angle = angle2;
            p2 = point1; p2_angle = angle1;
        }
    }

    /** @return Euclidean width of the door opening. */
    [[nodiscard]] float width() const { return (p2 - p1).norm(); }

    /** @return Geometric center of the door in local coordiantes. */
    [[nodiscard]] Eigen::Vector2f center() const { return 0.5f * (p1 + p2); }

    /** @return Angle of the door center relative to the robot. */
    [[nodiscard]] float center_angle() const { const auto c=center(); return atan2(c.x(),c.y()); }

    /** @return Geometric center of the door in global coordinates. */
    [[nodiscard]] Eigen::Vector2f center_global() const { return 0.5f * (p1_global + p2_global); }

    /**
     * @brief Calculates a navigation target point in front of the door.
     * Computes a point offset by 'offset' distance from the center, along the normal vector.
     * It chooses the side closer to the center of the robot's current room.
     * @param robot_pos Current robot position.
     * @param center_point Room center reference.
     * @param offset Distance from the door threshold.
     * @return Target point coordinates.
     */
    [[nodiscard]] Eigen::Vector2f center_before(const Eigen::Vector2f &robot_pos, const Eigen::Vector2f &center_point, float offset = 500.f) const   // a point 500mm before the center along the door direction
    {
        // computer the normal to the door direction pointing towards the robot
        Eigen::Vector2f dir = p2 - p1;
        const float dir_norm = dir.norm();
        const float EPSILON = 1e-6f;
        if (dir_norm < EPSILON) return center(); // degenerate door, return center
        dir /= dir_norm;
        // perpendicular (normal) to door direction
        Eigen::Vector2f normal(-dir.y(), dir.x());
        // choose the normal that points toward the robot
        Eigen::Vector2f pA = center() + offset * normal;
        Eigen::Vector2f pB = center() - offset * normal;
        float dist_A = (pA - center_point).norm();
        float dist_B = (pB - center_point).norm();
        if (dist_A < dist_B) return pA;
        else return pB;
    }

    /** @return Orientation angle of the door threshold vector (p2 - p1). */
    [[nodiscard]] float direction() const
    {
        Eigen::Vector2f dir = p2 - p1;
        return std::atan2(dir.y(), dir.x());
    }
};
using Doors = std::vector<Door>;
using Wall = std::tuple<Eigen::ParametrizedLine<float, 2>, int, Corner, Corner>; // lines in general form ax + by + c = 0
using Walls = std::vector<Wall>;

#endif //COMMON_TYPES_H
