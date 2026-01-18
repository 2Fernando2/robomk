/*
 *    Copyright (C) 2025 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author Alvaro Tardío Fernández
	@author Fernando Perez de vega
	@author Alejandro Orozco Santano
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H


// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <expected>
#include <random>
#include <doublebuffer/DoubleBuffer.h>
#include "time_series_plotter.h"
#include <cmath>

#ifdef emit
#undef emit
#endif
#include <execution>
#include <tuple>
#include <utility>
#include <vector>
#include <limits>    // Para std::numeric_limits
#include "room_detector.h"
#include "hungarian.h"
#include "nominal_room.h"
#include "door_detector.h"
#include "image_processor.h"
#include "pointcloud_center_estimator.h"
#include "door_crossing_tracker.h"

#include <ranges>
#include <expected>

#include <algorithm>
#include <cppitertools/enumerate.hpp>
#include "common_types.h"
#include "ransac_line_detector.h"
#include "time_series_plotter.h"
#include "cppitertools/itertools.hpp"


// input de teclado manual
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <cctype>

/**
 * @brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    /**
     * @brief Constructor for SpecificWorker.
     * Initilizes the state machine and checks startup flags.
     * @param configLoader Configuration loader for the component.
     * @param tprx Tuple of proxies required for the component.
     * @param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
	 *  @brief Destructor of the class.
	 *  Stop the robot and releases graphic resources before exiting.
	 */
	~SpecificWorker();


public slots:

	/**
	 * @brief  Initializes graphic components and viewers.
	 * Configures the viewer, loads room dimensions, and prepares telemetry plots.
	 */
	void initialize();
	/**
	 * @brief Main execution loop (Compute).
	 * Executed periodically. Reads sensors, runs the state machine logic, update pose, and sends commands to the robot.
	 */
	void compute();
	/**
	 * @brief Emergency state handler.
	 * Invoked when the component enters a critical error state.
	 */
	void emergency();
	/**
	 * @brief Restores normal operation after an emergency.
	 */
	void restore();
	/**
	 * @brief Performs initial connection checks.
	 * @return 0 if successful.
	 */
	int startup_check();
	/**
	 * @brief Slot to receive new navigation targets from the GUI.
	 * @param point Coordinates of the new target.
	 * @note Currently only prints debug info.
	 */
	void new_target_slot(QPointF);
	/**
	 * @brief Updates the robot's position visualization based on pure odometry.
	 * Reads the base state from the omnirobot proxy and updates the graphic.
	 */
	void update_report_posotion();

private:

	bool startup_check_flag;

	struct Params
	{
		float ROBOT_WIDTH = 460;  // mm
		float ROBOT_LENGTH = 480;  // mm // const float ROBOT_LENGTH = 400.f;
		float MAX_ADV_SPEED = 1000; // mm/s // const float MAX_ADV_SPEED = 1000.f;
		float MAX_ROT_SPEED = 1; // rad/s
		float MAX_SIDE_SPEED = 50; // mm/s
		float MAX_TRANSLATION = 500; // mm/s
		float MAX_ROTATION = 0.2;
		float STOP_THRESHOLD = 700; // mm // const float MIN_THRESHOLD = static_cast<float>(ROBOT_LENGTH) * 2.f;
		float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3; // mm // float adv_speed = MAX_ADV_SPEED;
		float LIDAR_FRONT_SECTION = 0.2; // rads, aprox 12 degrees
		// wall
		float LIDAR_DISTANCE = 12000.f;
		float LIDAR_RIGHT_SIDE_SECTION = M_PI/3; // rads, 90 degrees
		float LIDAR_LEFT_SIDE_SECTION = -M_PI/3; // rads, 90 degrees
		float WALL_MIN_DISTANCE = ROBOT_WIDTH*1.2;
		// match error correction
		float MATCH_ERROR_SIGMA = 150.f; // mm
		float DOOR_REACHED_DIST = 400.f;
		// std::string LIDAR_NAME_LOW = "pearl";
		// std::string LIDAR_NAME_HIGH = "helios";
		QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};

		// relocalization
		// float RELOCAL_CENTER_EPS = 300.f;    // mm: stop when |mean| < eps
		// float RELOCAL_KP = 0.002f;           // gain to convert mean (mm) -> speed (magnitude)
		// float RELOCAL_MAX_ADV = 300.f;       // mm/s cap while re-centering
		// float RELOCAL_MAX_SIDE = 300.f;      // mm/s cap while re-centering
		float RELOCAL_ROT_SPEED = 0.8f;     // rad/s while aligning
		// float RELOCAL_DELTA = 5.0f * M_PI/180.f; // small probe angle in radians
		// float RELOCAL_MATCH_MAX_DIST = 2000.f;   // mm for Hungarian gating
		// float RELOCAL_DONE_COST = 500.f;
		float RELOCAL_DONE_MATCH_MAX_ERROR = 2000.f;
		float RELOCAL_MAX_ORIENTED_ERROR = 0.1f;
	};
	Params params;

	float max_error_registered = -1.0;

	// viewer
	AbstractGraphicViewer *viewer, *viewer_room;
	QGraphicsPolygonItem *robot_draw, *robot_room_draw;
	QGraphicsRectItem *room_draw;

	// robot
	Eigen::Affine2f robot_pose;  // Eigen type to represent a rotation+translation

	// rooms
	std::vector<NominalRoom> nominal_rooms{NominalRoom{5500.f, 4000.f}, NominalRoom{8000.f, 4000.f}};
	int current_room = 0;
	rc::Room_Detector room_detector; // object to compute the corners
	rc::PointcloudCenterEstimator room_detector_estimator;
    rc::Hungarian hungarian; // object to match the two sets of corners

	// Door Crossing
	DoorCrossing door_crossing;
	std::vector<std::vector<std::optional<DoorCrossing>>> door_crossing_data;

	// Doors
	DoorDetector door_detector;
	Doors doors;
	Doors last_doors;
	int current_door = -1;
	bool door_selected = false;

	// random number generator
	std::random_device rd;

	// plotter
	std::unique_ptr<TimeSeriesPlotter> time_series_plotter;
	int match_error_graph; // To store the index of the speed graph

	// graphics
	QRectF dimensions{-5000, 2500, 10000, -5000};
	QRectF room_dimensions{-5000, -2500, 10000, 5000};

	// image processor
	rc::ImageProcessor image_processor;

	// relocalization
	bool relocal_centered = false;
	bool localised = false;

	// - STATE MACHINE RELATED -
	enum class STATE {GOTO_ROOM_CENTER, TURN, GOTO_DOOR, ORIENT_TO_DOOR, CROSS_DOOR};
	inline const char* to_string(const STATE s) const
	{
		switch(s) {
		case STATE::GOTO_DOOR:          return "GOTO_DOOR";
		case STATE::TURN:               return "TURN";
		case STATE::ORIENT_TO_DOOR:     return "ORIENT_TO_DOOR";
		case STATE::GOTO_ROOM_CENTER:   return "GOTO_ROOM_CENTER";
		case STATE::CROSS_DOOR:         return "CROSS_DOOR";
		default:                        return "UNKNOWN";
		}
	}
	STATE state = STATE::GOTO_ROOM_CENTER;
	using RetVal = std::tuple<STATE, float, float>;

	/**
	 * @brief Core of the state machine.
	 * Decides state transitions and calculates velocities based on the current state.
	 * @param points Filtered Lidar point cloud.
	 * @param center_point Estimated geometric center of the current room.
	 * @param corners Detected corners for relocalization.
	 * @param max_math_error Current geometric matching error.
	 * @return Tuple containing {Next State, Advance Velocity, Rotation Velocity}.
	 */
	RetVal state_machine(const RoboCompLidar3D::TPoints &points, const Eigen::Vector2f &center_point, const Corners &corners, const float &max_math_error);

	/**
	 * @brief Logic for the GOTO_ROOM_CENTER state.
	 * Directs the robot towards the estimated geometric center of the room.
	 * @param points Current point cloud.
	 * @return RetVal with movement commands.
	 */
	RetVal goto_room_center(const RoboCompLidar3D::TPoints &points);
	/**
	 * @brief Logic for the TURN state.
	 * Rotates the robot to search for numbers using DNN and relocalize.
	 * @param corners Geometric corners to refine pose during turn.
	 * @return RetVal with rotation commands.
	 */
	RetVal turn(const Corners &corners);

	/**
	* @brief Logic for the GOTO_DOOR state.
	* Navigates towards the approach position of a selected door.
	* @param points Lidar point cloud.
	* @param center_point Room center (used to calculate the "center_before" approach point).
	* @param scene Graphic scene to draw the debug target.
	* @return RetVal with approach commands.
	*/
	RetVal goto_door(const RoboCompLidar3D::TPoints &points, const Eigen::Vector2f &center_point, QGraphicsScene *scene);

	/**
	 * @brief Logic for the ORIENT_TO_DOOR state.
	 * Aligns the robot perpendicularly to the door frame before crossing.
	 * @param max_match_error Current localization error to decide whether to abort.
	 * @return RetVal with fine alignment commands.
	 */
	RetVal orient_to_door(const float &max_match_error);

	/**
	 * @brief Logic for the CROSS_DOOR state.
	 * Executes a "blind" or ballistic movement to traverse the door frame.
	 * @param points Point cloud (used to detect if the door has been crossed).
	 * @return RetVal with constant crossing velocity.
	 */
	RetVal cross_door(const RoboCompLidar3D::TPoints &points);

	// - DRAWING METHODS -
	/**
	 * @brief Draws Lidar points and estimated center in the viewer.
	 * @param filtered_points Filtered points.
	 * @param room_center Coordinate of the calculatd center.
	 * @param scene Qt scene where to draw.
	 */
	void draw_lidar(auto &filtered_points, Eigen::Vector2d room_center, QGraphicsScene *scene);
	/**
	 * @brief Draws the current navigation target in the viewer.
	 * @param point Target coordinates.
	 * @param scene Qt scene where to draw.
	 * @param last_iteratior Flag to clear the previous drawing or keep it.
	 */
	void draw_target(const Eigen::Vector2d &point, QGraphicsScene *scene, bool last_iteratior);
	/**
	 * @brief Draws the rectangle representing the current nominal room.
	 */
	void draw_nominal_room();
	/**
	 * @brief Draws the nominal doors in their global coordinates.
	 * @param scene Qt scene where to draw.
	 */
	void draw_nominal_doors(QGraphicsScene *scene);

	// - LIDAR MANAGEMENT -
	/**
	 * @brief Fetches and processes Lidar data.
	 * Calls the proxy and applies distance and isolated point filters.
	 * @return Optional with the filtered point cloud or nullptr if error.
	 */
	std::optional<RoboCompLidar3D::TPoints> read_data();

	/**
	 * @brief Searches for the index of the Lidar point closest to a given angle.
	 * @param points Point cloud
	 * @param angle Target angle in radians.
	 * @return Index in the vector or error if not found.
	 */
	std::expected<int, std::string> closest_lidar_index_to_given_angle(const auto &points, float angle);

	/**
	 * @brief Filters the point cloud by reducing angular density.
	 * Groups points by angular sector and keeps the closest one (minimum distance).
	 * @param points Original point cloud.
	 * @return Decimated point cloud.
	 */
	std::optional<RoboCompLidar3D::TPoints> filter_min_distance(const RoboCompLidar3D::TPoints &points);

	/**
	 * @brief Removes isolated points (noise) that do not have close neighbors.
	 * @param points Point cloud.
	 * @param d Maximum idstance to consider another point as a neighbor.
	 * @return Denoised point cloud.
	 */
	std::optional<RoboCompLidar3D::TPoints> filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d);

	/**
	 * @brief Filters unstable door detections by comparing with the previous frame.
	 * @param current_doors Doors detected in the current frame.
	 * @return List of temporally validated doors.
	 */
	Doors validate_doors(const Doors &current_doors);

	// - ROBOT POSE -
	/**
	 * @brief Proportional Controller to reach a target (x, y).
	 * Calculates necessary advance and rotation velocities.
	 * @param target Target coordinates in the robot's local frame.
	 * @return Tuple {Advance Velocity, Rotation Velocity, Angular Error}.
	 */
	std::tuple<float, float, double> do_work(const Eigen::Vector2d &target);

	/**
	 * @brief Updates global robot pose using geometric alignment.
	 * Uses Pseudoinverse to minimize error between observed corners and the nominal map.
	 * @param corners Locally detected corners.
	 * @param r_pose Current estimated robot pose.
	 * @param transform_corners If true, transforms nominal corners to local frame before matching.
	 * @return Optional with pair {New Pose, Match Error} if converged.
	 */
	std::optional<std::pair<Eigen::Affine2f, float>> update_robot_pose(const Corners &corners, const Eigen::Affine2f &r_pose, bool transform_corners);

	/**
	 * @brief Sends velocities commands to the robot (Omnirobot).
	 * @param adv Advance velocity.
	 * @param rot Rotation velocity.
	 * @param max_match_error Current localization error (used to slow down if error is high).
	 */
	void move_robot(float adv, float rot, float max_match_error);

signals:
	//void customSignal();
};

#endif
