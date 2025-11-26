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
	@author Alejandro Orozco Santano
	@author Alvaro Tardío Fernández
	@author Fernando Perez de vega
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
#include "room_detector.h"
#include "hungarian.h"
#include "nominal_room.h"
#include "door_detector.h"
#include "image_processor.h"

#include <ranges>
#include <expected>

#include <algorithm>
#include <cppitertools/enumerate.hpp>
#include "common_types.h"
#include "ransac_line_detector.h"
#include "time_series_plotter.h"
#include "cppitertools/itertools.hpp"

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    /**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
	~SpecificWorker();


public slots:

	void initialize();
	void compute();
	void emergency();
	void restore();
	int startup_check();
	void new_target_slot(QPointF);
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
		float LIDAR_RIGHT_SIDE_SECTION = M_PI/3; // rads, 90 degrees
		float LIDAR_LEFT_SIDE_SECTION = -M_PI/3; // rads, 90 degrees
		float WALL_MIN_DISTANCE = ROBOT_WIDTH*1.2;
		// match error correction
		float MATCH_ERROR_SIGMA = 150.f; // mm
		float DOOR_REACHED_DIST = 300.f;
		std::string LIDAR_NAME_LOW = "bpearl";
		std::string LIDAR_NAME_HIGH = "helios";
		QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};

		// relocalization
		float RELOCAL_CENTER_EPS = 300.f;    // mm: stop when |mean| < eps
		float RELOCAL_KP = 0.002f;           // gain to convert mean (mm) -> speed (magnitude)
		float RELOCAL_MAX_ADV = 300.f;       // mm/s cap while re-centering
		float RELOCAL_MAX_SIDE = 300.f;      // mm/s cap while re-centering
		float RELOCAL_ROT_SPEED = 0.3f;     // rad/s while aligning
		float RELOCAL_DELTA = 5.0f * M_PI/180.f; // small probe angle in radians
		float RELOCAL_MATCH_MAX_DIST = 2000.f;   // mm for Hungarian gating
		float RELOCAL_DONE_COST = 500.f;
		float RELOCAL_DONE_MATCH_MAX_ERROR = 1000.f;

		bool rotating = false;
		bool rot_direction = false; // true: right - false: left
		bool spiraling = false;
		float rot_speed = 0.f;

	};
	Params params;

	// viewer
	AbstractGraphicViewer *viewer, *viewer_room;
	QGraphicsPolygonItem *robot_draw, * robot_room_draw;
	QGraphicsRectItem *room_draw;

	// robot
	Eigen::Affine2d robot_pose;  // Eigen type to represent a rotation+translation

	// rooms
	std::vector<NominalRoom> nominal_rooms{NominalRoom{5500.f, 4000.f}, NominalRoom{8000.f, 4000.f}};
	int room_index = 0;
	rc::Room_Detector room_detector; // object to compute the corners
	rc::Hungarian hungarian; // object to match the two sets of corners
	QColor colors[2] = {QColor("red"), QColor("green")};

	// state machine
	enum class STATE {GOTO_DOOR, ORIENT_TO_DOOR, LOCALISE, GOTO_ROOM_CENTER, TURN, IDLE, CROSS_DOOR};
	inline const char* to_string(const STATE s) const
	{
		switch(s) {
		case STATE::IDLE:               return "IDLE";
		case STATE::LOCALISE:           return "LOCALISE";
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

	RetVal state_machine(const RoboCompLidar3D::TPoints &points, const Match &match, const Corners &corners, const Lines &lines, const Eigen::Vector2d &door_center
		, const float &max_math_error);

	RetVal localise(const Match &match);
	RetVal goto_room_center(const Lines &lines);
	RetVal turn(const Corners &corners);
	RetVal update_pose(const Corners &corners, const Match &match);

	RetVal orient_to_door(const Eigen::Vector2d &target, const float &max_match_error);
	RetVal goto_door(const RoboCompLidar3D::TPoints &points, const float &max_match_error);
	RetVal cross_door(const RoboCompLidar3D::TPoints &points);
	RetVal process_state(const RoboCompLidar3D::TPoints &data, const Corners &corners, const Match &match, AbstractGraphicViewer *viewer);

	//Draw
	void draw_lidar(auto &filtered_points, Eigen::Vector2d room_center, QGraphicsScene *scene);
	void draw_target(const Eigen::Vector2d &point, QGraphicsScene *scene, bool last_iteratior);

	//aux
	std::optional<RoboCompLidar3D::TPoints> read_data();
	std::expected<int, std::string> closest_lidar_index_to_given_angle(const auto &points, float angle);
	std::optional<RoboCompLidar3D::TPoints> filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints &points);
	std::optional<RoboCompLidar3D::TPoints> filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d);
	void print_match(const Match &match, const float error =1.f) const;
	std::tuple<float, float, double> do_work(const Eigen::Vector2d target);
	Eigen::Vector2d target_;

	// random number generator
	std::random_device rd;

	// DoubleBuffer for velocity commands
	DoubleBuffer<std::tuple<float, float, float, long>, std::tuple<float, float, float, long>> commands_buffer;
	std::tuple<float, float, float, long> last_velocities{0.f, 0.f, 0.f, 0.f};

	// plotter
	std::unique_ptr<TimeSeriesPlotter> time_series_plotter;
	int match_error_graph; // To store the index of the speed graph

	// Doors
	DoorDetector door_detector;
	Doors doors;
	Eigen::Vector2d door_center;

	// graphics
	QRectF dimensions{-5000, 2500, 10000, -5000};
	QRectF room_dimensions{-5000, -2500, 10000, 5000};

	// image processor
	rc::ImageProcessor image_processor;

	// timing

	// relocalization
	bool relocal_centered = false;
	bool localised = false;

	bool update_robot_pose(const Corners &corners, const Match &match);
	void move_robot(float adv, float rot, float max_match_error);
	Eigen::Vector3d solve_pose(const Corners &corners, const Match &match);
	void predict_robot_pose();
	std::tuple<float, float> robot_controller(const Eigen::Vector2f &target);


signals:
	//void customSignal();
};

#endif
