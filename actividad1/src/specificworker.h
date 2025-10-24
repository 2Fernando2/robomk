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
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H


// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <expected>
#include <genericworker.h>
#include <cppitertools/itertools.hpp>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <QPointF>

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

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();


public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();

	void new_target_slot(QPointF);

	std::optional<RoboCompLidar3D::TPoints> filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points);

	void update_report_posotion();

private:

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

	// graphics
	QRectF dimensions;
	AbstractGraphicViewer *viewer;
	QGraphicsPolygonItem *robot_polygon;

	struct LidarAngles
	{
		static constexpr float FRONT = 0.0f;
		static constexpr float FRONT_VISION = M_PI/8.f;
		static constexpr float FRONT_VISION_FW = M_PI/16.f;
		static constexpr float BACK = -M_PI;
		static constexpr float LEFT = -M_PI_2;
		static constexpr float RIGHT = M_PI_2;
		static constexpr float FRONT_LEFT = -3.f*M_PI/8.f; // FRONT_RIGHT is positive
		static constexpr float BACK_LEFT = -5.f*M_PI/8.f; // BACK_RIGHT is positive
	};

	const float ROBOT_LENGTH = 400.f;
	const float MIN_THRESHOLD = static_cast<float>(ROBOT_LENGTH) * 2.f;
	const float MAX_ADV_SPEED = 1000.f;
	float adv_speed = MAX_ADV_SPEED;
	float rot_speed = 0.f;
	bool rotating = false;
	bool rot_direction = false; // true: right - false: left
	bool spiraling = false;

	enum class State{IDLE, FORWARD, TURN, FOLLOW_WALL, SPIRAL};
	SpecificWorker::State state = SpecificWorker::State::FORWARD;
	std::optional<RoboCompLidar3D::TPoints> read_data();
	std::tuple<SpecificWorker::State, float, float> forward(auto &points);
	std::tuple<SpecificWorker::State, float, float> turn(auto &points);
	std::tuple<SpecificWorker::State, float, float> follow_wall(auto &points);
	std::tuple<SpecificWorker::State, float, float> spiral(auto &points);
	std::expected<bool, std::string> open_space(auto &points);

	std::optional<RoboCompLidar3D::TPoints> filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d);
	std::expected<int, std::string> closest_lidar_index_to_given_angle(const auto &points, float angle);
	void draw_lidar(auto &filtered_points, QGraphicsScene *scene);

signals:
	//void customSignal();
};

#endif
