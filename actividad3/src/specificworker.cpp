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
#include "specificworker.h"
#include <ranges>
#include <random>
#include <expected>
#include <vector>

#ifdef emit
#undef emit
#endif
#include <execution>
#include <expected>
#include <algorithm>
#include <cppitertools/enumerate.hpp>
#include "common_types.h"
#include "hungarian.h"
#include "ransac_line_detector.h"
#include "room_detector.h"
#include "time_series_plotter.h"

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif
		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

	if (this->startup_check_flag) {this->startup_check();}
	else
	{
		// Viewer
		viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
		auto [r, e] = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_draw = r;
		//this->resize(900, 450);

		viewer_room = new AbstractGraphicViewer(this->frame_room, this->dimensions);
		auto [rr, re] = viewer_room->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_room_draw = rr;

		// draw room in viewer_room
		viewer_room->scene.addRect(nominal_rooms[0].rect(), QPen(Qt::black, 30));

		//viwer_room->show();
		show();

		// init robot pose
		robot_pose.setIdentity();
		robot_pose.translate(Eigen::Vector2d(0.0,0.0));

		//time series plotter for match error
		TimeSeriesPlotter::Config plotConfig;
		plotConfig.title = "Maximun Match Error Over Time";
		plotConfig.yAxisLabel = "Error (mm)";
		plotConfig.timeWindowSeconds = 15.0; //Show a 15-second window
		plotConfig.autoScaleY = false;       //We will set a fixed range
		plotConfig.yMin = 0;
		plotConfig.yMax = 1000;

		time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, plotConfig);
		match_error_graph =  time_series_plotter->addGraph("", Qt::blue);

		//stop robot
		//move_robot(0, 0, 0);

	}
}

void SpecificWorker::compute()
{
	RoboCompLidar3D::TPoints points;
	if ( auto filter_data = read_data(); not filter_data.has_value())
	{ std::cerr << "No filter data found" << std::endl; return;}
	else points = filter_data.value();
	points = door_detector.filter_points(points, &viewer->scene);
//	draw_lidar(points, &viewer->scene);


	// compute corners
	const auto &[corners, lines] = room_detector.compute_corners(points, &viewer->scene);
	const auto center_opt = room_detector.estimate_center_from_walls(lines);
	if (center_opt.has_value())
		draw_lidar(points, center_opt.value(), &viewer->scene);
	else
	{ std::cerr << "No center room point found" << std::endl; return;}

	// match corners  transforming first nominal corners to robot's frame
	const auto match = hungarian.match(corners, nominal_rooms[0].transform_corners_to(robot_pose.inverse()));


	// compute max of  match error
	float max_match_error = 99999.f;
	if (not match.empty())
	{
		const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
			{ return std::get<2>(a) < std::get<2>(b); });
		max_match_error = static_cast<float>(std::get<2>(*max_error_iter));
		time_series_plotter->addDataPoint(match_error_graph,max_match_error);
		//print_match(match, max_match_error); //debugging
	}


	// update robot pose
	if (localised)
		update_robot_pose(corners, match);


	// Process state machine
	RetVal ret_val = process_state(data, corners, match, viewer);
	auto [st, adv, rot] = ret_val;
	state = st;


	// Send movements commands to the robot constrained by the match_error
	//qInfo() << __FUNCTION__ << "Adv: " << adv << " Rot: " << rot;
	move_robot(adv, rot, max_match_error);


	// draw robot in viewer
	robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
	const double angle = qRadiansToDegrees(std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0)));
	robot_room_draw->setRotation(angle);


	// update GUI
	time_series_plotter->update();
	lcdNumber_adv->display(adv);
	lcdNumber_rot->display(rot);
	lcdNumber_x->display(robot_pose.translation().x());
	lcdNumber_y->display(robot_pose.translation().y());
	lcdNumber_angle->display(angle);
	last_time = std::chrono::high_resolution_clock::now();;


	// corners,match, max_distance
	// compute corners
	//const auto &[corners, lines] = room_detector.compute_corners(points, &viewer->scene);
	// room_detector.estimate_center_from_walls(lines);

	// match
	// const auto match = hungarian.match(corners, nominal_rooms[0].transform_corners_to(robot_pose.inverse()));

	// compute max of match error

	// if(localised)
		  // update_robot_pose(corners, match);

	//state machine


	// const auto corners = room_detector.compute_corners(filter_data.value(), &viewer->scene);
	// const auto match = hungarian.match(std::get<0>(corners), nominal_rooms[0].transform_corners_to(robot_pose.inverse()), 1000);
	// // robot to room -> robot_pose | room to robot -> robot_pose.inverse()
	// for (auto &m : match)
 //    {
 //        qDebug() << std::get<0>(std::get<0>(m)).x() << " " << std::get<0>(std::get<0>(m)).y();
 //        qDebug() << std::get<0>(std::get<0>(m)).x() << " " << std::get<0>(std::get<0>(m)).y();
 //    }
 //    qDebug() << "----------------------------";
 //
	// // matrix
	// Eigen::MatrixXd W(match.size()*2,3);
 //    Eigen::VectorXd b(match.size()*2);
 //    for (const auto &&[i, m] : match | iter::enumerate)
 //    {
 //        auto &[meas_c, nom_c, _] = m;
 //        auto &[p_meas, __, ___] = meas_c;
 //        auto &[p_nom, ____, _____] = nom_c;
 //        b(2*i) = p_nom.x() - p_meas.x();
	// 	b(2*i+1) = p_nom.y() - p_meas.y();
	// 	W.block<1, 3>(2*i, 0) << 1.0, 0.0, -p_meas.y();
	// 	W.block<1, 3>(2*i+1, 0) << 0.0, 1.0, p_meas.x();
 //    }
 //    // estimate new pose with pseudoinverse
 //    const auto r = (W.transpose()*W).inverse()*W.transpose()*b;
 //    if (r.array().isNaN().any()) return;
 //
 //    // update robot pose
 //    robot_pose.translate(Eigen::Vector2d(r(0), r(1)));
 //    robot_pose.rotate(r(2));
 //
 //    // update robot draw
 //    robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
 //    double angle = std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
	// robot_room_draw->setRotation(qRadiansToDegrees(angle));
 //
	// auto result = state_Machine(filter_data);
	// omnirobot_proxy->setSpeedBase(0, std::get<1>(result), std::get<2>(result));
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::state_Machine(auto& points)
{
	auto result = std::tuple<SpecificWorker::State, float, float>();

	switch (state_)
	{
	case SpecificWorker::State::IDLE:
		break;

	case SpecificWorker::State::FORWARD:
		qInfo() << "State: FORWARD";
		result = forward(points.value());
		break;

	case SpecificWorker::State::TURN:
		qInfo() << "State: TURN";
		result = turn(points.value());
		break;

	case State::FOLLOW_WALL:
		qInfo() << "State: FOLLOW_WALL";
		result = follow_wall(points.value());
		break;
	case State::SPIRAL:
		qInfo() << "State: SPIRAL";
		result = spiral(points.value());
		break;
	default: break;
	}
	state_ = std::get<SpecificWorker::State>(result);

	return result;
}

SpecificWorker::RetVal SpecificWorker::goto_door(const RoboCompLidar3D::TPoints &points)
{
	return{};
}

SpecificWorker::RetVal SpecificWorker::orient_to_door(const RoboCompLidar3D::TPoints &points)
{
	return{};
}

SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points)
{
	return{};
}

SpecificWorker::RetVal SpecificWorker::localise(const Match &match)
{
	return{};
}

SpecificWorker::RetVal SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints &points)
{
	return{};
}

SpecificWorker::RetVal SpecificWorker::update_pose(const Corners &corners, const Match &match)
{
	return{};
}

SpecificWorker::RetVal SpecificWorker::turn(const Corners &corners)
{
	return{};
}

SpecificWorker::RetVal SpecificWorker::process_state(const RoboCompLidar3D::TPoints &data, const Corners &corners, const Match &match, AbstractGraphicViewer *viewer)
{
	return{};
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::spiral(auto &points)
{
	// Switch state condition

	auto min_point = get_min_point(points, -LidarAngles::FRONT_VISION, LidarAngles::FRONT_VISION);

	if (min_point->distance2d <= MIN_THRESHOLD){
		//qInfo() << "SPIRAL ending, turning now..";
		spiraling = false;
		return std::make_tuple(SpecificWorker::State::TURN, 0.f, 0.f);}

	// vel  100.f --++
	// rot  -1/1 ++--
	if (!spiraling){ // first time
		// Lateral check: ¿right or left?
		//qInfo() << "SPIRAL starting..";
		auto left = closest_lidar_index_to_given_angle(points, LidarAngles::LEFT);
		auto right = closest_lidar_index_to_given_angle(points, LidarAngles::RIGHT);
		if (not left or not right){std::cout << left.error() << " " << right.error() << std::endl; return {};}
		auto left_point = points[left.value()];
		auto right_point = points[right.value()];
		bool left_side = left_point.distance2d < right_point.distance2d;
		                              
		spiraling = true;
		adv_speed = 30.f;
		rot_direction = !left_side;
		rot_speed = left_side ? 1.f : -1.f;
	}
	if (adv_speed < MAX_ADV_SPEED) adv_speed += 5.f*std::sqrt(0.9f)/2.f;
	if (rot_direction){ // left (-)
		rot_speed+=0.001f;
		if(rot_speed >= 0.f) rot_speed = 0.f;}
	else{ // right (+)
		rot_speed-=0.001f;
		if(rot_speed <= 0.f) rot_speed = 0.f;}
	//qInfo() << "adv_speed: " << adv_speed << " - rot_speed: " << rot_speed;
	return std::make_tuple(SpecificWorker::State::SPIRAL, adv_speed, rot_speed);
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::turn(auto& points)
{
	// Switch state condition
	auto begin_offset = closest_lidar_index_to_given_angle(points,  -LidarAngles::FRONT_VISION);
	auto end_offset = closest_lidar_index_to_given_angle(points, LidarAngles::FRONT_VISION);
	if (not begin_offset or not end_offset){std::cout << begin_offset.error() << " " << end_offset.error() << std::endl; return {};}
	auto min = std::min_element(std::begin(points) + begin_offset.value(),  std::begin(points) + end_offset.value(),
								[](const auto& p1, const auto& p2){return p1.distance2d < p2.distance2d;});

	if (!rotating){ // first time rotating
		auto left_begin_offset = closest_lidar_index_to_given_angle(points, LidarAngles::LEFT);
		auto left_end_offset = closest_lidar_index_to_given_angle(points, -LidarAngles::FRONT_LEFT);
		if (not left_begin_offset or not left_end_offset){std::cout << left_begin_offset.error() << " " << left_end_offset.error() << std::endl; return {};}
		auto left_avg_distance = std::accumulate(std::begin(points) + left_begin_offset.value(), std::begin(points) + left_end_offset.value(), 0.0f,
												 [](const float acu, const auto& p){return acu + p.distance2d;});
		auto right_begin_offset = closest_lidar_index_to_given_angle(points, LidarAngles::FRONT_LEFT);
		auto right_end_offset = closest_lidar_index_to_given_angle(points, LidarAngles::RIGHT);
		if (not right_begin_offset or not right_end_offset){std::cout << right_begin_offset.error() << " " << right_end_offset.error() << std::endl; return {};}
		auto right_avg_distance = std::accumulate(std::begin(points) + right_begin_offset.value(), std::begin(points) + right_end_offset.value(), 0.0f,
												  [](const float acu, const auto& p){return acu + p.distance2d;});
		left_avg_distance /= static_cast<float>(left_end_offset.value() - left_begin_offset.value());
		right_avg_distance /= static_cast<float>(right_end_offset.value() - right_begin_offset.value());
		rot_speed = left_avg_distance <= right_avg_distance ? 0.7f : -0.7f;
		rot_direction = left_avg_distance <= right_avg_distance;
	}
	if (min->distance2d > MIN_THRESHOLD) // to stop turning
	{
		rotating = false;
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_int_distribution<int> dice(1, 2);
		auto _state = (dice(gen) == 1) ? SpecificWorker::State::FORWARD : SpecificWorker::State::FOLLOW_WALL;
		return std::make_tuple(_state, 0.f, 0.f);
	}
	rotating = true;
	return std::make_tuple(SpecificWorker::State::TURN, 0.f, rot_speed);
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::forward(auto &points)
{
	// Switch state condition
	auto begin_offset = closest_lidar_index_to_given_angle(points,  -LidarAngles::FRONT_VISION);
	auto end_offset = closest_lidar_index_to_given_angle(points, LidarAngles::FRONT_VISION);
	if (not begin_offset or not end_offset){std::cout << begin_offset.error() << " " << end_offset.error() << std::endl; return {};}
	auto min = std::min_element(std::begin(points) + begin_offset.value(),  std::begin(points) + end_offset.value(),
								[](const auto& p1, const auto& p2){return p1.distance2d < p2.distance2d;});

	//  && point->phi == std::clamp(point->phi, -LidarAngles::FRONT_VISION, LidarAngles::FRONT_VISION)
	if (min->distance2d <= MIN_THRESHOLD)
		return std::make_tuple(SpecificWorker::State::TURN, 0.f, 0.f);
	min = std::min_element(std::begin(points), std::end(points), [](const auto& p1, const auto& p2){return p1.distance2d < p2.distance2d;});
	if (min->distance2d > MIN_THRESHOLD)
		return std::make_tuple(SpecificWorker::State::SPIRAL, 0.f, 0.f);
	return std::make_tuple(SpecificWorker::State::FORWARD, 1000.f, 0.f);
}



std::tuple<SpecificWorker::State, float, float> SpecificWorker::follow_wall(auto &points)
{
	// Switch state condition
	auto begin_offset = closest_lidar_index_to_given_angle(points, -LidarAngles::FRONT_VISION);
	auto end_offset = closest_lidar_index_to_given_angle(points, LidarAngles::FRONT_VISION);
	if (not begin_offset or not end_offset){std::cout << begin_offset.error() << " " << end_offset.error() << std::endl; return {};}
	auto min_point = std::min_element(std::begin(points) + begin_offset.value(), std::begin(points) + end_offset.value(),
									  [](const auto& p1, const auto& p2) {return p1.distance2d < p2.distance2d;});
	if (min_point->distance2d <= MIN_THRESHOLD)
		return std::make_tuple(SpecificWorker::State::TURN, 0.f, 0.f);

	// Side check
	auto left = closest_lidar_index_to_given_angle(points, LidarAngles::LEFT);
	auto right = closest_lidar_index_to_given_angle(points, LidarAngles::RIGHT);
	if (not left or not right){std::cout << left.error() << " " << right.error() << std::endl; return {};}
	auto left_point = points[left.value()];
	auto right_point = points[right.value()];
	bool left_side = (left_point.distance2d < right_point.distance2d);
	auto lat_side_point = left_side ? left_point : right_point;
	float angle_side_begin = left_side ? LidarAngles::BACK_LEFT : -LidarAngles::FRONT_LEFT;
	float angle_side_end = left_side ? LidarAngles::FRONT_LEFT : -LidarAngles::BACK_LEFT;
	auto side_begin_offset = closest_lidar_index_to_given_angle(points, angle_side_begin);
	auto side_end_offset = closest_lidar_index_to_given_angle(points, angle_side_end);
	if (not side_begin_offset or not side_end_offset){std::cout << side_begin_offset.error() << "" << side_end_offset.error() << std::endl; return {};}
	auto min_side_point = std::min_element(std::begin(points) + side_begin_offset.value(), std::begin(points) + side_end_offset.value(),
										   [](const auto& p1, const auto& p2){return p1.distance2d < p2.distance2d;});
	// qInfo() << "Side angle: " << lat_side_point.phi << " - Min side angle: " << min_side_point->phi;

	rot_speed = min_side_point->phi > lat_side_point.phi ? 0.05f : -0.05f;
	bool angle_check = min_side_point->phi < lat_side_point.phi+0.01f and min_side_point->phi > lat_side_point.phi-0.01f;
	bool distance_check = min_side_point->distance2d < MIN_THRESHOLD+50.f and min_side_point->distance2d > MIN_THRESHOLD-50.f;
	if (angle_check and distance_check){
		//qInfo() << "Going straight!";
		return std::make_tuple(SpecificWorker::State::FOLLOW_WALL, 1000.f, 0.f);}
	//if(rot == 0.05f) qInfo() << "Adjusting right!"; else qInfo() << "Adjusting left!";
	return std::make_tuple(SpecificWorker::State::FOLLOW_WALL, 1000.f, rot_speed);
}

std::optional<RoboCompLidar3D::TPoint> SpecificWorker::get_min_point(const auto &points, const auto &angle_begin, const auto &angle_end)
{
	auto begin_offset = closest_lidar_index_to_given_angle(points, angle_begin);
	auto end_offset = closest_lidar_index_to_given_angle(points, angle_end);
	if (not begin_offset or not end_offset){std::cout << begin_offset.error() << " " << end_offset.error() << std::endl; return {};}
	auto min = std::min_element(std::begin(points)+begin_offset.value(), std::begin(points)+end_offset.value(),
		[](const auto& p1, const auto& p2){return p1.distance2d < p2.distance2d;});
	return *min;
}

std::expected<bool, std::string> SpecificWorker::open_space(auto &points)
{
	auto front_begin_offset = closest_lidar_index_to_given_angle(points, LidarAngles::BACK_LEFT);
	auto front_end_offset = closest_lidar_index_to_given_angle(points, -LidarAngles::BACK_LEFT);
	if (not front_begin_offset or not front_end_offset){std::cout << front_begin_offset.error() << " " << front_end_offset.error() << std::endl;
		return std::unexpected("Error calculating front begin or end offset in <open_space> method");}
	auto front_avg_distance = std::accumulate(std::begin(points) + front_begin_offset.value(),
											  std::begin(points) + front_end_offset.value(), 0.0f,
											  [](const float acu, const auto& p) {return acu + p.distance2d;});
	front_avg_distance /= (3.f*static_cast<float>(points.size())/4.f);

	auto back_end_offset_1 = closest_lidar_index_to_given_angle(points, LidarAngles::FRONT_LEFT);
	auto back_begin_offset_2 = closest_lidar_index_to_given_angle(points, -LidarAngles::FRONT_LEFT);
	if (not back_end_offset_1 or not back_begin_offset_2){std::cout << back_end_offset_1.error() << " " << back_begin_offset_2.error() << std::endl;
		return std::unexpected("Error calculating back begin or end offset in <open_space> method");}
	auto back_avg_distance = std::accumulate(std::begin(points), std::begin(points)+back_end_offset_1.value(), 0.0f,
											 [](const float acu, const auto& p) {return acu + p.distance2d;});
	back_avg_distance +=std::accumulate(std::begin(points)+back_begin_offset_2.value(), std::end(points), 0.0f,
										[](const float acu, const auto& p) {return acu + p.distance2d;});
	back_avg_distance /= (3.f*static_cast<float>(points.size())/4.f);
	return front_avg_distance > back_avg_distance + 1000.f; // big difference
}


//=========================================================================================================================================
std::optional<RoboCompLidar3D::TPoints> SpecificWorker::read_data()
{
	// Try-Catch block to read the laser data
	RoboCompLidar3D::TData data;
	try
	{ data = lidar3d_proxy->getLidarDataWithThreshold2d("pearl", 15000, 2);
	} catch(const Ice::Exception& ex){std::cout << ex.what() << std::endl; return {};}

	// filter
	//qInfo() << "full" << data.points.size();
	if (data.points.empty())
	{
		qWarning() << "No points received";
		return {};
	}
	RoboCompLidar3D::TPoints filter_data;
	if (const auto &filter_data_ = filter_min_distance_cppitertools(data.points);  filter_data_.has_value())
		filter_data = filter_data_.value();
	else
	{
		qWarning() << "No points filtered";
		return {};
	}

	// qInfo() << filter_data.size();


	//return filter_data;
	return filter_isolated_points(filter_data, 200);
}

std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d)
{
    if (points.empty()) return {};

    const float d_squared = d*d; // avoid sqrt by comparing squared distances
    std::vector<bool> hasNeighbor(points.size(), false);

    // Create index vector for parallel iteration
    std::vector<size_t> indices(points.size());
    std::iota(indices.begin(), indices.end(), size_t{0});

    // Parallelize outer loop - each thread checks one point
    std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i){
        const auto& p1 = points[i];
        // Sequential inner loop (avoid nested parallelism)
        for(auto &&[j,p2] : iter::enumerate(points))
        {
            if (i == j) continue;
            const float dx = p1.x - p2.x;
            const float dy = p1.y - p2.y;
            if (dx * dx + dy * dy <= d_squared)
            {
                hasNeighbor[i] = true;
                break;
            }
        }
    });

    // Collect result
    std::vector<RoboCompLidar3D::TPoint> result;
    result.reserve(points.size());
    for(auto &&[i,p] : iter::enumerate(points))
        if (hasNeighbor[i])
            result.push_back(points[i]);
    return result;
}


std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points)
{
	// non-empty condition
	if (points.empty())
		return {};

	RoboCompLidar3D::TPoints result; result.reserve(points.size());

	// 3. Loop over the groups produced by iter::groupby
	for (auto&& [angle, group] : iter::groupby(points, [](const auto& p)
		{float multiplier = std::pow(10.0f, 2); return std::floor(p.phi * multiplier) / multiplier; }))
	{
		// 'group' is an iterable object containing all Points for the current angle.
		auto min_it = std::min_element(std::begin(group), std::end(group),
			[](const auto& a, const auto& b) { return a.r < b.r; });
		// if (min_it->z > 300)
			result.emplace_back(*min_it); // Push the element with the minimum distance
	}

	return result;
}

std::expected<int, std::string> SpecificWorker::closest_lidar_index_to_given_angle(const auto &points, float angle)
{
	// search for the point in points whose phi value is closest to angle
	auto res = std::ranges::find_if(points, [angle](auto &a){ return a.phi > angle;});
	if(res != std::end(points))
		return std::distance(std::begin(points), res);
	else
		return std::unexpected("No closest value found in method <closest_lidar_index_to_given_angle>");
}


void SpecificWorker::draw_lidar(auto &filtered_points, Eigen::Vector2d room_center, QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

    // remove all items drawn in the previous iteration
    for(auto i: items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    auto color = QColor(Qt::green);
    auto brush = QBrush(QColor(Qt::green));
    for(auto &&[i, p] : filtered_points | iter::enumerate)
    {
    	brush = QColor(Qt::green);
    	auto item = scene->addRect(-50, -50, 100, 100, color, brush);
        item->setPos(p.x, p.y);
        items.push_back(item);
    	// qInfo() << std::hypot(p.x, p.y) << p.distance2d << p.r;
    }

    // compute and draw minimum distance point in frontal range
    auto offset_begin = closest_lidar_index_to_given_angle(filtered_points, -LidarAngles::FRONT_VISION);
    auto offset_end = closest_lidar_index_to_given_angle(filtered_points, LidarAngles::FRONT_VISION);
    if(not offset_begin or not offset_end)
    { std::cout << offset_begin.error() << " " << offset_end.error() << std::endl; return ;}    // abandon the ship
    auto min_point = std::min_element(std::begin(filtered_points) + offset_begin.value(), std::begin(filtered_points) + offset_end.value(), [](auto &a, auto &b)
    { return a.distance2d < b.distance2d; });
    QColor dcolor;
    if(min_point->distance2d < MIN_THRESHOLD)
        dcolor = QColor(Qt::red);
    else
        dcolor = QColor(Qt::magenta);
    auto ditem = scene->addRect(-100, -100, 200, 200, dcolor, QBrush(dcolor));
    ditem->setPos(min_point->x, min_point->y);
    items.push_back(ditem);

    // compute and draw minimum distance point to wall
    auto wall_res_right = closest_lidar_index_to_given_angle(filtered_points, LidarAngles::RIGHT);
    auto wall_res_left = closest_lidar_index_to_given_angle(filtered_points, LidarAngles::LEFT);
    if(not wall_res_right or not wall_res_left)   // abandon the ship
    {
        qWarning() << "No valid lateral readings" << QString::fromStdString(wall_res_right.error()) << QString::fromStdString(wall_res_left.error());
        return;
    }
    auto right_point = filtered_points[wall_res_right.value()];
    auto left_point = filtered_points[wall_res_left.value()];
	//qInfo() << "R: " << right_point.phi << " - L: " << left_point.phi;
    // compare both to get the one with minimum distance
    auto min_obj = (right_point.distance2d < left_point.distance2d) ? right_point : left_point;
	// std::cout << right_point.r << " " << left_point.r << " " << wall_res_right.value() << " " << wall_res_left.value() << " Size: " << filtered_points.size() << std::endl;
    auto item = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::orange), QBrush(QColor(QColorConstants::Svg::orange)));
	item->setPos(min_obj.x, min_obj.y);
    items.push_back(item);
	//qInfo() << "Indice wall_res_right" << wall_res_right.value() << right_point.r << right_point.x << right_point.y;
    // draw a line from the robot to the minimum distance point
    auto item_line = scene->addLine(QLineF(QPointF(0.f, 0.f), QPointF(min_obj.x, min_obj.y)), QPen(QColorConstants::Svg::orange, 10));
    items.push_back(item_line);

	// Draw min side point
	bool left_side = (left_point.distance2d < right_point.distance2d);
	float angle_side_begin = left_side ? LidarAngles::BACK_LEFT : -LidarAngles::FRONT_LEFT;
	float angle_side_end = left_side ? LidarAngles::FRONT_LEFT : -LidarAngles::BACK_LEFT;
	auto side_begin_offset = closest_lidar_index_to_given_angle(filtered_points, angle_side_begin);
	auto side_end_offset = closest_lidar_index_to_given_angle(filtered_points, angle_side_end);
	if (not side_begin_offset or not side_end_offset){std::cout << side_begin_offset.error() << "" << side_end_offset.error() << std::endl; return;}
	auto min_side_point = std::min_element(std::begin(filtered_points) + side_begin_offset.value(), std::begin(filtered_points) + side_end_offset.value(),
										   [](const auto& p1, const auto& p2){return p1.distance2d < p2.distance2d;});
	auto side_point_item = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::black), QBrush(QColorConstants::Svg::black));
	side_point_item->setPos(min_side_point->x, min_side_point->y);
	items.push_back(side_point_item);

    // Draw two lines coming out from the robot at angles given by params.LIDAR_OFFSET
    // Calculate the end points of the lines
    auto res_right = closest_lidar_index_to_given_angle(filtered_points, LidarAngles::FRONT_VISION);
    auto res_left = closest_lidar_index_to_given_angle(filtered_points, -LidarAngles::FRONT_VISION);
    if(not res_right or not res_left)
    { std::cout << res_right.error() << " " << res_left.error() << std::endl; return ;}
    // draw two lines at the edges of the range
    float right_line_length = filtered_points[res_right.value()].distance2d;
    float left_line_length = filtered_points[res_left.value()].distance2d;
    float angle1 = filtered_points[res_left.value()].phi;
    float angle2 = filtered_points[res_right.value()].phi;
    QLineF line_left{QPointF(0.f, 0.f),
                     robot_draw->mapToScene(left_line_length * sin(angle1), left_line_length * cos(angle1))};
    QLineF line_right{QPointF(0.f, 0.f),
                      robot_draw->mapToScene(right_line_length * sin(angle2), right_line_length * cos(angle2))};
    QPen left_pen(Qt::blue, 10); // Blue color pen with thickness 3
    QPen right_pen(Qt::red, 10); // Blue color pen with thickness 3
    auto line1 = scene->addLine(line_left, left_pen);
    auto line2 = scene->addLine(line_right, right_pen);
    items.push_back(line1);
    items.push_back(line2);

	//draw center point
	auto center = scene->addRect(room_center.x(), room_center.y(), 200, 200, QColor(QColorConstants::Svg::red), QBrush(QColorConstants::Svg::red));
	items.push_back(center);
}


void SpecificWorker::new_target_slot(QPointF point)
{
	qDebug() << "Slot new_target_slot llamado con el punto:" << point;
}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}



//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}




void SpecificWorker::update_report_posotion()
{
	try
	{
		RoboCompGenericBase::TBaseState bState;
		omnirobot_proxy->getBaseState(bState);
		robot_draw->setRotation(bState.alpha*180/M_PI);
		robot_draw->setPos(bState.x, bState.z);
		std::cout << bState.alpha << " " << bState.x << " " << bState.z << std::endl;
	}
	catch (const Ice::Exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}


/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->resetOdometer()
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setSpeedBase(float adv, float rot)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->stopBase()

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// RoboCompLaser::TLaserData this->laser_proxy->getLaserAndBStateData(RoboCompGenericBase::TBaseState bState)
// RoboCompLaser::LaserConfData this->laser_proxy->getLaserConfData()
// RoboCompLaser::TLaserData this->laser_proxy->getLaserData()

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

