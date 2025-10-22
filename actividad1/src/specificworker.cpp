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

#ifdef emit
#undef emit
#endif
#include <execution>
#include <expected>
#include <algorithm>
#include <cppitertools/enumerate.hpp>


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

	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	auto [r, e] = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 100, QColor("Blue"));
    robot_polygon = r;
    this->resize(900, 450);
	viewer->show();
	const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);
	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
	rotating = false;

    //initializeCODE

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

}



void SpecificWorker::compute()
{
	auto filter_data = read_data();
	auto result = std::tuple<SpecificWorker::State, float, float>();

	if (not filter_data.has_value())
	{
		std::cerr << "No filter data found" << std::endl;
		return;
	}
	auto i = closest_lidar_index_to_given_angle(filter_data.value(), qDegreesToRadians(90)).value();
	//qInfo() << filter_data.value().size() << i << filter_data.value()[i].r;

	// State machine
	auto begin = closest_lidar_index_to_given_angle(filter_data.value(),  -LidarAngles::FRONT_VISION);
	auto end = closest_lidar_index_to_given_angle(filter_data.value(), LidarAngles::FRONT_VISION);
	if (not begin or not end){std::cout << begin.error() << " " << end.error() << std::endl; return;}
	auto min = std::min_element(std::begin(filter_data.value()) + begin.value(),  std::begin(filter_data.value()) + end.value(),
								[](const auto& p1, const auto& p2){return p1.distance2d < p2.distance2d;});

	if (!rotating)
		rot = (min->phi < 0.f) ? 0.6f : -0.6f; // -phi turn to right (-rot) - phi turn to left (rot)

	switch (state)
	{
		case SpecificWorker::State::IDLE:
			break;

		case SpecificWorker::State::FORWARD:
			//qInfo() << "State: FORWARD";
			result = forward(min);
		break;

		case SpecificWorker::State::TURN:
			//qInfo() << "State: TURN";
	        result = turn(min);
	        break;

		case State::FOLLOW_WALL:
			// qInfo() << "State: FOLLOW_WALL";
			result = follow_wall(filter_data.value());
			break;
		//case State::SPIRAL:

		default: break;
	}
	state = std::get<SpecificWorker::State>(result);

	// Try-Catch block to send velocities to the robot
	omnirobot_proxy->setSpeedBase(0, std::get<1>(result), std::get<2>(result));

}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::follow_wall(auto &points)
{
	// Switch state condition
	auto begin_offset = closest_lidar_index_to_given_angle(points, -LidarAngles::FRONT_VISION_FW);
	auto end_offset = closest_lidar_index_to_given_angle(points, LidarAngles::FRONT_VISION_FW);
	if (not begin_offset or not end_offset){std::cout << begin_offset.error() << " " << end_offset.error() << std::endl; return {};}
	auto min_point = std::min_element(std::begin(points) + begin_offset.value(), std::begin(points) + end_offset.value(),
									  [](const auto& p1, const auto& p2) {return p1.distance2d < p2.distance2d;});
	if (min_point->distance2d <= MIN_THRESHOLD)
		return std::make_tuple(SpecificWorker::State::TURN, 0.f, 0.f);

	// Lateral check: ¿right or left?
	auto left = closest_lidar_index_to_given_angle(points, LidarAngles::LEFT);
	auto right = closest_lidar_index_to_given_angle(points, LidarAngles::RIGHT);
	if (not left or not right){std::cout << left.error() << " " << right.error() << std::endl; return {};}
	auto left_point = points[left.value()];
	auto right_point = points[right.value()];
	bool left_side = (left_point.distance2d < right_point.distance2d);
	auto angle_side_begin_offset = closest_lidar_index_to_given_angle(points.value(), (left_side) ? LidarAngles::FRONT_LEFT : -LidarAngles::BACK_LEFT);
	auto angle_side_end_offset = closest_lidar_index_to_given_angle(points.value(), (left_side) ? LidarAngles::BACK_LEFT : -LidarAngles::FRONT_LEFT);

	auto side_begin_offset = closest_lidar_index_to_given_angle(points, angle_side_begin_offset.value());
	auto side_end_offset = closest_lidar_index_to_given_angle(points, angle_side_end_offset.value());
	if (not side_begin_offset or not side_end_offset){std::cout << side_begin_offset.error() << "" << side_end_offset.error() << std::endl; return {};}
	auto min_side_point = std::min_element(std::begin(points) + side_begin_offset.value(), std::begin(points) + side_end_offset.value(),
											  [](const auto& p1, const auto& p2){return p1.distance2d < p2.distance2d;});

	// (p.phi < M_PI_2 + 0.01 and p.phi > M_PI_2 - 0.01)
	rot = (left_side && min_side_point->phi < angle_to_compare) ? 0.05f: -0.05f;
	if (min_side_point->phi < angle_to_compare+0.01f and angle_to_compare-0.01f)
		return std::make_tuple(SpecificWorker::State::FOLLOW_WALL, 600.f, 0.f);
	return std::make_tuple(SpecificWorker::State::FOLLOW_WALL, 50.f, rot);
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::forward(const __gnu_cxx::__normal_iterator<RoboCompLidar3D::TPoint*, std::vector<RoboCompLidar3D::TPoint>> &point)
{
	//  && point->phi == std::clamp(point->phi, -LidarAngles::FRONT_VISION, LidarAngles::FRONT_VISION)
	if (point->distance2d <= MIN_THRESHOLD)
			return std::make_tuple(SpecificWorker::State::TURN, 0.f, 0.f);
	return std::make_tuple(SpecificWorker::State::FOLLOW_WALL, 1000.f, 0.f);
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::turn(const __gnu_cxx::__normal_iterator<RoboCompLidar3D::TPoint*, std::vector<RoboCompLidar3D::TPoint>> &point)
{
	// point->phi != std::clamp(point->phi, -LidarAngles::FRONT_VISION, LidarAngles::FRONT_VISION)
	if (point->distance2d > MIN_THRESHOLD) {
		rotating = false;
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_int_distribution<int> dice(1, 2);
		auto _state = (dice(gen) == 1) ? SpecificWorker::State::FORWARD : SpecificWorker::State::FOLLOW_WALL;
		return std::make_tuple(SpecificWorker::State::FORWARD, 0.f, 0.f);
	}
	rotating = true;
	return std::make_tuple(SpecificWorker::State::TURN, 0.f, rot);
}


//=========================================================================================================================================
std::optional<RoboCompLidar3D::TPoints> SpecificWorker::read_data()
{
	// Try-Catch block to read the laser data
	RoboCompLidar3D::TData data;
	try
	{ data = lidar3d_proxy->getLidarDataWithThreshold2d("pearl", 15000, 1);
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

	// draw_lidar(filter_data, &viewer->scene);
	draw_lidar(data.points, &viewer->scene);
	return filter_data;
	//return data.points;
	//return filter_isolated_points(filter_data, 200);
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

void SpecificWorker::send_velocities(std::tuple<SpecificWorker::State, float, float> state)
{

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


void SpecificWorker::draw_lidar(auto &filtered_points, QGraphicsScene *scene)
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
    	 if (p.phi < M_PI_2 + 0.01 and p.phi > M_PI_2 - 0.01)
    	 {
     		brush = QColor(Qt::red);
    	 	auto j = closest_lidar_index_to_given_angle(filtered_points, M_PI_2).value();
	    	qInfo() << "In for" << filtered_points[i].r << std::hypot(p.x, p.y) << p.phi << i << j <<
	    		p.x << p.y << filtered_points[i].x << filtered_points[i].y << filtered_points[j].x <<
	    			filtered_points[j].y << filtered_points[j].phi;
    	 }
		else
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
    // compare both to get the one with minimum distance
    auto min_obj = (right_point.distance2d < left_point.distance2d) ? right_point : left_point;
	std::cout << right_point.r << " " << left_point.r << " " << wall_res_right.value() << " " << wall_res_left.value() << " Size: " << filtered_points.size() << std::endl;
    auto item = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::orange), QBrush(QColor(QColorConstants::Svg::orange)));
	item->setPos(min_obj.x, min_obj.y);
    items.push_back(item);
	qInfo() << "Indice wall_res_right" << wall_res_right.value() << right_point.r << right_point.x << right_point.y;
    // draw a line from the robot to the minimum distance point
    auto item_line = scene->addLine(QLineF(QPointF(0.f, 0.f), QPointF(min_obj.x, min_obj.y)), QPen(QColorConstants::Svg::orange, 10));
    items.push_back(item_line);

	//////
	bool left_side = (left_point.distance2d < right_point.distance2d);
	const float angle_to_compare = (left_side) ? LidarAngles::LEFT : LidarAngles::RIGHT;
	auto side_begin_offset = closest_lidar_index_to_given_angle(filtered_points, );
	auto side_end_offset = closest_lidar_index_to_given_angle(filtered_points, );
	if (not side_begin_offset or not side_end_offset){std::cout << side_begin_offset.error() << "" << side_end_offset.error() << std::endl; return;}
	auto min_side_point = std::min_element(std::begin(filtered_points) + side_begin_offset.value(), std::begin(filtered_points) + side_end_offset.value(),
											  [](const auto& p1, const auto& p2){return p1.distance2d < p2.distance2d;});
	///////
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
                     robot_polygon->mapToScene(left_line_length * sin(angle1), left_line_length * cos(angle1))};
    QLineF line_right{QPointF(0.f, 0.f),
                      robot_polygon->mapToScene(right_line_length * sin(angle2), right_line_length * cos(angle2))};
    QPen left_pen(Qt::blue, 10); // Blue color pen with thickness 3
    QPen right_pen(Qt::red, 10); // Blue color pen with thickness 3
    auto line1 = scene->addLine(line_left, left_pen);
    auto line2 = scene->addLine(line_right, right_pen);
    items.push_back(line1);
    items.push_back(line2);
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
		robot_polygon->setRotation(bState.alpha*180/M_PI);
		robot_polygon->setPos(bState.x, bState.z);
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

