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

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> dice(1, 2);

	// State machine
	auto begin = closest_lidar_index_to_given_angle(filter_data.value(),  -LidarAngles::FRONT_VISION);
	auto end = closest_lidar_index_to_given_angle(filter_data.value(), LidarAngles::FRONT_VISION);
	if (not begin or not end){std::cout << begin.error() << " " << end.error() << std::endl; return;}
	auto min = std::min_element(std::begin(filter_data.value()) + begin.value(),  std::begin(filter_data.value()) + end.value(),
								[](const auto& p1, const auto& p2){return p1.distance2d < p2.distance2d;});


	if (!rotating)
		dir = (min->phi < 0.f) ? 0.6f : -0.6f;
	// if (min->phi == 0.f) dir = dice(gen);
	// Local min
	//auto min_izq = std::min_element(std::begin(filter_data.value()) + 3*filter_data->size()/8,  std::begin(filter_data.value()) + 4*filter_data->size()/8,
	//							[](const auto& p1, const auto& p2) {return p1.distance2d < p2.distance2d;});
	//auto min_der = std::min_element(std::begin(filter_data.value()) + 4*filter_data->size()/8,  std::begin(filter_data.value()) + 5*filter_data->size()/8,
	//							[](const auto& p1, const auto& p2) {return p1.distance2d < p2.distance2d;});

	// qInfo() << "DIS2D de MIN: " << min->distance2d;
	switch (state)
	{
		case SpecificWorker::State::IDLE:
			break;

		case SpecificWorker::State::FORWARD:
			result = forward(min);
		break;

		case SpecificWorker::State::TURN:
	        result = turn(min);
	        break;

		//case State::FOLLOW_WALL: {}

		//case State::SPIRAL: {}

		default: break;
	}
	state = std::get<SpecificWorker::State>(result);

	// Try-Catch block to send velocities to the robot
	omnirobot_proxy->setSpeedBase(0, std::get<1>(result), std::get<2>(result));

}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::forward(const __gnu_cxx::__normal_iterator<RoboCompLidar3D::TPoint*, std::vector<RoboCompLidar3D::TPoint>> &point)
{
	if (point->distance2d <= MIN_THRESHOLD && point->phi == std::clamp(point->phi, -LidarAngles::FRONT_VISION, LidarAngles::FRONT_VISION))
			return std::tuple<SpecificWorker::State, float, float>(SpecificWorker::State::TURN, 0.f, 0.f);
	return std::tuple<SpecificWorker::State, float, float>(SpecificWorker::State::FORWARD, 1000.f, 0.f);
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::turn(const __gnu_cxx::__normal_iterator<RoboCompLidar3D::TPoint*, std::vector<RoboCompLidar3D::TPoint>> &point)
{
	if (point->phi != std::clamp(point->phi, -LidarAngles::FRONT_VISION, LidarAngles::FRONT_VISION)) {
			rotating = false;
			return std::tuple<SpecificWorker::State, float, float>(SpecificWorker::State::FORWARD, 0.f, 0.f);
		}
	rotating = true;
	return std::tuple<SpecificWorker::State, float, float>(SpecificWorker::State::TURN, 0.f, dir);
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

	draw_lidar(filter_data, &viewer->scene);
	return filter_data;
	// return filter_isolated_points(filter_data, 200);
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


void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene* scene)
{
	static std::vector<QGraphicsItem*> draw_points;
	for (const auto &p : draw_points)
	{
		scene->removeItem(p);
		delete p;
	}
	draw_points.clear();

	const QColor color("LightGreen");
	const QPen pen(color, 10);
	// const QBrush brush(color, Qt::SolidPattern);
	for (const auto &p : points)
	{
		const auto dp = scene->addRect(-25, -25, 50, 50, pen);
		dp->setPos(p.x, p.y);
		draw_points.push_back(dp); // add to the list of points to be deleted next time
	}

	// draw min point
	auto begin = closest_lidar_index_to_given_angle(points,  -LidarAngles::FRONT_VISION);
	auto end = closest_lidar_index_to_given_angle(points, LidarAngles::FRONT_VISION);
	if (not begin or not end){std::cout << begin.error() << " " << end.error() << std::endl; return;}
	auto middle = points[closest_lidar_index_to_given_angle(points, LidarAngles::FRONT).value()];
	auto min = std::min_element(std::begin(points) + begin.value(),  std::begin(points) + end.value(),
	                            [](const auto& p1, const auto& p2){return p1.distance2d < p2.distance2d;});
	qInfo() << "DIS2D: " << min->distance2d << " PHI: " << min->phi << " r: " << min->r << " z: " << min->z;
	QColor dcolor;
	if (min->distance2d < MIN_THRESHOLD){
		// qInfo() << "MIN detected! - DIS2D: " << min->distance2d << " r: " << min->r << " z: " << min->z;
		dcolor = QColor(Qt::red);}
	else
		dcolor = QColor(Qt::magenta);


	auto ditem = scene->addRect(-100, -100, 200, 200, dcolor, QBrush(dcolor));
	auto mitem = scene->addRect(-100, -100, 200, 200, QColor(Qt::black), QColor(Qt::black));
	ditem->setPos(min->x, min->y);
	mitem->setPos(middle.x, middle.y);
	draw_points.push_back(ditem);
	draw_points.push_back(mitem);
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

