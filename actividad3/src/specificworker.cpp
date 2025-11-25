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
		auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_draw = r;
		//this->resize(900, 450);

		viewer_room = new AbstractGraphicViewer(this->frame_room, this->dimensions);
		auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
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
	if ( const auto filter_data = read_data(); not filter_data.has_value())
	{ std::cerr << "No filter data found" << std::endl; return;}
	else points = filter_data.value();
	const auto &[filtered_points, doors_] = door_detector.filter_points(points, &viewer->scene);
	doors = doors_;
	//draw_lidar(points, &viewer->scene);

	// compute corners
	const auto &[corners, lines] = room_detector.compute_corners(filtered_points, &viewer->scene);
	const auto center_opt = room_detector.estimate_center_from_walls(lines);
	if (center_opt.has_value())
		draw_lidar(filtered_points, center_opt.value(), &viewer->scene);
	else
	{ std::cerr << "No center room point found" << std::endl; return;}

	 // match corners transforming first nominal corners to robot's frame
	 const auto match = hungarian.match(corners, nominal_rooms[room_index].transform_corners_to(robot_pose.inverse()));

	 // compute max of match error
	 float max_match_error = 99999.f;
	 if (not match.empty())
	 {
	 	const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
	 		{ return std::get<2>(a) < std::get<2>(b); });
	 	max_match_error = static_cast<float>(std::get<2>(*max_error_iter));
	 	time_series_plotter->addDataPoint(0,max_match_error);
	 	//print_match(match, max_match_error); //debugging
	 }

	 // update robot pose
	 if (localised)
	 	update_robot_pose(corners, match);

	 // Process state machine
	RetVal ret_val = state_machine(filtered_points, match, corners, lines, door_center, max_match_error);

	 auto [st, adv, rot] = ret_val;
	 state = st;

	//Send movements commands to the robot constrained by the match_error
	const auto &[_, __, angle_] = do_work(center_opt.value());
	qInfo() << __FUNCTION__ << to_string(state) << " Adv: " << adv << " Rot: " << rot << " Center: " << center_opt.value().norm() << " Angle_to_center: " << angle_;
	move_robot(adv, rot, max_match_error);

	//draw robot in viewer
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
	label_state_name->setText(to_string(state));
	label_state->setText(QString::number(room_index));
	// auto result = state_Machine(filter_data);
}

SpecificWorker::RetVal SpecificWorker::state_machine(const RoboCompLidar3D::TPoints &points, const Match &match, const Corners &corners, const Lines &lines, const Eigen::Vector2d &door_center, const float &max_match_error)
{
	RetVal res;
	switch (state)
	{
	case STATE::GOTO_ROOM_CENTER:
		res = goto_room_center(lines);
		break;
	case STATE::TURN:
		res = turn(corners);
		break;
	case STATE::GOTO_DOOR:
		res = goto_door(points, max_match_error);
		break;
	case STATE::ORIENT_TO_DOOR:
		res = orient_to_door(door_center, max_match_error);
		break;
	case STATE::CROSS_DOOR:
		res = cross_door(points);
		break;
	}
	return res;
}


SpecificWorker::RetVal SpecificWorker::localise(const Match &match)
{


	return{};
}

std::tuple<float, float, double> SpecificWorker::do_work(const Eigen::Vector2d target)
{
	// rotation
	auto angle = atan2(target.x(), target.y());
	double k = 0.5f;
	double rot_vel = angle * k;

	// break rotation
	const double R = std::log(0.2) / -M_PI_4*M_PI_4;
	auto break_rot = std::exp(-angle*angle * R);

	// advance
	double adv_vel = params.MAX_ADV_SPEED * break_rot;
	return {adv_vel, rot_vel, angle};
}

void SpecificWorker::draw_target(const Eigen::Vector2d &target, QGraphicsScene *scene)
{
	static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

	// remove all items drawn in the previous iteration
	for(auto i: items)
	{
		scene->removeItem(i);
		delete i;
	}
	items.clear();

	auto item = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::magenta), QBrush(QColorConstants::Svg::magenta));
	item->setPos(target.x(), target.y());
	items.push_back(item);
}

SpecificWorker::RetVal SpecificWorker::goto_room_center(const Lines &lines)
{
	auto center = room_detector.estimate_center_from_walls(lines);
	if (not center.has_value()) return {};

	if (center.value().norm() < 300.f)
		return {STATE::TURN, 0, 0};

	const auto &[adv_vel, rot_vel, _] = do_work(center.value());
	return {STATE::GOTO_ROOM_CENTER, adv_vel, rot_vel};
}

SpecificWorker::RetVal SpecificWorker::turn(const Corners &corners)
{
	if (const auto &[success, giro] = image_processor.check_colour_patch_in_image(camera360rgb_proxy, colors[room_index] , label_img); success)
	{
		// localised = ... localise(..)
		// if (localised) return {..}
		// else return {..}

		localised = true;
		return {STATE::GOTO_DOOR, 0, 0};
	}
	else
	{
		float vel_rot = 0.4f;
		return{STATE::TURN, 0, giro * vel_rot};
	}
}

SpecificWorker::RetVal SpecificWorker::goto_door(const RoboCompLidar3D::TPoints &points, const float &max_match_error)
{
	//if (max_match_error > params.RELOCAL_DONE_MATCH_MAX_ERROR)
	//{
	//	localised = false;
//		return {STATE::GOTO_ROOM_CENTER, 0, 0};
//	}
	if (doors.empty()) {std::cerr << "GOTO_DOOR: DOORS EMPTY"; return {};}
	Eigen::Vector2d door_center_2d(doors[0].center().x(), doors[0].center().y());
	door_center = door_center_2d;
	const auto door_center_point = doors[0].center();
	Eigen::Vector2d robot_position(robot_pose.translation().x(), robot_pose.translation().y());
	const auto target = doors[0].center_before(robot_position, 650.f);
	Eigen::Vector2d target_2d(target.x(), target.y());
	draw_target(target_2d, &viewer->scene);
	if (target.norm() < 300.f)
		return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};

	const auto &[adv_speed, rot_speed, _] = do_work(target_2d);
	return{STATE::GOTO_DOOR, adv_speed, rot_speed};
}

SpecificWorker::RetVal SpecificWorker::orient_to_door(const Eigen::Vector2d &target, const float &max_match_error)
{
//	if (max_match_error > params.RELOCAL_DONE_MATCH_MAX_ERROR)
//	{
//		localised = false;
//		return {STATE::GOTO_ROOM_CENTER, 0, 0};
//	}
	const auto &[_, rot_vel, angle] = do_work(target);
	if (std::abs(rot_vel) < 0.15f)
	{
		return {STATE::CROSS_DOOR, 0.f, 0.f};
	}
	return{STATE::ORIENT_TO_DOOR, 0.f, rot_vel};
}



SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points)
{
	static std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();
	static bool first_time = true;
	if (first_time) {first_time = false; last_time = std::chrono::high_resolution_clock::now();}
	auto now = std::chrono::high_resolution_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
	if (elapsed > 2000)
	{
		first_time = true;
		localised = false;
		room_index = (room_index+1) % std::size(nominal_rooms);
		return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
	}
	return{STATE::CROSS_DOOR, 700.f, 0.f};
}

SpecificWorker::RetVal SpecificWorker::update_pose(const Corners &corners, const Match &match)
{
	return{};
}

SpecificWorker::RetVal SpecificWorker::process_state(const RoboCompLidar3D::TPoints &data, const Corners &corners, const Match &match, AbstractGraphicViewer *viewer)
{
	return{};
}

bool SpecificWorker::update_robot_pose(const Corners &corners, const Match &match)
{
	Eigen::MatrixXd W(match.size()*2,3);
	Eigen::VectorXd b(match.size()*2);
	for (const auto &&[i, m] : match | iter::enumerate)
	{
		auto &[meas_c, nom_c, _] = m;
		auto &[p_meas, __, ___] = meas_c;
		auto &[p_nom, ____, _____] = nom_c;
		b(2*i) = p_nom.x() - p_meas.x();
		b(2*i+1) = p_nom.y() - p_meas.y();
		W.block<1, 3>(2*i, 0) << 1.0, 0.0, -p_meas.y();
		W.block<1, 3>(2*i+1, 0) << 0.0, 1.0, p_meas.x();
	}
	// estimate new pose with pseudoinverse
	const auto r = (W.transpose()*W).inverse()*W.transpose()*b;
	if (r.array().isNaN().any()) return false;
	// update robot pose
	robot_pose.translate(Eigen::Vector2d(r(0), r(1)));
	robot_pose.rotate(r(2));

	return true;
}
void SpecificWorker::move_robot(float adv, float rot, float max_match_error)
{
	omnirobot_proxy->setSpeedBase(0, adv, rot);
}
Eigen::Vector3d SpecificWorker::solve_pose(const Corners &corners, const Match &match){return {};}
void SpecificWorker::predict_robot_pose(){}
std::tuple<float, float> SpecificWorker::robot_controller(const Eigen::Vector2f &target){return {};}

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
    auto offset_begin = closest_lidar_index_to_given_angle(filtered_points, -params.LIDAR_FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_FRONT_SECTION);
    if(not offset_begin or not offset_end)
    { std::cout << offset_begin.error() << " " << offset_end.error() << std::endl; return ;}    // abandon the ship
    auto min_point = std::min_element(std::begin(filtered_points) + offset_begin.value(), std::begin(filtered_points) + offset_end.value(), [](auto &a, auto &b)
    { return a.distance2d < b.distance2d; });
    QColor dcolor;
    if(min_point->distance2d < params.STOP_THRESHOLD)
        dcolor = QColor(Qt::red);
    else
        dcolor = QColor(Qt::magenta);
    auto ditem = scene->addRect(-100, -100, 200, 200, dcolor, QBrush(dcolor));
    ditem->setPos(min_point->x, min_point->y);
    items.push_back(ditem);

    // compute and draw minimum distance point to wall
    auto wall_res_right = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_RIGHT_SIDE_SECTION);
    auto wall_res_left = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_LEFT_SIDE_SECTION);
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


    // Draw two lines coming out from the robot at angles given by params.LIDAR_OFFSET
    // Calculate the end points of the lines
    auto res_right = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_FRONT_SECTION);
    auto res_left = closest_lidar_index_to_given_angle(filtered_points, -params.LIDAR_FRONT_SECTION);
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

