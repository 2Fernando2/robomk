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

struct termios SpecificWorker::original_terminal;

SpecificWorker::SpecificWorker(const ConfigLoader &configLoader, TuplePrx tprx, bool startup_check)
        : GenericWorker(configLoader, tprx) {
    this->startup_check_flag = startup_check;
    if (this->startup_check_flag) {
        this->startup_check();
    } else {
#ifdef HIBERNATION_ENABLED
        hibernationChecker.start(500);
#endif

        statemachine.setChildMode(QState::ExclusiveStates);
        statemachine.start();

        auto error = statemachine.errorString();
        if (error.length() > 0) {
            qWarning() << error;
            throw error;
        }
    }
}

SpecificWorker::~SpecificWorker() {
    qInfo() << "MaxErrorRegistered: " << max_error_registered;
    restore_blocking_mode();
    move_robot(0.f, 0.f, 0.f);
    std::cout << "Destroying SpecificWorker" << std::endl;
}

void SpecificWorker::initialize() {
    std::cout << "initialize worker" << std::endl;
    set_non_blocking_mode();

    if (this->startup_check_flag) {
        this->startup_check();
    } else {
        // Viewer
        if (params.VERBOSE) qInfo() << "Creating viewer - frame 1.";
        viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
        auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0,
                                        100, QColor("Blue"));
        robot_draw = r;
        // this->resize(900, 450);

        if (params.VERBOSE) qInfo() << "Creating viewer room - frame 2.";
        viewer_room = new AbstractGraphicViewer(this->frame_room, this->dimensions);
        auto [rr, re] = viewer_room->add_robot(
                params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_room_draw = rr;

        // draw room in viewer_room
        // room_draw = viewer_room->scene.addRect(nominal_rooms[0].rect(), QPen(Qt::black, 30));

        // viwer_room->show();
        show();

        // init robot pose
        robot_pose.setIdentity();
        robot_pose.translate(Eigen::Vector2f(0.0, 0.0));

        // time series plotter for match error
        TimeSeriesPlotter::Config plotConfig;
        plotConfig.title = "Maximun Match Error Over Time";
        plotConfig.yAxisLabel = "Error (mm)";
        plotConfig.timeWindowSeconds = 15.0; // Show a 15-second window
        plotConfig.autoScaleY = false;       // We will set a fixed range
        plotConfig.yMin = 0;
        plotConfig.yMax = 2000;

        time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, plotConfig);
        match_error_graph = time_series_plotter->addGraph("", Qt::blue);

        // stop robot
        // move_robot(0, 0, 0);
    }
}

void SpecificWorker::compute() {

    // read lidar data and filter points.
    RoboCompLidar3D::TPoints points;
    if (const auto filter_data = read_data(); not filter_data.has_value()) {
        std::cerr << "No filter data found" << std::endl; return; }
    else points = filter_data.value();
    // draw_local_doors(&viewer->scene);

    std::vector<Eigen::Vector2d> points_eigen;
    points_eigen.reserve(points.size());
    for (const auto &p : points)
        points_eigen.emplace_back(p.x, p.y);

    rc::RansacLineDetector::Params ransac_params;
    ransac_params.min_line_length = 500.0;    // cuanto debe medir como mínimo una pared
    ransac_params.distance_threshold = 50.0;  // grosor admisible de la pared
    ransac_params.min_points_per_line = 20;   // ruido mínimo
    ransac_params.max_lines = 10;             // número de paredes que se buscan

    std::vector<LineSegment> detected_lines = rc::RansacLineDetector::detect_lines(points_eigen, ransac_params);
    std::vector<bool> is_wall_index(points.size(), false);
    for (const auto &line : detected_lines) {
        for (int idx : line.inlier_indices) {
            if (idx >= 0 && idx < points.size()) {
                is_wall_index[idx] = true;
            }
        }
    }

    RoboCompLidar3D::TPoints wall_points;
    RoboCompLidar3D::TPoints obstacle_points;


    for (const auto &p : points) {
        // Ignorar suelo y techo
        if (p.z < 100 || p.z > 2000) continue;
        // RANSAC filter helper
        if (p.z > 1200 && p.z < 1300) wall_points.push_back(p);
        if (p.z > 200 && p.z < 600) obstacle_points.push_back(p);
    }

    for (size_t i = 0; i < points.size(); ++i) {
        if (is_wall_index[i]) wall_points.push_back(points[i]);
        else obstacle_points.push_back(points[i]);
    }

    draw_lidar(wall_points, obstacle_points, &viewer->scene);

    const auto center_opt = room_detector_estimator.estimate(wall_points);
    if (center_opt.has_value()) draw_target(center_opt.value(), &viewer->scene, false);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*
    // detect doors and filter beyond points -> doors from robot perspective (?)
    // const auto &filtered_points = door_detector.filter_points(points, &viewer->scene);
    // doors = door_detector.doors();

    // compute corners - detect and draw them
    // const auto &[corners, lines] = room_detector.compute_corners(filtered_points, &viewer->scene);
    // const auto center_opt = room_detector_estimator.estimate(filtered_points);
    // if (center_opt.has_value()) draw_lidar(filtered_points, center_opt.value(), &viewer->scene);
    // else { std::cerr << "No center room point found" << std::endl; return; }
    // const Eigen::Vector2f &center_point = Eigen::Vector2f(center_opt.value().x(), center_opt.value().y());

    // update robot pose
    float max_match_error = 99999.f;
    Match match;
    if (localised)
    {
        if (const auto res = update_robot_pose(corners, robot_pose, true); res.has_value())
        {
            robot_pose = res.value().first;
            max_match_error = res.value().second;
            time_series_plotter->addDataPoint(match_error_graph, max_match_error);
        }
    }

    // Process state machine
    // RetVal ret_val = state_machine(filtered_points, center_point, corners, max_match_error);
    //auto [st, adv, rot] = ret_val;
    float adv = 0;
    float rot = 0;
    // state = st;
    if (max_match_error < 99999.f)
        max_error_registered = std::max(max_error_registered, max_match_error);

    // Send movements commands to the robot constrained by the match_error
    // if (keyw_mode) keyw(); else move_robot(adv, rot, max_match_error);

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
    label_state_name->setText(to_string(state));
    label_state->setText(QString::number(current_room));
    if (localised) label_localised->setText("Localised");
    else label_localised->setText("Lost");
    */
}


SpecificWorker::RetVal SpecificWorker::state_machine(const RoboCompLidar3D::TPoints &points, const Eigen::Vector2f &center_point, const Corners &corners, const float &max_match_error) {
    RetVal res;
    switch (state) {
        case STATE::GOTO_ROOM_CENTER:
            res = goto_room_center(points);
            break;
        case STATE::TURN:
            res = turn(corners);
            break;
        case STATE::GOTO_DOOR:
            res = goto_door(points, center_point, &viewer->scene);
            break;
        case STATE::ORIENT_TO_DOOR:
            res = orient_to_door(max_match_error);
            break;
        case STATE::CROSS_DOOR:
            res = cross_door(points);
            break;
    }
    return res;
}


std::tuple<float, float, double> SpecificWorker::do_work(const Eigen::Vector2d target) {
    // rotation
    auto angle = atan2(target.x(), target.y());
    double k = 0.5f;
    double rot_vel = angle * k;

    // break rotation
    const double R = std::log(0.2) * 2 / -M_PI_4 * M_PI_4;
    auto break_rot = std::exp(-angle * angle * R);

    // advance
    double adv_vel = params.MAX_ADV_SPEED * break_rot;
    return {adv_vel, rot_vel, angle};
}

void SpecificWorker::draw_target(const Eigen::Vector2d &target, QGraphicsScene *scene, bool last_iteratior) {
    static std::vector<QGraphicsItem *> items; // store items so they can be shown between iterations

    // remove all items drawn in the previous iteration
    for (auto i : items) {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    if (last_iteratior) return;

    auto item = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::silver), QBrush(QColorConstants::Svg::silver));
    item->setPos(target.x(), target.y());
    items.push_back(item);
}

void SpecificWorker::draw_nominal_room()
{
    if(room_draw) { viewer_room->scene.removeItem(room_draw); delete room_draw; }
    room_draw = viewer_room->scene.addRect(nominal_rooms[current_room].rect(), QPen(Qt::black, 30));
}

void SpecificWorker::draw_nominal_doors(QGraphicsScene *scene) {
    static std::vector<QGraphicsItem *> items; // store items so they can be shown between iterations

    // remove all items drawn in the previous iteration
    for (auto i : items) {
        scene->removeItem(i); delete i; }
    items.clear();

    for (const auto &d : nominal_rooms[current_room].doors){
        auto p1_point = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::orange),QBrush(QColorConstants::Svg::orange));
        qInfo() << d.p1_global.x() << d.p1_global.y() << d.p2_global.x() << d.p2_global.y();
        p1_point->setPos(d.p1_global.x(), d.p1_global.y());
        items.push_back(p1_point);
        auto p2_point = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::orange),QBrush(QColorConstants::Svg::orange));
        p2_point->setPos(d.p2_global.x(), d.p2_global.y());
        items.push_back(p2_point);

        // draw line between p1 and p2
        auto item_line = scene->addLine(QLineF(QPointF(d.p1_global.x(), d.p1_global.y()), QPointF(d.p2_global.x(), d.p2_global.y())), QPen(QColorConstants::Svg::crimson, 10));
        items.push_back(item_line);
    }
}

SpecificWorker::RetVal SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints &points) {
    static std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
    static bool first_time = true;
    if (first_time) {
        first_time = false;
        last_time = std::chrono::high_resolution_clock::now();
    }
    auto center = room_detector_estimator.estimate(points);
    if (not center.has_value())
        return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
    if (center.value().norm() < 320.f){
        // if (first_time) {first_time = false; last_time = std::chrono::high_resolution_clock::now();}
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
        if (elapsed > 500){
            first_time = true;
            if (center.value().norm() < 250.f)
                return {STATE::TURN, 0.f, 0.f};
        }
        // return {STATE::TURN, 0.f, 0.f};
    }
    const auto &doors = nominal_rooms[current_room].doors;
    if (door_crossing.valid) door_crossing.track_entering_door(doors);

    const auto &[adv_vel, rot_vel, _] = do_work(center.value());
    return {STATE::GOTO_ROOM_CENTER, adv_vel, rot_vel};
}

SpecificWorker::RetVal SpecificWorker::turn(const Corners &corners)
{
    //const auto &[success, room_index, left_right] = image_processor.check_colour_patch_in_image(camera360rgb_proxy,  this->label_img);
    const auto &[success, room_index, left_right] = image_processor.check_number_in_image(mnist_proxy, camera360rgb_proxy, this->label_img);
    // qInfo() << __FUNCTION__ << "Detected number: " << room_index;
    //if (detected_number > 0 && detected_number <= std::size(nominal_rooms))
    if (success)
    {
        current_room = room_index - 1;
        door_selected = false;

        // update robot pose to have a fresh value
        if (const auto res = update_robot_pose(corners, robot_pose, true); res.has_value())
            robot_pose = res.value().first;
        else return{STATE::TURN, 0.f, left_right*params.RELOCAL_ROT_SPEED/2};

        // save doors to nominal_room if not previously visited
        if (not nominal_rooms[current_room].visited) {
            nominal_rooms[current_room].name = image_processor.room_name_from_index(current_room);
            auto doors = door_detector.doors();
            if (doors.empty()){return{STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};}
            for (auto &d : doors)
            {
                d.p1_global = nominal_rooms[current_room].get_projection_of_point_on_closest_wall(robot_pose.cast<float>() * d.p1.cast<float>());
                d.p2_global = nominal_rooms[current_room].get_projection_of_point_on_closest_wall(robot_pose.cast<float>() * d.p2.cast<float>());
                qInfo() << d.p1_global.norm() << d.p2_global.norm();
            }

            nominal_rooms[current_room].doors = doors;
            nominal_rooms[current_room].visited = true;
        }

        draw_nominal_room();
        draw_nominal_doors(&viewer_room->scene);

        // finish door tracking and update door crossing info
        if (door_crossing.valid) {
            door_crossing.set_entering_data(current_room, nominal_rooms);
            nominal_rooms[door_crossing.leaving_room_index].doors[door_crossing.leaving_door_index].connects_to_door = door_crossing.entering_door_index;
            nominal_rooms[door_crossing.leaving_room_index].doors[door_crossing.leaving_door_index].connects_to_room = door_crossing.entering_room_index;
            nominal_rooms[door_crossing.leaving_room_index].doors[door_crossing.leaving_door_index].visited = true;
            nominal_rooms[current_room].doors[door_crossing.entering_door_index].connects_to_door = door_crossing.leaving_door_index;
            nominal_rooms[current_room].doors[door_crossing.entering_door_index].connects_to_room = door_crossing.leaving_room_index;
            door_crossing.valid = false;
        }
        localised = true;

        // door selection logic
        const auto nominal_doors = nominal_rooms[current_room].doors;
        const auto selected_doors = door_detector.doors();
        if (selected_doors.size() == 1){current_door = 0;}
        else {
            auto nominal_it = std::ranges::find_if(nominal_doors, [](const auto &door){return !door.visited;});
            if (nominal_it != nominal_doors.end()) current_door = std::distance(nominal_doors.begin(), nominal_it);
            else {
                std::mt19937 gen(rd());
                std::uniform_int_distribution<int> dice(0, nominal_doors.size()-1);
                current_door = dice(gen);
            }
        }
        return {STATE::GOTO_DOOR, 0.0f, 0.0f};
    }
    // continue turning
    return {STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};
}

SpecificWorker::RetVal SpecificWorker::goto_door(const RoboCompLidar3D::TPoints &points, const Eigen::Vector2f &center_point, QGraphicsScene *scene) {
    Doors doors;
    // Exit conditions
    if (doors = door_detector.doors(); doors.empty())
    {
        qInfo() << __FUNCTION__ << "No doors detected, switching to UPDATE_POSE";
        return {STATE::GOTO_DOOR, 0.0f, 0.0f}; // TODO: keep moving for a while?
    }

    // select from doors, the one closest to the nominal door
    Door target_door;
    if (localised)
    {
       auto nominal_door = nominal_rooms[current_room].doors[current_door];
       const auto selected_door = std::ranges::min_element(doors, [nominal_door, this](const auto &a, const auto &b)
       { return (a.center() - robot_pose.inverse() * nominal_door.center_global()).norm() < (b.center() - robot_pose.inverse() * nominal_door.center_global()).norm(); });
       door_selected = true;
       target_door = *selected_door;
    }
    else // select the one closest ot the robot's heading direction
    {
        const auto selected_door = std::ranges::min_element(doors, [](const auto &a, const auto &b)
        { return abs(a.p1_angle) < abs(b.p1_angle); });
        door_selected = true;
        target_door = *selected_door;
    }

    // distance to target is less than threshold, stop and switch to ORIENT_RO_DOOR
    constexpr float offset = 400.f;
    const auto target = target_door.center_before(robot_pose.translation(), center_point, offset);
    const auto dist_to_door = target.norm();

    // draw target
    static QGraphicsItem *door_target_draw = nullptr;
    if (door_target_draw != nullptr)
        scene->removeItem(door_target_draw);
    door_target_draw = scene->addEllipse(-50, -50, 100, 100, QPen(Qt::magenta), QBrush(Qt::magenta));
    door_target_draw->setPos(target.x(), target.y());

    // Exit condition
    if (dist_to_door < params.DOOR_REACHED_DIST)
    {
        if (door_target_draw != nullptr) {
            scene->removeItem(door_target_draw);
            door_target_draw = nullptr;
        }
        door_selected = false;
        return {STATE::ORIENT_TO_DOOR, 0.0f, 0.0f};
    }

    const auto target2d = Eigen::Vector2d(target.x(), target.y());
    const auto &[adv, rot, _] = do_work(target2d);
    return {STATE::GOTO_DOOR, adv, rot};
}


SpecificWorker::RetVal SpecificWorker::orient_to_door(const float &max_match_error) {
    static std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();
    static bool first_time = true;
    bool lost = false;
    //if (first_time) {first_time = false; last_time = std::chrono::high_resolution_clock::now();}
    if (max_match_error > params.RELOCAL_DONE_MATCH_MAX_ERROR) {
        if (first_time) {last_time = std::chrono::high_resolution_clock::now(); first_time = false;}
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
        if (elapsed > 3000) {
            localised = false;
            first_time = true;
            lost = true;
            // return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
        }
    } else first_time = true;

    const auto doors = door_detector.doors();
    // const auto doors = validate_doors(_doors);
    if (localised) { // && !door_selected
        auto nominal_door = nominal_rooms[current_room].doors[current_door];
        const auto selected_door = std::ranges::min_element(doors, [nominal_door, this](const auto &a, const auto &b) {
            return (a.center() - robot_pose.inverse() * nominal_door.center_global()).norm() < (b.center() - robot_pose.inverse() * nominal_door.center_global()).norm(); });
        // door_selected = true;
        draw_target(Eigen::Vector2d(selected_door->center().x(), selected_door->center().y()), &viewer->scene, false);
        if (abs(selected_door->center_angle()) < params.RELOCAL_MAX_ORIENTED_ERROR) {
            // door_selected = false;
            draw_target(Eigen::Vector2d(selected_door->center().x(), selected_door->center().y()), &viewer->scene, true);
            return {STATE::CROSS_DOOR, 0.7f, 0.f};
        }
        else {
            const auto target = Eigen::Vector2d(selected_door->center().x(), selected_door->center().y());
            return {STATE::ORIENT_TO_DOOR, 0.f, std::get<1>(do_work(target))};
        }
    }
    else { // select the one closest to the robot's heading direction
        const auto selected_door = std::ranges::min_element(doors, [](const auto &a, const auto &b)
            {return std::fabs(a.center_angle()) < std::fabs(b.center_angle()); });
        // door_selected = true;
        if (lost) {
            draw_target(Eigen::Vector2d(selected_door->center().x(), selected_door->center().y()), &viewer->scene, true);
            return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
        }
        draw_target(Eigen::Vector2d(selected_door->center().x(), selected_door->center().y()), &viewer->scene, false);
        if (abs(selected_door->center_angle()) < params.RELOCAL_MAX_ORIENTED_ERROR) {
            draw_target(Eigen::Vector2d(selected_door->center().x(), selected_door->center().y()), &viewer->scene, true);
            door_selected = false;
            return {STATE::CROSS_DOOR, 0.7f, 0.f};
        }
        else {
            const auto target = Eigen::Vector2d(selected_door->center().x(), selected_door->center().y());
            return {STATE::ORIENT_TO_DOOR, 0.f, std::get<1>(do_work(target))};
        }
    }
}

SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points) {
    static std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();
    static bool first_time = true;

    if (first_time) {first_time = false; last_time = std::chrono::high_resolution_clock::now();}

    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
    if (elapsed > 2000) {
        first_time = true;
        localised = false;

        if (static_cast<size_t>(current_room) >= door_crossing_data.size()) door_crossing_data.resize(current_room + 1);
        if (static_cast<size_t>(current_door) >= door_crossing_data[current_room].size()) door_crossing_data[current_room].resize(current_door + 1);
        auto &dc_opt = door_crossing_data[current_room][current_door];
        if (not dc_opt.has_value()){ DoorCrossing dc{current_room, current_door}; dc_opt = dc; }
        auto data = dc_opt.value();
        data.set_entering_data(current_room, nominal_rooms);
        door_crossing = data;
        return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
    }
    return {STATE::CROSS_DOOR, 750.f, 0.f};
}


std::optional<std::pair<Eigen::Affine2f, float>> SpecificWorker::update_robot_pose(const Corners &corners, const Eigen::Affine2f &r_pose, bool transform_corners)
{
    Match match;
    if (transform_corners)
        match = hungarian.match(corners, nominal_rooms[current_room].transform_corners_to(r_pose.inverse()));
    else
        match = hungarian.match(corners, nominal_rooms[current_room].corners());

    if (match.empty() or match.size() < 3)
        return {};

    const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
        {return std::get<2>(a) < std::get<2>(b);});
    const auto max_match_error = std::get<2>(*max_error_iter);

    // create matrices W and B for pose estimation
    Eigen::MatrixXd W(match.size() * 2, 3);
    Eigen::VectorXd b(match.size() * 2);
    for (const auto &&[i, m] : match | iter::enumerate) {
        auto &[meas_c, nom_c, _] = m;
        auto &[p_meas, __, ___] = meas_c;
        auto &[p_nom, ____, _____] = nom_c;
        b(2 * i) = p_nom.x() - p_meas.x();
        b(2 * i + 1) = p_nom.y() - p_meas.y();
        W.block<1, 3>(2 * i, 0) << 1.0, 0.0, -p_meas.y();
        W.block<1, 3>(2 * i + 1, 0) << 0.0, 1.0, p_meas.x();
    }
    // estimate new pose with pseudoinverse
    const auto r = (W.transpose() * W).inverse() * W.transpose() * b;
    if (r.array().isNaN().any())
        return {};
    auto r_pose_copy = r_pose;
    // update robot pose
    r_pose_copy.translate(Eigen::Vector2f(r(0), r(1)));
    r_pose_copy.rotate(r(2));
    return {{r_pose_copy, max_match_error}};
}
void SpecificWorker::move_robot(float adv, float rot, float max_match_error) {
    omnirobot_proxy->setSpeedBase(0, adv, rot);
}

//=========================================================================================================================================
std::optional<RoboCompLidar3D::TPoints> SpecificWorker::read_data() {
    // Try-Catch block to read the laser data
    RoboCompLidar3D::TData data;
    try {
        data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", params.LIDAR_DISTANCE, 1);
        // data = lidar3d_proxy->getLidarDataWithThreshold2d("pearl", 15000, 2);
    } catch (const Ice::Exception &ex) {
        std::cout << ex.what() << std::endl;
        return {};
    }

    // filter
    // qInfo() << "full" << data.points.size();
    if (data.points.empty()) {
        qWarning() << "No points received";
        return {};
    }
    RoboCompLidar3D::TPoints filter_data;
    if (const auto &filter_data_ = filter_min_distance(data.points); filter_data_.has_value())
        filter_data = filter_data_.value();
    else {
        qWarning() << "No points filtered";
        return {};
    }

    // return filter_data;
    return filter_isolated_points(filter_data, 200);
}

std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d) {
    if (points.empty())
        return {};

    const float d_squared = d * d; // avoid sqrt by comparing squared distances
    std::vector<bool> hasNeighbor(points.size(), false);

    // Create index vector for parallel iteration
    std::vector<size_t> indices(points.size());
    std::iota(indices.begin(), indices.end(), size_t{0});

    // Parallelize outer loop - each thread checks one point
    std::for_each(std::execution::par, indices.begin(), indices.end(),
                  [&](size_t i) {
                      const auto &p1 = points[i];
                      // Sequential inner loop (avoid nested parallelism)
                      for (auto &&[j, p2] : iter::enumerate(points)) {
                          if (i == j)
                              continue;
                          const float dx = p1.x - p2.x;
                          const float dy = p1.y - p2.y;
                          if (dx * dx + dy * dy <= d_squared) {
                              hasNeighbor[i] = true;
                              break;
                          }
                      }
                  });

    // Collect result
    std::vector<RoboCompLidar3D::TPoint> result;
    result.reserve(points.size());
    for (auto &&[i, p] : iter::enumerate(points))
        if (hasNeighbor[i])
            result.push_back(points[i]);
    return result;
}

std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_min_distance(const RoboCompLidar3D::TPoints &points) {
    // non-empty condition
    if (points.empty())
        return {};

    RoboCompLidar3D::TPoints result;
    result.reserve(points.size());

    // 3. Loop over the groups produced by iter::groupby
    for (auto &&[angle, group] : iter::groupby(points, [](const auto &p) {
        float multiplier = std::pow(10.0f, 2);
        return std::floor(p.phi * multiplier) / multiplier;
    })) {
        // 'group' is an iterable object containing all Points for the current
        // angle.
        auto min_it = std::min_element(
                std::begin(group), std::end(group),
                [](const auto &a, const auto &b) { return a.r < b.r; });
        // if (min_it->z > 300)
        result.emplace_back(*min_it); // Push the element with the minimum distance
    }

    return result;
}

std::expected<int, std::string> SpecificWorker::closest_lidar_index_to_given_angle(const auto &points, float angle) {
    // search for the point in points whose phi value is closest to angle
    auto res = std::ranges::find_if(points, [angle](auto &a) { return a.phi > angle; });
    if (res != std::end(points))
        return std::distance(std::begin(points), res);
    else
        return std::unexpected("No closest value found in method <closest_lidar_index_to_given_angle>");
}

void SpecificWorker::draw_lidar(auto &walls, auto &obstacles, auto *scene) {
    static std::vector<QGraphicsItem*> items;

    for (auto i : items) {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    // Draw red walls
    auto brush_wall = QBrush(QColor(Qt::red));
    auto pen_wall = QPen(Qt::NoPen);
    for (const auto &p : walls) {
        auto item = scene->addRect(-25, -25, 50, 50, pen_wall, brush_wall);
        item->setPos(p.x, p.y);
        items.push_back(item);
    }

    // Draw green obstacles
    auto brush_obs = QBrush(QColor(Qt::darkGreen));
    for (const auto &p : obstacles) {
        auto item = scene->addRect(-20, -20, 40, 40, pen_wall, brush_obs);
        item->setPos(p.x, p.y);
        items.push_back(item);
    }
}

void SpecificWorker::draw_lidar(auto &filtered_points, Eigen::Vector2d room_center, QGraphicsScene *scene) {
    static std::vector<QGraphicsItem *> items; // store items so they can be shown between iterations

    // remove all items drawn in the previous iteration
    for (auto i : items) {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    auto color = QColor(Qt::green);
    auto brush = QBrush(QColor(Qt::green));
    for (auto &&[i, p] : filtered_points | iter::enumerate) {
        brush = QColor(Qt::green);
        auto item = scene->addRect(-50, -50, 100, 100, color, brush);
        item->setPos(p.x, p.y);
        items.push_back(item);
        // qInfo() << std::hypot(p.x, p.y) << p.distance2d << p.r;
    }

    // draw center point
    auto center = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::pink), QBrush(QColor(QColorConstants::Svg::pink)));
    center->setPos(room_center.x(), room_center.y());
    items.push_back(center);
}

void SpecificWorker::new_target_slot(QPointF point) {
    qDebug() << "Slot new_target_slot llamado con el punto:" << point;
}

void SpecificWorker::emergency() {
    std::cout << "Emergency worker" << std::endl;
    // emergencyCODE
    //
    // if (SUCCESSFUL) //The componet is safe for continue
    //   emmit goToRestore()
}

// Execute one when exiting to emergencyState
void SpecificWorker::restore() {
    std::cout << "Restore worker" << std::endl;
    // restoreCODE
    // Restore emergency component
}

int SpecificWorker::startup_check() {
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
    return 0;
}

void SpecificWorker::update_report_posotion() {
    try {
        RoboCompGenericBase::TBaseState bState;
        omnirobot_proxy->getBaseState(bState);
        robot_draw->setRotation(bState.alpha * 180 / M_PI);
        robot_draw->setPos(bState.x, bState.z);
        std::cout << bState.alpha << " " << bState.x << " " << bState.z << std::endl;
    } catch (const Ice::Exception &e) {
        std::cout << e.what() << std::endl;
    }
}

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// RoboCompDifferentialRobot::void
// this->differentialrobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompDifferentialRobot::void
// this->differentialrobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompDifferentialRobot::void
// this->differentialrobot_proxy->getBaseState(RoboCompGenericBase::TBaseState
// state) RoboCompDifferentialRobot::void
// this->differentialrobot_proxy->resetOdometer()
// RoboCompDifferentialRobot::void
// this->differentialrobot_proxy->setOdometer(RoboCompGenericBase::TBaseState
// state) RoboCompDifferentialRobot::void
// this->differentialrobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompDifferentialRobot::void
// this->differentialrobot_proxy->setSpeedBase(float adv, float rot)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->stopBase()

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// RoboCompLaser::TLaserData
// this->laser_proxy->getLaserAndBStateData(RoboCompGenericBase::TBaseState
// bState) RoboCompLaser::LaserConfData this->laser_proxy->getLaserConfData()
// RoboCompLaser::TLaserData this->laser_proxy->getLaserData()

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData
