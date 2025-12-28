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
        viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
        auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0,
                                        100, QColor("Blue"));
        robot_draw = r;
        // this->resize(900, 450);

        viewer_room = new AbstractGraphicViewer(this->frame_room, this->dimensions);
        auto [rr, re] = viewer_room->add_robot(
                params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_room_draw = rr;

        // draw room in viewer_room
        room_draw = viewer_room->scene.addRect(nominal_rooms[0].rect(), QPen(Qt::black, 30));

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

    // detect doors and filter beyond points -> doors from robot perspective (?)
    const auto &[filtered_points, doors_] = door_detector.filter_points(points, &viewer->scene);
    doors = doors_;

    // compute corners - detect and draw them
    const auto &[corners, lines] = room_detector.compute_corners(filtered_points, &viewer->scene);
    const auto center_opt = room_detector_estimator.estimate(filtered_points);
    if (center_opt.has_value()) draw_lidar(filtered_points, center_opt.value(), &viewer->scene);
    else { std::cerr << "No center room point found" << std::endl; return; }

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

    /* -> -> -> -> -> -> -> -> ->  moved to "update_robot_pose()"
    if (localised)
    {
        // match corners transforming first nominal corners to robot's frame
        match = hungarian.match(corners, nominal_rooms[current_room].transform_corners_to(robot_pose.inverse()));

        // compute max of match error
        if (not match.empty())
        {
            const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
            {
                return std::get<2>(a) < std::get<2>(b);
            });
            max_match_error = static_cast<float>(std::get<2>(*max_error_iter));
            time_series_plotter->addDataPoint(match_error_graph, max_match_error);
            // print_match(match, max_match_error); // debugging
        }
        update_robot_pose(corners, robot_pose, true);
        }
    } -> -> -> -> -> -> -> -> -> moved to "update_robot_pose()" */

    // Process state machine
    RetVal ret_val = state_machine(filtered_points, match, corners, max_match_error);
    auto [st, adv, rot] = ret_val;
    state = st;
    if (max_match_error < 99999.f)
        max_error_registered = std::max(max_error_registered, max_match_error);

    // Send movements commands to the robot constrained by the match_error
    if (keyw_mode) keyw(); else move_robot(adv, rot, max_match_error);

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
}


SpecificWorker::RetVal SpecificWorker::state_machine(const RoboCompLidar3D::TPoints &points, const Match &match, const Corners &corners, const float &max_match_error) {
    RetVal res;
    switch (state) {
        case STATE::GOTO_ROOM_CENTER:
            res = goto_room_center(points);
            break;
        case STATE::TURN:
            res = turn(corners);
            break;
        case STATE::GOTO_DOOR:
            res = goto_door(points, &viewer->scene);
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

SpecificWorker::RetVal SpecificWorker::localise(const Match &match) {
    return {};
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

    for (const auto &d : doors){
        // draw p1 and p2 points from door
        auto p1_point = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::orange),QBrush(QColorConstants::Svg::orange));
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
    static bool first_time;
    if (first_time) {
        first_time = false;
        last_time = std::chrono::high_resolution_clock::now();
    }
    auto center = room_detector_estimator.estimate(points);
    if (not center.has_value())
        return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
    if (center.value().norm() < 320.f){
        if (first_time) {first_time = false; last_time = std::chrono::high_resolution_clock::now();}
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
        if (elapsed > 500){
            first_time = true;
            if (center.value().norm() < 250.f)
                return {STATE::TURN, 0.f, 0.f};
        }
        // return {STATE::TURN, 0.f, 0.f};
    }

    const auto &[adv_vel, rot_vel, _] = do_work(center.value());
    return {STATE::GOTO_ROOM_CENTER, adv_vel, rot_vel};
}

SpecificWorker::RetVal SpecificWorker::turn(const Corners &corners)
{
    const auto &[success, room_index, left_right] = image_processor.check_colour_patch_in_image(camera360rgb_proxy,  this->label_img);
    if (success)
    {
        current_room = room_index;

        const auto m = hungarian.match(corners, nominal_rooms[current_room].corners());
        if (m.empty()) { qInfo() << __FUNCTION__ << "empty match"; }
        if (m.size() < 3)
        {
            qInfo() << __FUNCTION__ << "m size < 3";
            return {STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};
        }

        const auto max_error_iter = std::ranges::max_element(m, [](const auto &a, const auto &b)
        { return std::get<2>(a) < std::get<2>(b); });

        if (const auto max_match_error = std::get<2>(*max_error_iter); max_match_error > params.RELOCAL_DONE_MATCH_MAX_ERROR)
        {
            qInfo() << __FUNCTION__ << "match error > " << params.RELOCAL_DONE_MATCH_MAX_ERROR;
            return {STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};
        }

        // update robot pose to have a fresh value
        update_robot_pose(corners, robot_pose, true);

        // save doors to nominal_room
        auto doors = door_detector.doors();
        if (doors.empty())
        {
            qWarning() << __FUNCTION__ << "empty doors";
            return {STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};
        }

        for (auto &d : doors)
        {
            d.p1_global = nominal_rooms[current_room].get_projection_of_point_on_closest_wall(robot_pose * d.p1);
            d.p2_global = nominal_rooms[current_room].get_projection_of_point_on_closest_wall(robot_pose * d.p2);
        }

        nominal_rooms[current_room].doors = doors;

        // choose door to go
        current_door = 0; // TODO MORE SOPHISTICATED CHOICE
        // we need to match the current selected nominal door to the successive local doors detected during the approach
        // select the local door closest to the selected nominal door
        const auto nominal_door = nominal_rooms[current_room].doors[current_room];
        const auto visible_doors = door_detector.doors();
        const auto selected_door = std::ranges::min_element(visible_doors, [nominal_door, this](const auto &a, const auto &b)
        {
            return (a.center() - robot_pose.inverse() * nominal_door.center_global()).norm()
                    < (b.center() - robot_pose.inverse() * nominal_door.center_global()).norm();
        });
        nominal_rooms[current_room].doors[current_door].p1 = selected_door->p1;
        nominal_rooms[current_room].doors[current_door].p2 = selected_door->p2;
        draw_nominal_room();
        draw_nominal_doors(&viewer_room->scene);

        localised = true;
        return {STATE::GOTO_DOOR, 0.0f, 0.0f};
    }
    // continue turning
    return {STATE::TURN, 0.0f, left_right*params.RELOCAL_ROT_SPEED};
}

SpecificWorker::RetVal SpecificWorker::goto_door(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene) {
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
        qInfo() << __FUNCTION__ << "Localised, selecting door closest to nominal door";
        const auto dn = nominal_rooms[current_room].doors[current_door];
        const auto sd = std::ranges::min_element(doors, [dn, this](const auto &a, const auto &b)
        { return (a.center() - robot_pose.inverse() * dn.center_global()).norm() < (b.center() - robot_pose.inverse() * dn.center_global()).norm(); });
        target_door = *sd;
    }
    else // select the one closest ot the robot's heading direction
    {
        qInfo() << __FUNCTION__ << "No localised, selecting door closest to robot heading";
        const auto sd = std::ranges::min_element(doors, [](const auto &a, const auto &b)
        { return abs(a.p1_angle) < abs(b.p1_angle); });
        target_door = *sd;
    }
    qInfo() << target_door.p1.x() << target_door.p1.y();

    // distance to target is less than threshold, stop and switch to ORIENT_RO_DOOR
    constexpr float offset = 600.f;
    const auto target = target_door.center_before(robot_pose.translation(), offset);
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
        qInfo() << __FUNCTION__ << "Door reached at distance " << dist_to_door << ", switching to ORIENT_TO_DOOR";
        return {STATE::ORIENT_TO_DOOR, 0.0f, 0.0f};
    }

    qInfo() << __FUNCTION__ << "moving to door at " << target.x() << "," << target.y() << " dist: " << dist_to_door;
    const auto target2d = Eigen::Vector2d(target.x(), target.y());
    const auto &[adv, rot, _] = do_work(target2d); // go to first detected door
    return {STATE::GOTO_DOOR, adv, rot};


    // static std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();
    // static bool first_time = true;

    // if (first_time) {
    //    first_time = false;
    //    last_time = std::chrono::high_resolution_clock::now();
    // }

    // auto now = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
    // if (elapsed > 3000) {
    //     first_time = true;
    //     // std::cout << "GOTO_DOOR: Timeout." << std::endl;
    //     if (max_match_error > params.RELOCAL_DONE_MATCH_MAX_ERROR) {
    //         localised = false;
    //         // current_door = -1;
    //         return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
    //     }
    // }
    /*

    // 1. Comprobar si el error excede el umbral
    if (max_match_error > params.RELOCAL_DONE_MATCH_MAX_ERROR) {
        // Si es la primera vez que se detecta el error malo, o si el timeout ya se reseteó, actualizamos last_time.
        if (first_time) {
            last_time = std::chrono::high_resolution_clock::now();
            first_time = false;
        }

        // 2. Comprobar si el error ha estado superando el umbral por más de 3 segundos
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

        if (elapsed > 3000) {
            localised = false;
            return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
        }
    } else first_time = true;

    if (doors.empty()) return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};

    const auto target = robot_pose.inverse() * nominal_rooms[current_room].doors[current_door].center_before(robot_pose.translation());
    if (target.norm() < 300.f) {
        draw_target(target.cast<double>(), &viewer->scene, true);
        return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};
    }

    draw_target(target.cast<double>(), &viewer->scene, false);
    const auto &[adv_speed, rot_speed, _] = do_work(target.cast<double>());

    return {STATE::GOTO_DOOR, adv_speed, rot_speed};
    */
}


SpecificWorker::RetVal SpecificWorker::orient_to_door(const float &max_match_error) {
    static std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();
    static bool first_time = true;
    if (first_time) {first_time = false; last_time = std::chrono::high_resolution_clock::now();}

    // auto now = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
    // if (elapsed > 10000) { // 10s
    //     first_time = true;
    //     if (max_match_error > params.RELOCAL_DONE_MATCH_MAX_ERROR) {
    //         localised = false;
    //         current_door = -1; // Reset solo por error
    //         return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
    //     }
    //     return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
    // }

    if (max_match_error > params.RELOCAL_DONE_MATCH_MAX_ERROR) {
        if (first_time) {last_time = std::chrono::high_resolution_clock::now(); first_time = false;}

        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

        if (elapsed > 3000) {
            localised = false;
            return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
        }
    } else first_time = true;

    if (nominal_rooms[current_room].doors.empty() or current_door == -1 or current_door >= static_cast<int>(nominal_rooms[current_room].doors.size())) {
        current_door = -1;
        return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
    }

    const Eigen::Vector2f door_center_global = nominal_rooms[current_room].doors[current_door].center_global();
    const Eigen::Vector2f door_center_robot_frame = robot_pose.inverse() * door_center_global;
    door_center = door_center_robot_frame.cast<double>();

    const auto &[adv_vel, rot_vel, angle] = do_work(door_center);
    if (angle < M_PI_4 && std::abs(rot_vel) < 0.15f) {
        draw_target(door_center, &viewer->scene, true);
        return {STATE::CROSS_DOOR, 0.f, 0.f};
    }

    draw_target(door_center, &viewer->scene, false);
    return {STATE::ORIENT_TO_DOOR, 0.f, rot_vel};
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
        current_room = (current_room + 1) % std::size(nominal_rooms);
        current_door = -1;
        draw_nominal_room();
        return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
    }
    return {STATE::CROSS_DOOR, 750.f, 0.f};
}
SpecificWorker::RetVal SpecificWorker::update_pose(const Corners &corners, const Match &match) { return {}; }

SpecificWorker::RetVal SpecificWorker::process_state(const RoboCompLidar3D::TPoints &data, const Corners &corners, const Match &match, AbstractGraphicViewer *viewer) { return {}; }

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
        float max_distance = 0;
        for (const auto &n : nominal_rooms)
        {
            if (n.length > max_distance)
                max_distance = n.length;
            if (n.width > max_distance)
                max_distance = n.width;
        }
        data = lidar3d_proxy->getLidarDataWithThreshold2d("pearl", max_distance, 2);
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
    if (const auto &filter_data_ = filter_min_distance_cppitertools(data.points);
            filter_data_.has_value())
        filter_data = filter_data_.value();
    else {
        qWarning() << "No points filtered";
        return {};
    }

    // qInfo() << filter_data.size();

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

std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints &points) {
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
    auto center = scene->addRect(-100, -100, 200, 200, QColor(QColorConstants::Svg::red), QBrush(QColor(QColorConstants::Svg::red)));
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
