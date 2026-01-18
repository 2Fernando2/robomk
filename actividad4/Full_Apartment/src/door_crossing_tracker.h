//
// Created by pbustos on 14/12/25.
//

#ifndef LOCALISER_DOOR_CROSSING_TRACKER_H
#define LOCALISER_DOOR_CROSSING_TRACKER_H

// Door crossing struct to track used door in crossing situations

        /**
         * @brief Struct to manage the state of the robot while transitioning between rooms.
         * Tracks the "Leaving" door and resolves the "Entering" door in the next room's coordinate system.
         */
        struct DoorCrossing
        {
            int leaving_room_index = -1;            // Index of the room the robot is existing.
            int leaving_door_index = -1;            // Index of the door being crossed in the previous room.
            int entering_room_index = -1;           // Index of the room the robot is entering.
            int entering_door_index = -1;           // Index of the corresponding door in the new room.
            bool valid = false;                             // only true if both leaving and entering data are set
            DoorCrossing() = default;                       // attribs take default values

            /**
             * @brief Initializes a crossing event starting from a specific room and door
             */
            DoorCrossing(int room_index, int door_index)    // leaving room and door are initialized
                : leaving_room_index{room_index}, leaving_door_index{door_index} { valid = true; }
            Eigen::Vector2f leaving_door_center{0.f, 0.f};   // local coordinate of tracked door

            /**
             * @brief Updates the local tracking of the door being crossed.
             * Finds the closest door in the current scan to the last known door position to keep tracking it.
             * @param doors List of currently detected doors.
             */
            void track_entering_door(const Doors &doors)
            {
                if (doors.empty()) return;
                // find the door whose center is closest to the leaving door center in local coordinates
                leaving_door_center = std::ranges::min_element(doors, [d =leaving_door_center](const auto &a, const auto &b)
                { return (a.center()-d).norm() < (b.center()-d).norm();})->center();
            }

            /**
             * @brief Resolves the identity of the door in the new room (Entering Door).
             * Uses the tracked local position transformed into global coordinates to find which
             *  nominal door corresponds to the one just crossed.
             * @param room_index Index of the new room (Entering room).
             * @param nom_rooms Reference to the vector of Nominal Rooms (map).
             */
            void set_entering_data(int room_index, const std::vector<NominalRoom> &nom_rooms)          // compute door index from leaving_door_center
            {
                entering_room_index = room_index;
                // find the door in nominal_rooms[room_index] whose center is closest to leaving_door_center transformed to global
                const auto &nominal_doors = nom_rooms[room_index].doors;
                if (nominal_doors.empty())
                {
                    qWarning() << __FUNCTION__ << "empty nominal doors for room" << room_index;
                    return;
                }
                const auto closest_door = std::ranges::min_element(nominal_doors, [this](const auto &a, const auto &b)
                { return (a.center_global() - leaving_door_center).norm() < (b.center_global() - leaving_door_center).norm(); });
                entering_door_index = static_cast<int>(std::distance(nominal_doors.begin(), closest_door));
            }
        };
#endif //LOCALISER_DOOR_CROSSING_TRACKER_H