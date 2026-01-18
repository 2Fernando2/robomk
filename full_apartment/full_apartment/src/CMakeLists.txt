# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
        ../src/hungarian.cpp
        ../src/ransac_line_detector.cpp
        ../src/room_detector.cpp
        ../src/door_detector.cpp
        ../src/qcustomplot.cpp
        ../src/time_series_plotter.cpp
        ../src/pointcloud_center_estimator.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
        ../src/hungarian.h
        ../src/ransac_line_detector.h
        ../src/room_detector.h
        ../src/munkres.hpp
        ../src/common_types.h
        ../src/door_detector.h
        ../src/qcustomplot.h
        ../src/time_series_plotter.h
        ../src/nominal_room.h
        ../src/pointcloud_center_estimator.h
        ../src/door_crossing_tracker.h
)

INCLUDE($ENV{ROBOCOMP}/cmake/modules/opencv4.cmake)

SET(LIBS ${LIBS} tbb Qt6PrintSupport)
LIST(APPEND LIBS ${LIBS})

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-fmax-errors=1 -fno-char8_t)

