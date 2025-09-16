cmake_minimum_required(VERSION 3.0)
project(ejemplo1)
MESSAGE( STATUS "COMPILING WITH QT6" )
find_package(Qt6 REQUIRED COMPONENTS Gui Widgets Sql Core OpenGL Xml  )
set(QT_LIBRARIES Qt6::Gui Qt6::Core Qt6::Widgets Qt6::Sql Qt6::OpenGL)

MACRO(QT_WRAP_CPP output )
       FOREACH( input_file ${ARGN} )
               QT6_WRAP_CPP( ${output} ${input_file} )
       ENDFOREACH( input_file)
       SET_PROPERTY(SOURCE ${${output}} PROPERTY SKIP_AUTOGEN ON)
ENDMACRO(QT_WRAP_CPP)

MACRO (QT_WRAP_UI output )
       FOREACH( input_file ${ARGN} )
               MESSAGE( STATUS "Generating header file from ${input_file}")
               QT6_WRAP_UI( ${output} ${input_file} )
       ENDFOREACH( input_file)
       SET_PROPERTY(SOURCE ${${output}} PROPERTY SKIP_AUTOGEN ON)
ENDMACRO(QT_WRAP_UI)

include_directories(${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})

# Sources set
SET ( SOURCES  ${SOURCES}  ejemplo1.cpp  main.cpp )
# Headers set
SET ( HEADERS  ${HEADERS} ejemplo1.h )

SET( CMAKE_AUTOMOC ON )
SET( CMAKE_AUTOUIC ON )

# Specify construction and link process
ADD_EXECUTABLE( ejemplo1 ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES}  ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( ejemplo1 ${LIBS} ${STATIC_LIBS} ${SPECIFIC_LIBS} ${QT_LIBRARIES} )
INSTALL(TARGETS ejemplo1  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )

