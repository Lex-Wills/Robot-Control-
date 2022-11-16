cmake_minimum_required(VERSION 3.1.0)
project(RX150_Interface)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        /usr/share/cmake_modules/cmake/Modules)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

enable_testing()

#  Dependencies: Boost
find_package( Boost REQUIRED COMPONENTS system program_options unit_test_framework)
include_directories ( ${Boost_INCLUDE_DIRS} )
link_directories ( ${Boost_LIBRARY_DIRS} )

message("Boost libraries: " ${Boost_LIBRARIES})

#  Dependencies: Eigen3
find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

#  Dependencies: Qt5
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Widgets  CONFIG REQUIRED)
find_package(Qt5Xml  CONFIG REQUIRED)
find_package(Qt5OpenGL  CONFIG REQUIRED)
include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS})
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#  Dependencies: QGLViewer
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
FIND_PACKAGE(QGLViewer REQUIRED)
INCLUDE_DIRECTORIES(SYSTEM ${QGLVIEWER_INCLUDE_DIR} ${GLUT_INCLUDE_PATH} ${OPENGL_INCLUDE_DIR})
SET(QT_LIBRARIES  Qt5::Core Qt5::Widgets Qt5::Xml Qt5::OpenGL ${OPENGL_LIBRARIES} ${QGLVIEWER_LIBRARIES} -lpthread)


set(CMAKE_AUTOMOC ON)


set(CMAKE_VERBOSE_MAKEFILE true)

INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/detail)


SET(HEADERS
        rx150/RX150RobotInterface.h
        rx150/RX150Kinematics.h
        detail/tcp/RobotServerComm.h
        detail/tcp/tcp_connection.h
        detail/tcp/tcp_server.h
        detail/tcp/RX150RemoteInterface.h
        detail/serial/RX150SerialInterface.h
        detail/opengl/SceneViewer.h
        detail/qt/SliderBridge.h
)

SET(SOURCES
        rx150/RX150Kinematics.cpp
        detail/tcp/RobotServerComm.cpp
        detail/tcp/tcp_connection.cpp
        detail/tcp/tcp_server.cpp
        detail/tcp/RX150RemoteInterface.cpp
        detail/serial/RX150SerialInterface.cpp
        detail/opengl/SceneViewer.cpp
        detail/qt/SliderBridge.cpp
)

SET(GCC_LIBS -pthread -lpthread -lserial)
INCLUDE_DIRECTORIES(SYSTEM "/usr/include/libserial")

add_library(RX150Interface ${HEADERS} ${SOURCES})
target_link_libraries(RX150Interface ${Boost_LIBRARIES} ${GCC_LIBS}) 


set(SERVERS sim-server rx150-server)
foreach(server ${SERVERS})
	add_executable(${server} servers/${server}.cpp) 
	target_link_libraries(${server} RX150Interface ${Boost_LIBRARIES} ${GCC_LIBS})
endforeach()

set(CLIENTS echo-client animation-demo position-control manual-control)
foreach(client ${CLIENTS})
	add_executable(${client} clients/${client}.cpp) 
	target_link_libraries(${client} RX150Interface ${Boost_LIBRARIES} ${GCC_LIBS})
endforeach()


set(QT_CLIENTS 3d-vis slider-control)
foreach(client ${QT_CLIENTS})
	add_executable(${client} clients/${client}.cpp ${HEADERS} ${SOURCES} ${QT_MOC_SRCS}) 
	target_link_libraries(${client} RX150Interface ${QT_LIBRARIES} )
endforeach()


ADD_SUBDIRECTORY(test)


