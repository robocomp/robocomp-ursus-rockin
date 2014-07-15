cmake_minimum_required(VERSION 2.6)
PROJECT( cuba2dnaturallandmarksComp )
IF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
MESSAGE(FATAL_ERROR "
 ROBOCOMPROOT variable not set, check your bashrc profile!
")
ENDIF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )

# Sources set
SET ( SOURCES
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  ./cuba2dnaturallandmarksComp.cpp
  ./cuba2dnaturallandmarksI.cpp
  ./worker.cpp
  ./cdata.cpp
  ./circle.cpp
  ./curvaturalaser.cpp
#  $ENV{ROBOCOMP}/classes/kalman/kalmanroi2d.cpp
  $ENV{ROBOCOMP}/classes/rcdraw/rcdraw.cpp
  $ENV{ROBOCOMP}/classes/rcdraw/rcdrawrobot.cpp
)

# Headers set
SET ( HEADERS
 ./cuba2dnaturallandmarksI.h
 ./worker.h
 $ENV{ROBOCOMP}/classes/rcdraw/rcdraw.h
 $ENV{ROBOCOMP}/classes/rcdraw/rcdrawrobot.h
)

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( DifferentialRobot Laser Cuba2Dnaturallandmarks CamMotion CCDAmatching CommonHead JointMotor )
SET (EXECUTABLE_OUTPUT_PATH $ENV{ROBOCOMP}/components/robocomp-ursus-rockin/cuba2dnaturallandmarksComp/bin)

# Qt4
ADD_DEFINITIONS( -O3 -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTXML TRUE )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )


# Specify construction and link process
ADD_EXECUTABLE( cuba2dnaturallandmarksComp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( cuba2dnaturallandmarksComp ${QT_LIBRARIES} ${LIBS} -lboost_system)
