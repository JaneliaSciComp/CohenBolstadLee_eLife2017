SET(foo_MOC_HDRS
	Console.h
	RadioButtonGroup.h
	graphwidget.h
	viewingWindowQt.h
	QCustomPlot/qcustomplot.h
	SerialPortDialog//settingsdialog.h
)

SET(foo_RCCS jovian.qrc)
IF( QT4_FOUND )
	QT4_ADD_RESOURCES(foo_RCC_SRCS ${foo_RCCS})
ELSE()
	QT5_ADD_RESOURCES(foo_RCC_SRCS ${foo_RCCS})
ENDIF()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
  # As moc files are generated in the binary dir, tell CMake
  # to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(TARGET_SRC
	CameraBlockUpdateCallback.cpp
	CameraPathUpdateCallback.cpp
	CameraThresholdUpdateCallback.cpp
    Communicator.cpp
    Config_Memento.cpp
    Console.cpp
    MyGraphicsWindowQt.cpp
    MotionCallback.cpp
    RadioButtonGroup.cpp
    edge.cpp
    graphwidget.cpp
    hud.cpp
    node.cpp
    scene_model.cpp
    viewingWindowQt.cpp
    QCustomPlot/qcustomplot.cpp
	SerialPortDialog//settingsdialog.cpp
    ${foo_RCC_SRCS}
)

SET(TARGET_H
	CameraBlockUpdateCallback.h
	CameraPathUpdateCallback.h
	CameraThresholdUpdateCallback.h
	CameraUpdateCallback.h
	ColladaVisitor.h
	Communicator.h
	Config_Memento.h
	Console.h
	Globals.h
	Graph_Evaluator.h
	MyGraphicsWindowQt.h
	MotionCallback.h
    RadioButtonGroup.h
	edge.h
	graphwidget.h
	hud.h
	node.h
	scene_model.h
	shader_defs.h
	viewingWindowQt.h
	QCustomPlot/qcustomplot.h
	SerialPortDialog//settingsdialog.h
)

#ADD_DEFINITIONS(-DUSE_QT4 -DQT_OPENGL_SUPPORT)

IF( QT4_FOUND )
  INCLUDE_DIRECTORIES( ${Jovian_SOURCE_DIR}/src ${OSG_INCLUDE_DIR} ${QT_INCLUDE_DIR} ${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR} ${QT_QTOPENGL_INCLUDE_DIR} ${QT_WIDGETS_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
ELSE()
  INCLUDE_DIRECTORIES( ${Jovian_SOURCE_DIR}/src ${OSG_INCLUDE_DIR} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS} ${Qt5PrintSupport_INCLUDE_DIRS} ${Qt5SerialPort_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )
ENDIF()

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

#### end var setup  ###
INCLUDE(OsgMacroUtils)
add_library( Jovian ${TARGET_SRC} ${TARGET_H} )

