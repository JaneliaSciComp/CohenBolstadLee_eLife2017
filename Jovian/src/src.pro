DESTDIR = ..
TARGET = dragonvre

SOURCES = \
	AdapterWidget.cpp 	\
	arena.cpp 	\
	brownian_sphere.cpp 	\
	Console_QT.cpp		\
        dSFMT/dSFMT.c	\
        geometry.cpp            \
        hud.cpp                 \
	MotionCallback.cpp 	\
	ModelSelector.cpp 	\
	terrain.cpp 		\
	path_animation.cpp 	\
        pythonthread.cpp        \
	shader_utility.cpp 	\
        main.cpp

HEADERS = 			\
	AdapterWidget.h      	\
	arena.h      	\
        boost_python_interface.h\
	brownian_sphere.h    	\
	Console_QT.h           	\
	dSFMT/dSFMT.h	\
        geometry.h              \
        hud.h                   \
	MotionCallback.h 	\
	ModelSelector.h 	\
	path_animation.h     	\
        pythonthread.h          \
	terrain_normals.h	\
	terrain_texcoords.h	\
	terrain_coords.h	\
	shader_utility.h 	\
	UI_Console.h         

INCLUDEPATH += . /usr/local/include ${PYTHONINCLUDE}
INCLUDEPATH += /Users/bolstadm/software/include/

DEFINES += D_USE_QT

CONFIG += qt thread
QT += opengl

#mac::LIBS += -L/usr/local/lib -losgDB -losgGA -losgSim -losgUtil -losgText -losgViewer -losg #-framework Carbon -framework AppKit -lz -lm -framework ApplicationServices

LIBS += -L/usr/local/lib -L${CSE_QT_HOME}/lib -losgDB -losgGA -losgSim -losgUtil -losgViewer -losg -lOpenThreads -framework QtGui -framework Carbon -framework AppKit -framework QtOpenGL -framework QtCore -lz -lm -framework ApplicationServices

LIBS += $$(PYTHONLIB) /Users/bolstadm/software/lib/libboost_python-mt.a

# check if debug or release
CONFIG(debug, debug|release) {
  DEBUG_EXT = _d 
} else {
  DEBUG_EXT = 
}

