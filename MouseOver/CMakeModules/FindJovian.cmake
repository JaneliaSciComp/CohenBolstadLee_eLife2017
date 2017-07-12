# Locate jovian.
#
# This script defines:
#   JOVIAN_FOUND, set to 1 if found
#   JOVIAN_LIBRARY
#   JOVIAN_LIBRARY_debug
#   JOVIAN_LIBRARIES, all jovian libraries
#   JOVIAN_INCLUDE_DIR
#
# This script will look in standard locations for installed jovian. However, if you
# install jovian into a non-standard location, you can use the JOVIAN_ROOT
# variable (in environment or CMake) to specify the location.
#
# You can also use jovian out of a source tree by specifying JOVIAN_SOURCE_DIR
# and JOVIAN_BUILD_DIR (in environment or CMake).


SET( JOVIAN_BUILD_DIR "" CACHE PATH "If using jovian out of a source tree, specify the build directory." )
SET( JOVIAN_SOURCE_DIR "" CACHE PATH "If using jovian out of a source tree, specify the root of the source tree." )
SET( JOVIAN_ROOT "" CACHE PATH "Specify non-standard jovian install directory. It is the parent of the include and lib dirs." )

MACRO( FIND_JOVIAN_INCLUDE THIS_JOVIAN_INCLUDE_DIR THIS_JOVIAN_INCLUDE_FILE )
    UNSET( ${THIS_JOVIAN_INCLUDE_DIR} )
    MARK_AS_ADVANCED( ${THIS_JOVIAN_INCLUDE_DIR} )
    MESSAGE("${JOVIAN_SOURCE_DIR}")
    FIND_PATH( ${THIS_JOVIAN_INCLUDE_DIR} ${THIS_JOVIAN_INCLUDE_FILE}
        PATHS
            ${JOVIAN_ROOT}
            $ENV{JOVIAN_ROOT}
            ${JOVIAN_SOURCE_DIR}/src
            $ENV{JOVIAN_SOURCE_DIR}/src
            /usr/local
            /usr
            /sw/ # Fink
            /opt/local # DarwinPorts
            /opt/csw # Blastwave
            /opt
            "C:/Program Files/jovian"
            "C:/Program Files (x86)/jovian"
            ~/Library/Frameworks
            /Library/Frameworks
    )
ENDMACRO( FIND_JOVIAN_INCLUDE THIS_JOVIAN_INCLUDE_DIR THIS_JOVIAN_INCLUDE_FILE )

FIND_JOVIAN_INCLUDE( JOVIAN_INCLUDE_DIR Globals.h )
message( STATUS ${JOVIAN_INCLUDE_DIR} )

MACRO( FIND_JOVIAN_LIBRARY MYLIBRARY MYLIBRARYNAME )
    UNSET( ${MYLIBRARY} CACHE )
    UNSET( ${MYLIBRARY}_debug CACHE )
    MARK_AS_ADVANCED( ${MYLIBRARY} )
    MARK_AS_ADVANCED( ${MYLIBRARY}_debug )
    MESSAGE("${MYLIBRARY}")
    FIND_LIBRARY( ${MYLIBRARY}
        NAMES
            ${MYLIBRARYNAME}
        PATHS
            ${JOVIAN_ROOT}
            $ENV{JOVIAN_ROOT}
            ${JOVIAN_BUILD_DIR}
            $ENV{JOVIAN_BUILD_DIR}
            ~/Library/Frameworks
            /Library/Frameworks
            /usr/local
            /usr
            /sw
            /opt/local
            /opt/csw
            /opt
            "C:/Program Files/jovian"
            "C:/Program Files (x86)/jovian"
            /usr/freeware/lib64
        PATH_SUFFIXES
            lib
            lib/Release
            lib/RelWithDebInfo
            .
    )
    FIND_LIBRARY( ${MYLIBRARY}_debug
        NAMES
            ${MYLIBRARYNAME}d
        PATHS
            ${JOVIAN_ROOT}
            $ENV{JOVIAN_ROOT}
            ${JOVIAN_BUILD_DIR}
            $ENV{JOVIAN_BUILD_DIR}
            ~/Library/Frameworks
            /Library/Frameworks
            /usr/local
            /usr
            /sw
            /opt/local
            /opt/csw
            /opt
            "C:/Program Files/jovian"
            "C:/Program Files (x86)/jovian"
            /usr/freeware/lib64
        PATH_SUFFIXES
            lib
            lib/Debug
            .
    )
    message( STATUS ${${MYLIBRARY}} ${${MYLIBRARY}_debug} )
    message( STATUS ${MYLIBRARYNAME} )
    IF( ${MYLIBRARY} )
        SET( JOVIAN_LIBRARIES ${JOVIAN_LIBRARIES}
            "optimized" ${${MYLIBRARY}}
        )
    ENDIF( ${MYLIBRARY} )
    IF( ${MYLIBRARY}_debug )
        SET( JOVIAN_LIBRARIES ${JOVIAN_LIBRARIES}
            "debug" ${${MYLIBRARY}_debug}
        )
    ENDIF( ${MYLIBRARY}_debug )
ENDMACRO(FIND_JOVIAN_LIBRARY LIBRARY LIBRARYNAME)

FIND_JOVIAN_LIBRARY( JOVIAN_LIBRARY Jovian )

SET( JOVIAN_FOUND 0 )
IF( JOVIAN_LIBRARY AND JOVIAN_INCLUDE_DIR )
    SET( JOVIAN_FOUND 1 )
ENDIF( JOVIAN_LIBRARY AND JOVIAN_INCLUDE_DIR )
IF( JOVIAN_LIBRARY_debug AND JOVIAN_INCLUDE_DIR )
    SET( JOVIAN_FOUND 1 )
ENDIF( JOVIAN_LIBRARY_debug AND JOVIAN_INCLUDE_DIR )
