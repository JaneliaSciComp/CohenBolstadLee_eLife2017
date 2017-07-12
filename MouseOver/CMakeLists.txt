set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE)

IF(WIN32)
    CMAKE_MINIMUM_REQUIRED(VERSION 2.4.6 FATAL_ERROR)
ELSE(WIN32)
    IF(APPLE)
        CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0 FATAL_ERROR)
    ELSE(APPLE)
        CMAKE_MINIMUM_REQUIRED(VERSION 2.4.4 FATAL_ERROR)
    ENDIF(APPLE)
ENDIF(WIN32)

if(COMMAND cmake_policy)
    # Works around warnings libraries linked against that don't
    # have absolute paths (e.g. -lpthreads)
    cmake_policy(SET CMP0003 NEW)

    # Works around warnings about escaped quotes in ADD_DEFINITIONS
    # statements.
    cmake_policy(SET CMP0005 NEW)

    # cmake-2.6.1 introduces policy cmp0008 decide how to treat full path libraries that do not appear to be valid library file names
    # quote from cvslog "Such libraries worked by accident in the VS IDE and Xcode generators in CMake 2.4 and below."
    if(${CMAKE_MAJOR_VERSION} EQUAL 2 AND ${CMAKE_MINOR_VERSION} GREATER 4 AND ${CMAKE_PATCH_VERSION} GREATER 0)
        cmake_policy(SET CMP0008 OLD)
    endif()
endif()

PROJECT(MouseOver)

SET(MOUSEOVER_MAJOR_VERSION 0)
SET(MOUSEOVER_MINOR_VERSION 1)
SET(MOUSEOVER_PATCH_VERSION 0)
SET(MOUSEOVER_SOVERSION 60)

# set to 0 when not a release candidate, non zero means that any generated
# svn tags will be treated as release candidates of given number
SET(MOUSEOVER_RELEASE_CANDIDATE 0)

SET(MOUSEOVER_VERSION ${MOUSEOVER_MAJOR_VERSION}.${MOUSEOVER_MINOR_VERSION}.${MOUSEOVER_PATCH_VERSION})

# We have some custom .cmake scripts not in the official distribution.
# Maybe this can be used override existing behavior if needed?
SET(CMAKE_MODULE_PATH "${MouseOver_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")

FIND_PACKAGE(Jovian)

IF(JOVIAN_FOUND)
	SET(CMAKE_MODULE_PATH "${JOVIAN_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")
# Find OpenGL
FIND_PACKAGE(OpenGL)
#SET(OpenSceneGraph_DEBUG TRUE)

INCLUDE( CMakeMacros )
include( FindOSGHelper )
include( FindBulletHelper )

FIND_PACKAGE( OSGWorks REQUIRED )
FIND_PACKAGE( OSGBullet REQUIRED )

# add bullet & osg for entire project for include & lib paths
IF  (DESIRED_QT_VERSION)
  IF  (DESIRED_QT_VERSION MATCHES 5)
    FIND_PACKAGE(Qt5Widgets)
  ELSEIF (DESIRED_QT_VERSION MATCHES 4)
    FIND_PACKAGE(Qt4)
  ELSE()
    FIND_PACKAGE(Qt3)
  ENDIF()

ELSE()

  FIND_PACKAGE(Qt4)

  IF (NOT QT4_FOUND)
    find_package(Qt5Core)
    find_package(Qt5OpenGL)
    find_package(Qt5PrintSupport)
    find_package(Qt5SerialPort)
    find_package(Qt5Widgets)
  ELSE()
    FIND_PACKAGE(QtSerialPort)
  ENDIF()
ENDIF()

SET(Boost_USE_MULTITHREADED ON)
find_path( BOOST_ROOT "Boost" DOC "Root for the Boost libraries" )
SET(Boost_ADDITIONAL_VERSIONS "1.54" "1.54.0" "1.55" "1.55.0" "1.56" "1.56.0" "1.57" "1.57.0" "1.58" "1.58.0")
FIND_PACKAGE(Boost 1.53 COMPONENTS system thread program_options date_time filesystem regex chrono)

IF(APPLE)
    FIND_LIBRARY(CARBON_LIBRARY Carbon)
    FIND_LIBRARY(COCOA_LIBRARY Cocoa)
ENDIF()

IF(UNIX)
    # Not sure what this will do on Cygwin and Msys
    # Also, remember OS X X11 is a user installed option so it may not exist.
    FIND_PACKAGE(X11)
    # Some Unicies need explicit linkage to the Math library or the build fails.
    FIND_LIBRARY(MATH_LIBRARY m)
ENDIF()

# Make the headers visible to everything
IF(NOT ${PROJECT_BINARY_DIR} EQUAL ${PROJECT_SOURCE_DIR})
   INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/include)
ENDIF()

INCLUDE_DIRECTORIES(
	  ${JOVIAN_INCLUDE_DIR}
    ${OSG_INCLUDE_DIRS}
    ${OSGWORKS_INCLUDE_DIR}
    ${OSGBULLET_INCLUDE_DIR}
    ${BULLET_INCLUDE_DIRS}
    ${BULLET_EXTRAS_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
)

macro(get_WIN32_WINNT version)
    if (WIN32 AND CMAKE_SYSTEM_VERSION)
        set(ver ${CMAKE_SYSTEM_VERSION})
        string(REPLACE "." "" ver ${ver})
        string(REGEX REPLACE "([0-9])" "0\\1" ver ${ver})

        set(${version} "0x${ver}")
    endif()
endmacro()

if (WIN32)
    get_WIN32_WINNT(ver)
    add_definitions(-D_WIN32_WINNT=${ver})
endif()

# Common global definitions
#ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK)
# Platform specific definitions


IF(WIN32)

    IF(MSVC)
        # turn off various warnings
        # foreach(warning 4244 4251 4267 4275 4290 4786 4305 4996)
        #     SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd${warning}")
        # endforeach(warning)

        # This option is to enable the /DYNAMICBASE switch
        # It is used to workaround a bug in Windows 7 when linking in release, which results in corrupt
        # binaries. See this page for details: http://www.wintellect.com/CS/blogs/jrobbins/archive/2009/01/24/the-case-of-the-corrupt-pe-binaries.aspx
        OPTION(WIN32_USE_DYNAMICBASE "Set to ON to build OpenSceneGraph with the /DYNAMICBASE option to work around a bug when linking release executables on Windows 7." OFF)
        MARK_AS_ADVANCED(WIN32_USE_DYNAMICBASE)
        IF(WIN32_USE_DYNAMICBASE)
            SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /DYNAMICBASE")
        ENDIF(WIN32_USE_DYNAMICBASE)

        # More MSVC specific compilation flags
        ADD_DEFINITIONS(-D_SCL_SECURE_NO_WARNINGS)
        ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)

        OPTION(MSVC_DISABLE_CHECKED_ITERATORS "Set to ON to disable Visual C++ checked iterators. If you do this you must ensure that every other project in your solution and all dependencies are compiled with _SECURE_SCL=0." OFF)
        MARK_AS_ADVANCED(MSVC_DISABLE_CHECKED_ITERATORS)
        IF(MSVC_DISABLE_CHECKED_ITERATORS)
            ADD_DEFINITIONS(-D_SECURE_SCL=0)
        ENDIF(MSVC_DISABLE_CHECKED_ITERATORS)
    ENDIF()

    #needed for net plugin
    SET (OSG_SOCKET_LIBS wsock32)
    # Both Cygwin and Msys need -DNOMINMAX ???
    IF(UNIX)
        ADD_DEFINITIONS(-DNOMINMAX)
    ENDIF()

########################################################################################################
# the following options are MSVC specific,
# the first OSG_MSVC_VERSIONED_DLL activate a custom build-time layout that should allow to run examples and application
# fron bin folder without requiring installation step.
# it also prepend "osg${OPENSCENEGRAPH_SOVERSION}-" to only .dll files, leaving .lib files untouched in lib
# it also use a hack to get rid of Debug and Release folder in MSVC projects
# all the .dll and .pdb are in bin and all the .lib and .exp are in lib
#
# the second option disable incremental linking in debug build , that is enabled by default by CMake
##########################################################################################################

    IF(MSVC)
        IF(${CMAKE_MAJOR_VERSION} EQUAL 2 AND ${CMAKE_MINOR_VERSION} EQUAL 4 AND ${CMAKE_PATCH_VERSION} LESS 7)
            MESSAGE("Warning:  disabling versioned options 2.4.6 exibits inconsintencies in .pdb naming, at least under MSVC, suggested upgrading at least to 2.4.7")
            SET(OSG_MSVC_VERSIONED_DLL OFF)
            SET(OSG_MSVC_DEBUG_INCREMENTAL_LINK ON)
        ELSE()
            OPTION(OSG_MSVC_VERSIONED_DLL "Set to ON to build OpenSceneGraph with versioned dll names" ON)
            MARK_AS_ADVANCED(OSG_MSVC_VERSIONED_DLL)
            OPTION(OSG_MSVC_DEBUG_INCREMENTAL_LINK "Set to OFF to build OpenSceneGraph without incremental linking in debug (release is off by default)" ON)
            MARK_AS_ADVANCED(OSG_MSVC_DEBUG_INCREMENTAL_LINK)
            IF(NOT OSG_MSVC_DEBUG_INCREMENTAL_LINK)
                SET(CMAKE_MODULE_LINKER_FLAGS_DEBUG "/debug /INCREMENTAL:NO")
                SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "/debug /INCREMENTAL:NO")
                SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "/debug /INCREMENTAL:NO")
            ENDIF(NOT OSG_MSVC_DEBUG_INCREMENTAL_LINK)
        ENDIF()
    ENDIF(MSVC)
ENDIF(WIN32)

OPTION(OSG_DISABLE_MSVC_WARNINGS "Set to OFF to not disable MSVC warnings generated by OSG headers." ON)
MARK_AS_ADVANCED(OSG_DISABLE_MSVC_WARNINGS)

################################################################################
# Set Version Info resource file

IF(MSVC)
    SET(OPENSCENEGRAPH_VERSIONINFO_RC "${PROJECT_BINARY_DIR}/PlatformSpecifics/Windows/OpenSceneGraphVersionInfo.rc")
    CONFIGURE_FILE("${CMAKE_CURRENT_SOURCE_DIR}/PlatformSpecifics/Windows/OpenSceneGraphVersionInfo.rc.in"
                   "${OPENSCENEGRAPH_VERSIONINFO_RC}")
ENDIF()

################################################################################
# Optional build components


################################################################################
# 3rd Party Dependency Stuff
IF(WIN32)
    INCLUDE(Find3rdPartyDependencies)
ENDIF()

# Common to all platforms:
FIND_PACKAGE(OSG)
################################################################################
# Create bin and lib directories if required

IF("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
   FILE(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin ${CMAKE_BINARY_DIR}/lib ${CMAKE_BINARY_DIR}/lib/${OSG_PLUGINS})
ENDIF()


################################################################################
# Installation stuff

SET(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "add a postfix, usually d on windows")
ADD_DEFINITIONS(-DOSG_DEBUG_POSTFIX=${CMAKE_DEBUG_POSTFIX})

IF(UNIX AND NOT WIN32)
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")
  SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D_DEBUG")
ENDIF()

IF(CYGWIN)
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")
  SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D_DEBUG")
ENDIF()

IF(UNIX AND NOT WIN32 AND NOT APPLE)
  IF(CMAKE_SIZEOF_VOID_P MATCHES "8")
      SET(LIB_POSTFIX "64" CACHE STRING "suffix for 32/64 dir placement")
      MARK_AS_ADVANCED(LIB_POSTFIX)
  ENDIF()
ENDIF()
IF(NOT DEFINED LIB_POSTFIX)
    SET(LIB_POSTFIX "")
ENDIF()

# Here we apparantly do some funky stuff with making the bin/ and lib/
# folders which is probably needed to work around a very old CMake bug?

#SET(OUTPUT_BINDIR ${PROJECT_BINARY_DIR}/bin/${CMAKE_SYSTEM_NAME})
SET(OUTPUT_BINDIR ${PROJECT_BINARY_DIR}/bin)
MAKE_DIRECTORY(${OUTPUT_BINDIR})
IF(MSVC AND NOT MSVC_IDE)
    MAKE_DIRECTORY(${OUTPUT_BINDIR}/${OSG_PLUGINS})
ENDIF(MSVC AND NOT MSVC_IDE)

#SET(OUTPUT_LIBDIR ${PROJECT_BINARY_DIR}/lib/${CMAKE_SYSTEM_NAME})
SET(OUTPUT_LIBDIR ${PROJECT_BINARY_DIR}/lib)
MAKE_DIRECTORY(${OUTPUT_LIBDIR})
IF(NOT MSVC)
    MAKE_DIRECTORY(${OUTPUT_LIBDIR}/${OSG_PLUGINS})
ENDIF(NOT MSVC)

# On CMake 2.4.x use EXECUTABLE_OUTPUT_PATH and LIBRARY_OUTPUT_PATH and later
# we work around the DLL placement by use of the PREFIX target property hack
#
# On CMake 2.6.x use the newly minted CMAKE_LIBRARY_OUTPUT_DIRECTORY,
# CMAKE_ARCHIVE_OUTPUT_DIRECTORY & CMAKE_RUNTIME_OUTPUT_DIRECTORY

IF(CMAKE_MAJOR_VERSION EQUAL 2 AND CMAKE_MINOR_VERSION LESS 5)
    SET(EXECUTABLE_OUTPUT_PATH ${OUTPUT_BINDIR})
    SET(LIBRARY_OUTPUT_PATH    ${OUTPUT_LIBDIR})
ELSE()
    # If CMake >= 2.6.0
    SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_LIBDIR})
    SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_BINDIR})
    IF(WIN32)
        SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_BINDIR})
    ELSE(WIN32)
        SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_LIBDIR})
    ENDIF(WIN32)
ENDIF()

#SET(INSTALL_BINDIR OpenSceneGraph/bin)
#SET(INSTALL_INCDIR OpenSceneGraph/include)
#SET(INSTALL_LIBDIR OpenSceneGraph/lib)
#SET(INSTALL_DOCDIR OpenSceneGraph/doc)

################################################################################
# User Options


# Expose CMAKE_INCLUDE_PATH and CMAKE_LIBARY_PATH to the GUI so users
# may set these values without needing to manipulate the environment.
SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} CACHE STRING "You may add additional search paths here. Use ; to separate multiple paths.")
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} CACHE STRING "You may add additional search paths here. Use ; to separate multiple paths.")
# We are proposing that a new variable called CMAKE_PREFIX_PATH be introduced
# to CMake to compliment CMAKE_INCLUDE_PATH and CMAKE_LIBRARY_PATH.
# A formal feature request has been submited to CMake, Bug #4947.
# It is intended for those users who have common prefixes for their INCLUDE
# and LIBRARY locations. So if users have headers in /usr/local/include
# and libraries in /usr/local/lib, the common prefix is /usr/local.
# It should also cover the case where headers and libraries are
# in the same directory.
# Our proposal expects that FIND_* commands will automatically search for
# CMAKE_PREFIX_PATH right after CMAKE_INCLUDE_PATH or CMAKE_LIBRARY_PATH.
# Obviously, since CMake does not currently support this, we must write
# our Find*.cmake modules to explicitly support this. Otherwise, this variable
# will have no impact.
# This is unofficial so this may be removed or changed at anytime.
SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} CACHE STRING "(EXPERIMENTAL) You may add additional search paths here. Use ; to separate multiple paths.")

# This is for an advanced option to give aggressive warnings
# under different compilers. If yours is not implemented, this option
# will not be made available.
IF(CMAKE_COMPILER_IS_GNUCXX)
    # To be complete, we might also do GNUCC flags,
    # but everything here is C++ code.
    # -Wshadow and -Woverloaded-virtual are also interesting flags, but OSG
    # returns too many hits.
    # FYI, if we do implement GNUCC, then -Wmissing-prototypes in another
    # interesting C-specific flag.
    # Also, there is a bug in gcc 4.0. Under C++, -pedantic will create
    # errors instead of warnings for certain issues, including superfluous
    # semicolons and commas, and the use of long long. -fpermissive seems
    # to be the workaround.
    SET(OSG_AGGRESSIVE_WARNING_FLAGS -Wall -Wparentheses -Wno-long-long -Wno-import -pedantic -Wreturn-type -Wmissing-braces -Wunknown-pragmas -Wunused -fpermissive)

    # Previous included -Wformat=2 in OSG_AGGRESSIVE_WARNING_FLAGS but had to remove it due to standard library errors


ELSE()
    IF(MSVC)
        # FIXME: What are good aggressive warning flags for Visual Studio?
        # And do we need to further subcase this for different versions of VS?
        # CMake variables: MSVC60, MSVC70, MSVC71, MSVC80, CMAKE_COMPILER_2005
        SET(OSG_AGGRESSIVE_WARNING_FLAGS /W4 /wd4706 /wd4127 /wd4100)


    ELSE()
        # CMake lacks an elseif, so other non-gcc, non-VS compilers need
        # to be listed below. If unhandled, OSG_AGGRESSIVE_WARNING_FLAGS should
        # remain unset.
    ENDIF()
ENDIF()

# This part is for the CMake menu option to toggle the warnings on/off.
# This will only be made available if we set values for OSG_AGGRESSIVE_WARNING_FLAGS.
IF(OSG_AGGRESSIVE_WARNING_FLAGS)

    IF (APPLE)
        SET(DEFAULT_USE_AGGRESSIVE_WARNINGS OFF)
    ELSE()
        SET(DEFAULT_USE_AGGRESSIVE_WARNINGS ON)
    ENDIF()

    OPTION(OSG_USE_AGGRESSIVE_WARNINGS "Enable to activate aggressive warnings" ${DEFAULT_USE_AGGRESSIVE_WARNINGS})
    MARK_AS_ADVANCED(OSG_USE_AGGRESSIVE_WARNINGS)

    IF(OSG_USE_AGGRESSIVE_WARNINGS)
        # Add flags defined by OSG_AGGRESSIVE_WARNING_FLAGS if they aren't already there
        FOREACH(flag ${OSG_AGGRESSIVE_WARNING_FLAGS})
            IF(NOT CMAKE_CXX_FLAGS MATCHES "${flag}")
                SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${flag}")
            ENDIF()
        ENDFOREACH()
    ELSE()
        # Remove all flags considered aggresive
        FOREACH(flag ${OSG_AGGRESSIVE_WARNING_FLAGS})
            STRING(REGEX REPLACE "${flag}" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        ENDFOREACH()
    ENDIF()
ENDIF()

#INCLUDE(OsgMacroUtils)
ADD_SUBDIRECTORY(src)

# Set defaults for Universal Binaries. We want 32-bit Intel/PPC on 10.4
# and 32/64-bit Intel/PPC on >= 10.5. Anything <= 10.3 doesn't support.
IF(APPLE)
    # These are just defaults/recommendations, but how we want to build
    # out of the box. But the user needs to be able to change these options.
    # So we must only set the values the first time CMake is run, or we
    # will overwrite any changes the user sets.
    # FORCE is used because the options are not reflected in the UI otherwise.
    # Seems like a good place to add version specific compiler flags too.
    IF(NOT OSG_CONFIG_HAS_BEEN_RUN_BEFORE)
        # This is really fragile, but CMake doesn't provide the OS system
        # version information we need. (Darwin versions can be changed
        # independently of OS X versions.)
        # It does look like CMake handles the CMAKE_OSX_SYSROOT automatically.
        IF(EXISTS /Developer/SDKs/MacOSX10.5.sdk)
            # 64-bit compiles are not supported with Carbon. We should enable
            # 64-bit compilation by default once osgviewer has been
            # rewritten with Cocoa.
            #SET(CMAKE_OSX_ARCHITECTURES "ppc;i386;ppc64;x86_64" CACHE STRING "Build architectures for OSX" FORCE)
            SET(CMAKE_OSX_ARCHITECTURES "i386" CACHE STRING "Build architectures for OSX" FORCE)
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmacosx-version-min=10.5 -ftree-vectorize -fvisibility-inlines-hidden" CACHE STRING "Flags used by the compiler during all build types." FORCE)
        ELSEIF(EXISTS /Developer/SDKs/MacOSX10.4u.sdk)
            SET(CMAKE_OSX_ARCHITECTURES "ppc;i386" CACHE STRING "Build architectures for OSX" FORCE)
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmacosx-version-min=10.4 -ftree-vectorize -fvisibility-inlines-hidden" CACHE STRING "Flags used by the compiler during all build types." FORCE)
        ELSE()
            # No Universal Binary support
            # Should break down further to set the -mmacosx-version-min,
            # but the SDK detection is too unreliable here.
        ENDIF()
    ENDIF(NOT OSG_CONFIG_HAS_BEEN_RUN_BEFORE)

ENDIF(APPLE)



# Run this as late as possible so users can easier spot the message
IF (NOT DEFINED REQUIRES_LIBPATH_MESSAGE AND ${CMAKE_INSTALL_PREFIX} STREQUAL "/usr/local")
    SET(REQUIRES_LIBPATH_MESSAGE ON)
ENDIF()

IF(REQUIRES_LIBPATH_MESSAGE)
    IF (NOT OSG_LIBPATH_MESSAGE_HAS_BEEN_RUN_BEFORE)
        SET(OSG_LIBPATH_MESSAGE_HAS_BEEN_RUN_BEFORE 1 CACHE INTERNAL "Flag to track whether the libpath message has been reported before")

        MESSAGE("\nThe build system is configured to instal libraries to ${CMAKE_INSTALL_PREFIX}/lib${LIB_POSTFIX}\n"
            "Your applications may not be able to find your installed libraries unless you:\n"
            "    set your LD_LIBRARY_PATH (user specific) or\n"
            "    update your ld.so configuration (system wide)")
        IF(IS_DIRECTORY /etc/ld.so.conf.d)
            MESSAGE("You have an ld.so.conf.d directory on your system, so if you wish to ensure that\n"
                "applications find the installed osg libraries, system wide, you could install a\n"
                "openscenegraph specific ld.so configuration with:\n"
                "    sudo make install_ld_conf\n")
            CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/packaging/ld.so.conf.d/openscenegraph.conf.in
                ${PROJECT_BINARY_DIR}/packaging/ld.so.conf.d/openscenegraph.conf
            )
            ADD_CUSTOM_TARGET(install_ld_conf ${CMAKE_COMMAND} -E copy_if_different
                ${PROJECT_BINARY_DIR}/packaging/ld.so.conf.d/openscenegraph.conf
                /etc/ld.so.conf.d/openscenegraph.conf
                COMMAND ldconfig
                COMMENT "Copying openscenegraph.conf to /etc/ld.so.conf.d and running ldconfig"
            )
        ELSE()
            IF(EXISTS /etc/ld.so.conf)
                MESSAGE("You have an ld.so.conf file in /etc, if you wish to ensure \n"
                "that applications find the installed osg libraries, system wide, you\n"
                "could add ${CMAKE_INSTALL_PREFIX}/lib${LIB_POSTFIX} to it.")
            ENDIF()
        ENDIF()

        # emit a message during installation.
        INSTALL(CODE "MESSAGE(\"Libraries were installed to ${CMAKE_INSTALL_PREFIX}lib${LIB_POSTFIX}.\\nYou may need to update your ld.so configuration. \")")

    ENDIF(NOT OSG_LIBPATH_MESSAGE_HAS_BEEN_RUN_BEFORE)

ELSE()
    SET(OSG_LIBPATH_MESSAGE_HAS_BEEN_RUN_BEFORE 0 CACHE INTERNAL "Flag to track whether the libpath message has been reported before")
ENDIF()


# This needs to be run very last so other parts of the scripts can take
# advantage of this.
IF(NOT OSG_CONFIG_HAS_BEEN_RUN_BEFORE)
    SET(OSG_CONFIG_HAS_BEEN_RUN_BEFORE 1 CACHE INTERNAL "Flag to track whether this is the first time running CMake or if CMake has been configured before")
ENDIF()

#-----------------------------------------------------------------------------
### uninstall target
#-----------------------------------------------------------------------------
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

ENDIF()

