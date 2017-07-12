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

PROJECT(RemoteDataServer)

SET(REMOTEDATASERVER_MAJOR_VERSION 0)
SET(REMOTEDATASERVER_MINOR_VERSION 1)
SET(REMOTEDATASERVER_PATCH_VERSION 0)
SET(REMOTEDATASERVER_SOVERSION 60)

# set to 0 when not a release candidate, non zero means that any generated
# svn tags will be treated as release candidates of given number
SET(REMOTEDATASERVER_RELEASE_CANDIDATE 0)

SET(REMOTEDATASERVER_VERSION ${REMOTEDATASERVER_MAJOR_VERSION}.${REMOTEDATASERVER_MINOR_VERSION}.${REMOTEDATASERVER_PATCH_VERSION})

# We have some custom .cmake scripts not in the official distribution.
# Maybe this can be used override existing behavior if needed?
SET(CMAKE_MODULE_PATH "${RemoteDataServer_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")

message("${CMAKE_MODULE_PATH}")

FIND_PACKAGE(Jovian)

IF(JOVIAN_FOUND)
	SET(CMAKE_MODULE_PATH "${JOVIAN_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")

	INCLUDE( CMakeMacros )
	ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK)

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
	  # Some Unicies need explicit linkage to the Math library or the build fails.
	  FIND_LIBRARY(MATH_LIBRARY m)
	ENDIF()

	# Make the headers visible to everything
	IF(NOT ${PROJECT_BINARY_DIR} EQUAL ${PROJECT_SOURCE_DIR})
	  INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/include)
	ENDIF()

	# Common global definitions
	#ADD_DEFINITIONS(-D)
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

	  # Both Cygwin and Msys need -DNOMINMAX ???
	  IF(UNIX)
            ADD_DEFINITIONS(-DNOMINMAX)
	  ENDIF()
	ENDIF()


	################################################################################
	# Create bin and lib directories if required

	IF("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
	  FILE(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin ${CMAKE_BINARY_DIR}/lib )
	ENDIF()


	################################################################################
	# Installation stuff

	SET(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "add a postfix, usually d on windows")

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

	#SET(OUTPUT_LIBDIR ${PROJECT_BINARY_DIR}/lib/${CMAKE_SYSTEM_NAME})
	SET(OUTPUT_LIBDIR ${PROJECT_BINARY_DIR}/lib)
	MAKE_DIRECTORY(${OUTPUT_LIBDIR})

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

	INCLUDE(OsgMacroUtils)
	ADD_SUBDIRECTORY(src)

	# Set defaults for Universal Binaries. We want 32-bit Intel/PPC on 10.4
	# and 32/64-bit Intel/PPC on >= 10.5. Anything <= 10.3 doesn't support.
	IF(APPLE)
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

	ENDIF(APPLE)



	# Run this as late as possible so users can easier spot the message
	IF (NOT DEFINED REQUIRES_LIBPATH_MESSAGE AND ${CMAKE_INSTALL_PREFIX} STREQUAL "/usr/local")
	  SET(REQUIRES_LIBPATH_MESSAGE ON)
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

