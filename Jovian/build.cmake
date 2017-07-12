#!/bin/bash

# Run without arguments to see usage.

function download {
    if [[ ! -e $2 ]]; then
        echo "Downloading $1"
        curl -L $1 -o $2
    fi
}

shopt -s expand_aliases;
BOOST_VERSION=1_58_0
CMAKE_VERSION=3.1.3
CMAKE_ARGS=""
CMAKE_PLATFORM_ARGS=""
CMAKE_BUILD="Release"
CMAKE_EXE=""
FORCE=0
BUILD_DIR=`pwd`
ROOT_DIR=`pwd`
USE_QT4=0

set -eu

KERNEL=(`uname -s | tr [A-Z] [a-z]`)
ARCH=(`uname -m | tr [A-Z] [a-z]`)
case $KERNEL in
    darwin)
        OS=macosx
        ;;
    mingw*)
        OS=windows
        KERNEL=windows
        ;;
    windows-x86_64)
        ;;
    *)
        OS=$KERNEL
        ;;
esac
case $ARCH in
    arm*)
        ARCH=arm
        ;;
    i386|i486|i586|i686)
        ARCH=x86
        ;;
    amd64|x86-64)
        ARCH=x86_64
        ;;
esac
PLATFORM=$OS-$ARCH
echo "Detected platform \"$PLATFORM\""

while [[ $# > 0 ]]; do
    case "$1" in
        -platform)
            shift
            PLATFORM="$1"
            ;;
        -qt4)
            CMAKE_ARGS+=" -DFORCE_QT4:BOOL=OFF"
            USE_QT4=1
            ;;
        -f)
            FORCE=1
            ;;
        -debug)
            CMAKE_PLATFORM_ARGS+=" -DCMAKE_CXX_FLAGS_RELEASE:STRING=\" /MD /Zi /Od /Ob0 /DEBUG\""
            CMAKE_PLATFORM_ARGS+=" -DCMAKE_C_FLAGS_RELEASE:STRING=\" /MD /Zi /Od /Ob0 /DEBUG\""
            CMAKE_PLATFORM_ARGS+=" -DCMAKE_EXE_LINKER_FLAGS_RELEASE:STRING=\" /debug /INCREMENTAL:NO\""
            CMAKE_PLATFORM_ARGS+=" -DCMAKE_MODULE_LINKER_FLAGS_RELEASE:STRING=\" /debug /INCREMENTAL:NO\""
            CMAKE_PLATFORM_ARGS+=" -DCMAKE_SHARED_LINKER_FLAGS_RELEASE:STRING=\" /debug /INCREMENTAL:NO\""
            ;;
        install)
            OPERATION=install
            ;;
        clean)
            OPERATION=clean
            ;;
        clobber)
            OPERATION=clobber
            ;;
        *)
            BUILD_DIR="$1"
            ;;
    esac
    shift
done
echo "Targeting platform \"$PLATFORM\""
echo "Root directory \"$ROOT_DIR\""
echo "Build directory \"$BUILD_DIR\""

if [[ -z ${OPERATION:-} ]]; then
    echo "Usage: build.cmake [-platform <name>] [-qt4] [-f] [-debug] <install | clean | clobber> <build directory>"
    echo "where possible platform names are: linux-x86, linux-x86_64, macosx-x86_64, windows-x86, windows-x86_64, etc."
    echo " -qt4 - build with Qt4 (Qt5 is the default)"
    echo " -f - force rebuilding of components"
    echo " -debug - Generates a debug build (default is release)"
    echo " clean - removes the current build for platform"
    echo " clobber - cleans, and removes the current cmake directories"
    exit 1
fi

if [ $PLATFORM = "windows-x86_64" ]; then
    ARCH=x86_64
fi

cd $BUILD_DIR

: "${CMAKE_DIR:=""}"

case $OPERATION in
    install)
        # See if the CMAKE_DIR is set
        if [ ! "$CMAKE_DIR" = "" ]; then
            if [[ -e $CMAKE_DIR ]]; then
                CMAKE_EXE="$CMAKE_DIR/bin/cmake"
            fi
        fi

        # If CMAKE_EXE is not set, then either find or build cmake
        if [ "$CMAKE_EXE" = "" ]; then
            if hash cmake 2>/dev/null; then
                CMAKE_EXE="cmake"
            else
        		if [[ ! -e cmake-$CMAKE_VERSION/bin/cmake ]]; then
        			if [[ ! -e cmake-$CMAKE_VERSION ]]; then
        				echo "Downloading cmake"
        				download http://www.cmake.org/files/v3.1/cmake-$CMAKE_VERSION.tar.gz cmake-$CMAKE_VERSION.tar.gz
        				tar xvzf cmake-$CMAKE_VERSION.tar.gz
        			fi
        			cd cmake-$CMAKE_VERSION
        			./configure --prefix=.
        			make
        			make install
        			cd ..
        		fi
                CMAKE_EXE="../cmake-$CMAKE_VERSION/bin/cmake"
            fi
        fi

        echo "Using $CMAKE_EXE"

		if [[ ! -e build_$PLATFORM ]]; then
			mkdir build_$PLATFORM
		fi

        if [ $PLATFORM = "windows-x86_64" ]; then
            CMAKE_EXE+=" -G \"Visual Studio 12 2013 Win64\""
        else
            CMAKE_EXE+=" -G \"Visual Studio 12 2013\""
        fi

        BOOST_ROOT="C:\Program Files\boost_$BOOST_VERSION"
        cd build_$PLATFORM
#############################
##       Build Boost       ##
#############################
        if [[ ! -e "C:\Program Files\boost_$BOOST_VERSION" ]]; then
            boost_dot_version=`echo $BOOST_VERSION | awk '{gsub(/_/,".")}; 1'`
            download http://sourceforge.net/projects/boost/files/boost/$boost_dot_version/boost_$BOOST_VERSION.tar.gz/download boost_$BOOST_VERSION.tar.gz
            if [[ ! -e boost_$BOOST_VERSION || $FORCE -eq 1 ]]; then
                tar xvzf boost_$BOOST_VERSION.tar.gz
            fi
            cd boost_$BOOST_VERSION
            cmd //c .\\bootstrap.bat --without-libraries=python
            cmd //c .\\b2.exe --toolset=msvc-12.0 address-model=64 --prefix="$BOOST_ROOT" --without-python --build-type=complete install
            cd ..
        fi

#############################
##      Build Collada      ##
#############################
        if [[ ! -e "C:\Program Files\collada-dom" || $FORCE -eq 1 ]]; then
            if [[ ! -e collada-dom ]]; then
                /c/Program\ Files/7-Zip/7z x -y $ROOT_DIR/external_libraries/collada-dom.tgz
                /c/Program\ Files/7-Zip/7z x -y collada-dom.tar
                rm -rf collada-dom.tar
            fi

            download http://www.mira-project.org/downloads/3rdparty/bin-win64/zlib-1.2.5.win64.zip zlib-1.2.5.win64.zip
            if [[ ! -e zlib-1.2.5.win64 ]]; then
                /c/Program\ Files/7-Zip/7z x zlib-1.2.5.win64.zip
            fi

            download http://www.mira-project.org/downloads/3rdparty/bin-win64/libxml2-2.7.8.win64.zip libxml2-2.7.8.win64.zip
            if [[ ! -e libxml2-2.7.8.win64 ]]; then
                /c/Program\ Files/7-Zip/7z x libxml2-2.7.8.win64.zip
            fi

            download http://www.mira-project.org/downloads/3rdparty/bin-win64/iconv-1.14.0.win64.zip iconv-1.14.0.win64.zip
            if [[ ! -e iconv-1.14.0.win64 ]]; then
                /c/Program\ Files/7-Zip/7z x iconv-1.14.0.win64.zip
            fi

            if [[ ! -e collada_dom ]]; then
                mkdir collada_dom
            fi
            cd collada_dom

            CMAKE_ARGS="-DBOOST_ROOT:PATH=\"$BOOST_ROOT\" "
            CMAKE_ARGS+="-DOPT_DOUBLE_PRECISION:BOOL=OFF "
            CMAKE_ARGS+="-DOPT_COLLADA15:BOOL=OFF "
            CMAKE_ARGS+="-DLIBXML2_INCLUDE_DIR:PATH=$BUILD_DIR/build_$PLATFORM/libxml2-2.7.8.win64/include "
            CMAKE_ARGS+="-DLIBXML2_LIBRARIES:FILEPATH=$BUILD_DIR/build_$PLATFORM/libxml2-2.7.8.win64/lib/libxml2.lib "
            CMAKE_ARGS+="-DZLIB_INCLUDE_DIR:PATH=$BUILD_DIR/build_$PLATFORM/zlib-1.2.5.win64/include "
            CMAKE_ARGS+="-DZLIB_LIBRARY:FILEPATH=$BUILD_DIR/build_$PLATFORM/zlib-1.2.5.win64/lib/zdll.lib "
            echo $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS --build collada_dom ../collada-dom
            eval $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS --build collada_dom ../collada-dom

            echo "Building collada-dom"
            devenv collada-dom.sln -project ALL_BUILD -build $CMAKE_BUILD -out collada-dom.txt
            echo "Installing collada-dom"
            devenv collada-dom.sln -project INSTALL -build $CMAKE_BUILD -out collada-dom_install.txt
            cd ..
            cp $BUILD_DIR/build_$PLATFORM/libxml2-2.7.8.win64/bin/libxml2.dll /c/Program\ Files/collada-dom/bin
            cp $BUILD_DIR/build_$PLATFORM/zlib-1.2.5.win64/zlib1.dll /c/Program\ Files/collada-dom/bin
            cp $BUILD_DIR/build_$PLATFORM/iconv-1.14.0.win64/bin/iconv.dll /c/Program\ Files/collada-dom/bin
        fi

        export COLLADA_DIR=/c/Program\ Files/collada-dom
        export PATH=$PATH:$COLLADA_DIR/bin:$COLLADA_DIR/lib

#############################
##        Build Qt4        ##
#############################
        if [[ $USE_QT4 -eq 1 && (! -e "C:\Qt\Qt4.8.6_x64" || $FORCE -eq 1) ]]; then
            download http://download.qt.io/archive/qt/4.8/4.8.6/qt-everywhere-opensource-src-4.8.6.tar.gz qt-everywhere-opensource-src-4.8.6.tar.gz
            if [[ ! -e "qt-everywhere-opensource-src-4.8.6" || $FORCE -eq 1 ]]; then
                /c/Program\ Files/7-Zip/7z x -y qt-everywhere-opensource-src-4.8.6.tar.gz
                /c/Program\ Files/7-Zip/7z x -y qt-everywhere-opensource-src-4.8.6.tar
                rm -rf qt-everywhere-opensource-src-4.8.6.tar
            fi
            if [[ ! -e qt_build ]]; then
                mkdir qt_build
            fi
            cd qt_build
            ../qt-everywhere-opensource-src-4.8.6/configure.exe -prefix /c/Qt/Qt4.8.6_x64 -debug-and-release -no-qt3support -no-multimedia -no-webkit -qt-zlib -qt-libpng -qt-libjpeg  -no-webkit -opensource -confirm-license -nomake examples -nomake demos -platform win32-msvc2013
            nmake
            nmake install
            cd ..
        fi

        if [[ $USE_QT4 -eq 1 ]]; then
            export PATH=/c/Qt/Qt4.8.6_x64/bin:/c/Qt/Qt4.8.6_x64/lib:$PATH
        fi

        if [[ $USE_QT4 -eq 1 ]]; then
            if [[ ! -e qtserialport ]]; then
                /c/Program\ Files/7-Zip/7z x -y $ROOT_DIR/external_libraries/qtserialport.tgz
                /c/Program\ Files/7-Zip/7z x -y qtserialport.tar
                rm -rf qtserialport.tar
            fi

            if [[ ! -e qtserialport_build ]]; then
                mkdir qtserialport_build
            fi
            cd qtserialport_build
            qmake -config release ../qtserialport/qtserialport.pro
            nmake
            nmake install
            cd ..
        fi

#############################
##  Build OpenSceneGraph   ##
#############################
        if [[ ! -e "C:\Program Files\OpenSceneGraph" || $FORCE -eq 1 ]]; then
            if [[ ! -e OpenSceneGraph ]]; then
                /c/Program\ Files/7-Zip/7z x -y $ROOT_DIR/external_libraries/OpenSceneGraph-Data.tgz
                /c/Program\ Files/7-Zip/7z x -y OpenSceneGraph-Data.tar
                rm -rf OpenSceneGraph-Data.tar
                /c/Program\ Files/7-Zip/7z x -y $ROOT_DIR/external_libraries/OpenSceneGraph.tgz
                /c/Program\ Files/7-Zip/7z x -y OpenSceneGraph.tar
                rm -rf OpenSceneGraph.tar
            fi
            export OSG_FILE_PATH=$BUILD_DIR/build_$PLATFORM/OpenSceneGraph-Data

            if [[ ! -e OpenSceneGraph_build ]]; then
                mkdir OpenSceneGraph_build
            fi
            cd OpenSceneGraph_build

            CMAKE_ARGS="-DBUILD_OSG_APPLICATIONS:BOOL=ON "
            CMAKE_ARGS+="-DBUILD_OSG_EXAMPLES:BOOL=ON "
            CMAKE_ARGS+="-DJPEG_INCLUDE_DIR:PATH= "
            CMAKE_ARGS+="-DJPEG_LIBRARY:FILEPATH= "
            CMAKE_ARGS+="-DTIFF_INCLUDE_DIR:PATH= "
            CMAKE_ARGS+="-DTIFF_LIBRARY:FILEPATH= "
            CMAKE_ARGS+="-DCOLLADA_BOOST_FILESYSTEM_LIBRARY:FILEPATH=C:/Program\ Files/Boost_1.58/lib/boost_filesystem-vc120-mt-1_58.lib "
            CMAKE_ARGS+="-DCOLLADA_BOOST_FILESYSTEM_LIBRARY_DEBUG:FILEPATH=C:/Program\ Files/Boost_1.58/lib/boost_filesystem-vc120-mt-gd-1_58.lib "
            CMAKE_ARGS+="-DCOLLADA_BOOST_SYSTEM_LIBRARY:FILEPATH=C:/Program\ Files/Boost_1.58/lib/boost_system-vc120-mt-1_58.lib "
            CMAKE_ARGS+="-DCOLLADA_BOOST_SYSTEM_LIBRARY_DEBUG:FILEPATH=C:/Program\ Files/Boost_1.58/lib/boost_system-vc120-mt-gd-1_58.lib "
            CMAKE_ARGS+="-DCOLLADA_DYNAMIC_LIBRARY:FILEPATH=C:/Program\ Files/collada-dom/lib/collada-dom2.4-sp-vc100-mt.lib "
            CMAKE_ARGS+="-DCOLLADA_INCLUDE_DIR:PATH=C:/Program\ Files/collada-dom/include/collada-dom2.4 "
            CMAKE_ARGS+="-DCOLLADA_LIBXML_LIBRARY:FILEPATH=$BUILD_DIR/build_$PLATFORM/libxml2-2.7.8.win64/lib/libxml2.lib "
            CMAKE_ARGS+="-DCOLLADA_MINIZIP_LIBRARY:FILEPATH=$BUILD_DIR/build_$PLATFORM/collada-dom/dom/external-libs/minizip-1.1/Release/minizip.lib "
            CMAKE_ARGS+="-DCOLLADA_MINIZIP_LIBRARY_DEBUG:FILEPATH=$BUILD_DIR/build_$PLATFORM/collada-dom/dom/external-libs/minizip-1.1/Release/minizip.lib "
            CMAKE_ARGS+="-DCOLLADA_PCRECPP_LIBRARY:FILEPATH=$BUILD_DIR/build_$PLATFORM/collada-dom/dom/external-libs/pcre-8.02/Release/pcrecpp_local.lib "
            CMAKE_ARGS+="-DCOLLADA_PCRECPP_LIBRARY_DEBUG:FILEPATH=$BUILD_DIR/build_$PLATFORM/collada-dom/dom/external-libs/pcre-8.02/Release/pcrecpp_locald.lib "
            CMAKE_ARGS+="-DCOLLADA_ZLIB_LIBRARY:FILEPATH=$BUILD_DIR/build_$PLATFORM/zlib-1.2.5.win64/lib/zdll.lib "
            echo $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS ../OpenSceneGraph
            eval $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS ../OpenSceneGraph

            echo "Building OpenSceneGraph"
            devenv OpenSceneGraph.sln -project ALL_BUILD -build $CMAKE_BUILD -out OpenSceneGraph.txt
            echo "Installing OpenSceneGraph"
            devenv OpenSceneGraph.sln -project INSTALL -build $CMAKE_BUILD -out OpenSceneGraph_install.txt
            cd ..
        fi

        export OSG_FILE_PATH=$BUILD_DIR/build_$PLATFORM/OpenSceneGraph-Data
        export OSG_DIR=/c/Program\ Files/OpenSceneGraph

#############################
##      Build Bullet       ##
#############################
        if [[ ! -e "C:\Program Files\BULLET_PHYSICS" || $FORCE -eq 1 ]]; then
            if [[ ! -e bullet ]]; then
                /c/Program\ Files/7-Zip/7z x -y $ROOT_DIR/external_libraries/bullet.tgz
                /c/Program\ Files/7-Zip/7z x -y bullet.tar
                rm -rf bullet.tar
            fi

            if [[ ! -e bullet_build ]]; then
                mkdir bullet_build
            fi
            cd bullet_build

            CMAKE_ARGS="-DINSTALL_EXTRA_LIBS:BOOL=ON "
            CMAKE_ARGS+="-DINSTALL_LIBS:BOOL=ON "
            CMAKE_ARGS+="-DUSE_GLUT:BOOL=OFF "
            CMAKE_ARGS+="-DUSE_GRAPHICAL_BENCHMARK:BOOL=OFF "

            CMAKE_ARGS+="-DUSE_MSVC_RUNTIME_LIBRARY_DLL:BOOL=ON " # Otherwise osgbullet fails
            echo $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS ../bullet
            eval $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS ../bullet

            echo "Building BULLET_PHYSICS"
            devenv BULLET_PHYSICS.sln -project ALL_BUILD -build $CMAKE_BUILD -out BULLET_PHYSICS.txt
            echo "Installing BULLET_PHYSICS"
            devenv BULLET_PHYSICS.sln -project INSTALL -build $CMAKE_BUILD -out BULLET_PHYSICS_install.txt
            cd ..
        fi


#############################
##     Build osgWorks      ##
#############################
        if [[ ! -e "C:\Program Files\osgWorks" || $FORCE -eq 1 ]]; then
            if [[ ! -e osgWorks ]]; then
                /c/Program\ Files/7-Zip/7z x -y $ROOT_DIR/external_libraries/osgworks.tgz
                /c/Program\ Files/7-Zip/7z x -y osgworks.tar
                rm -rf osgworks.tar
            fi

            if [[ ! -e osgworks_build ]]; then
                mkdir osgworks_build
            fi
            cd osgworks_build

            CMAKE_ARGS=" "
            echo $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS ../osgWorks
            eval $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS ../osgWorks

            echo "Building osgWorks"
            devenv osgWorks.sln -project ALL_BUILD -build $CMAKE_BUILD -out osgWorks.txt
            echo "Installing osgWorks"
            devenv osgWorks.sln -project INSTALL -build $CMAKE_BUILD -out osgWorks_install.txt
            cd ..
        fi

        export PATH=$PATH:/c/Program\ Files/osgWorks/bin

#############################
##     Build osgBullet     ##
#############################
        if [[ ! -e "C:\Program Files\osgBullet" || $FORCE -eq 1 ]]; then
            if [[ ! -e osgBullet ]]; then
                /c/Program\ Files/7-Zip/7z x -y $ROOT_DIR/external_libraries/osgbullet.tgz
                /c/Program\ Files/7-Zip/7z x -y osgbullet.tar
                rm -rf osgbullet.tar
            fi

            if [[ ! -e osgBullet_build ]]; then
                mkdir osgBullet_build
            fi
            cd osgBullet_build

            CMAKE_ARGS="-DosgWorks_DIR:PATH=\"C:\Program Files\osgWorks\lib\" "

            echo $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS ../osgBullet
            eval $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS ../osgBullet

            echo "Building osgBullet"
            devenv osgBullet.sln -project ALL_BUILD -build $CMAKE_BUILD -out osgBullet.txt
            echo "Installing osgBullet"
            devenv osgBullet.sln -project INSTALL -build $CMAKE_BUILD -out osgBullet_install.txt
            cd ..
        fi

        export PATH=$PATH:/c/Program\ Files/osgBullet/bin

        BOOST_ROOT="/c/Program\ Files/boost_$BOOST_VERSION"

#############################
##       Build Jovian      ##
#############################
        if [[ ! -e Jovian_build ]]; then
            mkdir Jovian_build
        fi
        cd Jovian_build

        CMAKE_ARGS="-DBOOST_ROOT:PATH=$BOOST_ROOT "

        echo $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS $CMAKE_PLATFORM_ARGS $ROOT_DIR
        eval $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_PLATFORM_ARGS $CMAKE_ARGS $ROOT_DIR

        echo "Building Jovian"
        devenv Jovian.sln -project ALL_BUILD -build $CMAKE_BUILD -out Jovian.txt
        cd ..

#############################
##     Build MouseOver     ##
#############################
        if [[ ! -e MouseOver_build ]]; then
            mkdir MouseOver_build
        fi
        cd MouseOver_build

        CMAKE_ARGS="-DBOOST_ROOT:PATH=$BOOST_ROOT "
        CMAKE_ARGS+="-DJOVIAN_BUILD_DIR:PATH=$BUILD_DIR/build_$PLATFORM/Jovian_build "
        CMAKE_ARGS+="-DJOVIAN_SOURCE_DIR:PATH=$ROOT_DIR "

        echo $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS $CMAKE_PLATFORM_ARGS $ROOT_DIR/../MouseOver
        eval $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS $CMAKE_PLATFORM_ARGS $ROOT_DIR/../MouseOver

        echo "Building MouseOver"
        devenv MouseOver.sln -project ALL_BUILD -build $CMAKE_BUILD -out MouseOver.txt
        cd ..

#############################
## Build RemoteDataServer  ##
#############################
        if [[ ! -e RemoteDataServer_build ]]; then
            mkdir RemoteDataServer_build
        fi
        cd RemoteDataServer_build

        CMAKE_ARGS="-DBOOST_ROOT:PATH=$BOOST_ROOT "
        CMAKE_ARGS+="-DJOVIAN_BUILD_DIR:PATH=$BUILD_DIR/build_$PLATFORM/Jovian_build "
        CMAKE_ARGS+="-DJOVIAN_SOURCE_DIR:PATH=$ROOT_DIR "

        echo $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS $CMAKE_PLATFORM_ARGS $ROOT_DIR/../RemoteDataServer
        eval $CMAKE_EXE -DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD $CMAKE_ARGS $CMAKE_PLATFORM_ARGS $ROOT_DIR/../RemoteDataServer

        echo "Building RemoteDataServer"
        devenv RemoteDataServer.sln -project ALL_BUILD -build $CMAKE_BUILD -out RemoteDataServer.txt
        cd ..

        ;;
    clean)
        echo "Cleaning build_$PLATFORM directories"
		if [[ -e build_$PLATFORM ]]; then
			rm -rf build_$PLATFORM
		fi
        ;;
    clobber)
        echo "Cleaning cmake directories"
		if [[ -e cmake-$CMAKE_VERSION ]]; then
			rm -rf cmake-$CMAKE_VERSION
		fi
		if [[ -e build_$PLATFORM  ]]; then
			rm -rf build_$PLATFORM
		fi

        if [[ -e "C:\Program Files\boost_$BOOST_VERSION" ]]; then
            rm -rf "C:\Program Files\boost_$BOOST_VERSION"
        fi

        if [[ -e "C:\Program Files\collada-dom" ]]; then
            rm -rf "C:\Program Files\collada-dom"
        fi

        if [[ -e "C:\Program Files\OpenSceneGraph" ]]; then
            rm -rf "C:\Program Files\OpenSceneGraph"
        fi

        if [[ -e "C:\Program Files\BULLET_PHYSICS" ]]; then
            rm -rf "C:\Program Files\BULLET_PHYSICS"
        fi

        if [[ -e "C:\Program Files\osgWorks" ]]; then
            rm -rf "C:\Program Files\osgWorks"
        fi

        if [[ -e "C:\Program Files\osgBullet" ]]; then
            rm -rf "C:\Program Files\osgBullet"
        fi
        ;;
esac

