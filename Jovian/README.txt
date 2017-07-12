Build instructions for Jovian project

This document assumes that we are building under Windows (XP or 7) and using Visual Studio 2013 (VS12).
The link for VS 13 is https://www.visualstudio.com/downloads/download-visual-studio-vs#DownloadFamilies_4. Scroll to the bottom of the page and select older versions. You will most likely need to sign up for the free Visual Studio Dev subscription to enable the download of Vs 2013 (the latest version is Visual Studio 2013 Update 5).

Note: We'll be building Jovian and related tools in 64-bit, assuming a 64-bit install of windows7.

For Windows, we reccomend installing a path editor. We use the version from RedFern
 (http://download.cnet.com/Path-Editor/3000-2094_4-10672356.html)

We recommend putting the source into the root directory of any drive as MinGW seems to have
problems with My Documents and other typical Windows file names.

Reading this, you've unpacked the distribution. Our typical layout looks like:
D:\Jovian\
	build
	Jovian (source)
	MouseOver
	RemoteDataServer

New MingW based install
1. Install minimal MingW
  - Download mingw installer http://sourceforge.net/projects/mingw/files/latest/download?source=files
  - Run the installer (mingw-get-setup.exe)
    - Select the defaults for initial steps
    - When the GUI to install mingw components opens, select "Basic Setup" from the left panel
    - Select "msys-base" from the right panel. Select "Mark for Installation".
    - From the Installation menu, select "Apply Changes"

2. Install Curl
  - Download curl: http://www.confusedbycode.com/curl/#downloads
  - Select 64-bit
  - Run installer and follow prompts

3. Install Qt
  - Download from https://download.qt.io/archive/qt/5.5/; Select latest version (currently 5.5.1)
  - Choose 64-bit Qt: qt-opensource-windows-x86-msvc2013_64-5.5.x.exe
  - Follow installation directions (you can skip the Qt account registration step)
  - If not using a path editor:
    - Control Panel > System > Advanced Setting > Environment Variable
    - Edit the System PATH environment variable
  - Add the Qt bin directory for this version: C:\Qt\Qt5.5.x\5.5\msvc2013_64\bin)

4. (Optional)Install CMake
  - If you choose to skip this step, cmake will automatically be built from source
    but unavailable to the rest of the system
  - Download latest from http://www.cmake.org/cmake/resources/software.html.
  - Choose the version labeled "Windows (Win32 Installer)"
  - Install, select option "Add to PATH for all users"

5. Enable Windows Folder Permissions
  - The build process intstalls files into C:\Program Files from a MingW shell window. In order for this
    to work, the user building the software needs "Full Control" of the Program Files folder.
  - Right-click on C:\Program Files from Windows Explorer
  - Select Properties
  - Select Security tab
  - Either verify or enable "Full Control" in the bottom "Permissions for Authenicated Users" window.

6. Enable Visual Studio from MinGW
  - From the Start Menu, Select All Programs->Visual Studio 2013->Visual Studio Tools
  - Run "VS2013 x64 Native Tools Command Prompt"
  - In the command prompt, type "cd c:\MinGW\msys\1.0" (assuming a default installation
    location of MinGW)
  - type "msys.bat". This will open a second command prompt with green text. From
    here on, all operations will be in this window.
  - Since we're now running under MinGW, everything has a Unix syntax, so
    directories/drives are addressed as /z/Jovian instead of z:\Jovian

7. Build Jovian and MouseOver
  - cd to the Jovian source directory
  - run "./build.cmake -platform windows-x86_64 install <build_location>
    (in our default scenario that directory is /d/Jovian/build)
    There are other arguments to the build script. Run build.cmake to see all
    the options
  - Get a cup of coffee. A new build can take 1-2 hours to complete. See next entry down.
  - Sometimes, a bug in Collada causes the build script to exit with an error, even though it compiled. To fix:
  	- Collada requires the boost libraries, so the script will build boost first
  	- Depending on disk and system speed, this can take up to 45 minutes
  	- Collada will build and exit
  	- Restart the build script In the MingW window, you can repeatedly hit the up arrow key until the "./build.cmake" command appears (should be one key press). Hit return to restart
  	- The build script will skip boost, install Collada, and continue to completion.


8. Add paths
  - MouseOver needs to find a number of dlls, so add the following paths to PATH
    - Boost lib
    - OpenSceneGraph bin
    - OpenSceneGraph plugins bin (in OSG/bin/plugins-x.y.z)
    - osgWorks bin
    - osgBullet bin
    - Collada bin

