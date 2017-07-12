# - Try to find QtSerialPort (an add-on for the Qt5 library, providing a
# single interface for both hardware and virtual serial ports.)
# Once done this will define
#
#  QtSerialPort_FOUND - system has QtSerialPort
#  QtSerialPort_INCLUDE_DIRS - the QtSerialPort include directory
#  QtSerialPort_LIBRARIES - the libraries needed to use QtSerialPort

FIND_LIBRARY (QtSerialPort_LIBRARY QtSerialPort)
FIND_PATH (QtSerialPort_INCLUDE_DIR QtSerialPort/qserialport.h)

set(QtSerialPort_LIBRARIES ${QtSerialPort_LIBRARY} )
set(QtSerialPort_INCLUDE_DIRS ${QtSerialPort_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set QtSerialPort_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(QtSerialPort DEFAULT_MSG
                                  QtSerialPort_LIBRARY QtSerialPort_INCLUDE_DIR)

mark_as_advanced(QtSerialPort_INCLUDE_DIR QtSerialPort_LIBRARY)