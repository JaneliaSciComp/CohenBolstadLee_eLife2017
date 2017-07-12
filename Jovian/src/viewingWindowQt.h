/* -*-c++-*- */

/*
    System designed by Jeremy D. Cohen, Albert K. Lee, and Mark Bolstad, 2010-2015
    Software designed and implemented by Mark Bolstad, 2010-2015

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef VIEWINGWINDOWQT
#define VIEWINGWINDOWQT

#include <iostream>
#include <vector>

#ifndef Q_MOC_RUN
#include <boost/asio.hpp> // include boost
#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>
#endif

#include <btBulletDynamicsCommon.h>

#include <QtCore/QTimer>
#include <QApplication>
#include <QMainWindow>

#include <osg/Node>
#include <osg/BoundingBox>
#include <osg/StateSet>
#include <osg/Switch>
#include <osg/Texture2D>
#include <osgViewer/Viewer>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#include <osgQt/GraphicsWindowQt>

// For GLSL Shading
#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>

#include <graphwidget.h>
#include <scene_model.h>

typedef boost::shared_ptr< bool > bool_ptr;

/** @brief The Viewer in the Jovian MVC architecture.
 * @details This viewer is an implementation of an osg::Viewer using Qt. It strictly
 * handles the setup of displays and the drawing of the graphics, plus a few incidental
 * items that don't belong in the Viewer (like, per-timestep output to the console)
 */

class Viewing_Window_Qt: public QObject, public osgViewer::Viewer
{
    Q_OBJECT

  public:

    /// @name Initialization
    ///@{
    Viewing_Window_Qt( int nCameras, std::vector< Graph_Widget* > g,
                       osg::ArgumentParser& arguments, Scene_Model_Ptr model );
    ///@}

    /// @name Duplication
    ///@{
    ///@}

    /// @name Move
    ///@{
    ///@}

    /// @name Destruction
    ///@{
    ~Viewing_Window_Qt();
    ///@}

    /// @name Access
    ///@{
    int width() const { if ( _multipleScreens ) return _width; else return _width / numCameras; }
    int height() const { return _height; }
    int number_of_cameras() const { return numCameras; }
    osg::Switch* indicator() const { return _indicator; }

    /**
     * @brief What is the average frame time
     * @details Do the division here in case the call is made before averaging is complete
     * If the average hasn't even started we return 1 to guarantee a valid sample
     * @return double - the average frame time; returns 1 if called before averaging
     * has begun
     */
    //
    double average_frame_time() { return ( ( _avg_frame_count == 0 ) ? 1 : _avg_frame_time / ( double )_avg_frame_count ); }
    ///@}

    /// @name Measurement
    ///@{
    ///@}

    /// @name Comparison
    ///@{
    ///@}

    /// @name Status report
    ///@{
    ///@}

    /// @name Status setting
    ///@{
    /**
     * @brief Sets the focal length along the x-axis
     * @details Sets the focal length along the x-axis. This value is used in the distortion
     * calculation for the built-in cylindrical mapping.
     *
     * @param i - camera index
     * @param focal_length - The new focal length
     */
    void set_x_focal_length( int i, float focal_length ) { _x_focal_length[i] = focal_length; setup_slaves();};

    /**
     * @brief Sets the focal length along the y-axis
     * @details Sets the focal length along the y-axis. This value is used in the distortion
     * calculation for the built-in cylindrical mapping.
     *
     * @param i - camera index
     * @param focal_length - The new focal length
     */
    void set_y_focal_length( int i, float focal_length ) { _y_focal_length[i] = focal_length; setup_slaves();};

    /**
     * @brief Sets the center of the cylindrical distortion
     * @details Sets the center of the cylindrical distortion mapping
     *
     * @param i - camera index
     * @param distorts - Does changing the X center effect the Y progection
     * @param x - x-coordinate of the new center
     * @param y - y-coordinate of the new center
     */
    void set_center( int i, bool distorts, float x, float y ) { _bottom_texcoord[i] = osg::Vec2( x, y ); _x_center_distorts_y = distorts; setup_slaves();};

    /**
     * @brief Sets the OpenGL clear color
     * @details Before the primitives are drawn, OpenGL sets the color of every pixel to
     * the clear or background color. This routine sets the clear color and updates the scene
     *
     * @param color - the new clear/background color
     */
    void set_clear_color( osg::Vec4 color ) { _clear_color = color; setup_slaves(); }

    /**
     * @brief Sets a camera's global brightness
     * @details This sets the global brightness for a camera. Think of it as a filter
     * that is applied after everything is rendered. The default is white, so the
     * primitive colors pass through unchanged
     *
     * @param i - camera index
     * @param color - the color to use for controlling the brightness
     */
    void set_global_brightness( int i, osg::Vec4 color ) { _global_brightness[i] = color; setup_slaves(); }

    /**
     * @brief Sets the node the camera will follow
     * @details Set the node which will be attached to the NodeTrackerManipulator
     */
    void set_track_node();

    /**
     * @brief Sets the field of view of the cameras
     * @details Sets the field of view of the cameras
     *
     * @param fov - the field of view (radians)
     */
    void set_field_of_view( float fov ) { _fov = fov; setup_slaves(); }

    /**
     * @brief The angle of rotation between cameras
     * @details This value is only active if rotate cameras has been enabled. This sets
     * the amount each camera is rotated from the previous one. For example, to get a
     * 180 degree FOV with two cameras, the FOV for each is 90 with an offset of 90
     *
     * @param fov_offset - the angular offset between cameras (radians)
     */
    void set_field_of_view_offset( float fov_offset ) { _fov_offset = fov_offset; setup_slaves(); }

    /**
     * @brief Should the cameras be rotate with respect to each other
     * @details Determines whether to rotate each camera with respect to the previous one.
     * See set_field_of_view_offset for a detailed description.
     *
     * @param yes_or_no - bool
     */
    void set_rotated_cameras( bool yes_or_no ) { _rotate_camera = yes_or_no; setup_slaves(); }

    /**
     * @brief Sets the size of the indicator widget
     * @details The indicator is an on-screen square that when active changes color on
     * each frame (black/white normally, purple/green in high-speed mode). This is
     * designed so that a camera can be attached and monitor the actual frame rate
     *
     * @param new_size - the new size (in pixels)
     */
    void set_indicator_size( int new_size ) { _indicator_size = new_size; setup_slaves(); }

    /**
     * @brief Sets the thickness of the contrasting, always on border
     * @details The indicator is an on-screen square that when active changes color on
     * each frame (black/white normally, purple/green in high-speed mode). This is
     * designed so that a camera can be attached and monitor the actual frame rate
     *
     * @param new_border - border_width (in pixels)
     */
    void set_indicator_border_size( int new_border ) { _indicator_border_size = new_border; setup_slaves(); }

    /**
     * @brief Sets the position of the loweer-left corner.
     * @details The indicator is an on-screen square that when active changes color on
     * each frame (black/white normally, purple/green in high-speed mode). This is
     * designed so that a camera can be attached and monitor the actual frame rate
     *
     * @param new_pos - x,y origin of indicator (in pixels, [0-screen_width] for a single camera)
     */
    void set_indicator_position( osg::Vec2 new_pos ) { _indicator_pos = new_pos; setup_slaves(); }

    /**
     * @brief Sets which screen the indicator should be displayed on
     * @details The indicator is an on-screen square that when active changes color on
     * each frame (black/white normally, purple/green in high-speed mode). This is
     * designed so that a camera can be attached and monitor the actual frame rate
     *
     * @param screen - screen number ([0-number_of_cameras-1])
     */
    void set_indicator_screen( int screen ) { _whichScreen = screen; setup_slaves(); }

    /**
     * @brief Sets the mode for the indicator. Options are Off, Blink On Connection, On
     * @details The indicator is an on-screen square that when active changes color on
     * each frame (black/white normally, purple/green in high-speed mode). This is
     * designed so that a camera can be attached and monitor the actual frame rate
     *
     * @param index - mode (valid range [0-2])
     */
    void set_indicator_mode( int index ) { _indicator_on = index; }

    /**
     * @brief Sets the callback to be called during the update traversal.
     * @details OpenSceneGraph makes several traversals of the scene graph before drawing.
     * One of those traversals is an update pass which is typically used to update
     * the position of an object during animation. This sets the callback to occur
     * at the start of the update traversal.
     *
     * @param callback - The callback to use at the start of the update traversal
     * @param reset - Should we reset the timers back to 0
     */
    void set_update_callback( osg::NodeCallback* callback, bool reset = true );

    /**
     * @brief Sets the name of the serial port for exporting per timestep information
     * @details We use the serial port to communicate with external programs that
     * may need to process per timestep information (position, velocity, heading, collisions).
     * In a true MVC architecture, the belongs in the `C' component, but it's
     * here for convenience
     *
     * @param port name of the serial port, typically COM[1-9] if on windows
     */
    void set_serial_port( std::string port );

    /**
     * @brief Sets the rate at which to send information to the console and serial port
     * @details We implemented this feature to throttle the output rate from
     * once per frame. In high-speed mode we were killing the Arduido's doing the
     * processing of the frame output
     *
     * @param value - output frequency in Hz
     */
    void set_output_rate( int value ) { _output_rate = 1.f / ( float )value; }

    /**
     * @brief Sets the id of the output format to use
     * @details The output format is of the form:
     *      time, x, y, z, speed, heading, format id, [parameters,]
     *      # of collisions, collision list
     * This value controls what is to be printed in the 'parameters' section
     *
     * @param format_id - Positive integer with '0' meaning no parameter values are
     * printed
     */
    void set_output_format( int format_id ) { _output_format = format_id; }

    /**
     * @brief Sets whether we should use the custom OpenGL widget or the OSG version
     * @details Primarily here for historical reasons, the original version of Jovian
     * there was no Qt version of the OSG viewer in the library, they had a demo version
     * which was implemented as the `custom_widget'. Later updates to OSG rolled the
     * example code into the main library, however, it still doesn't work properly for
     * OS X. When the OS X version is fixed, this will be removed
     *
     * @param yes_or_no - bool, use the custom (My_Graphics_Window_Qt) or the builtin
     * OpenGL widget
     */
    void use_custom_widget( bool yes_or_no ) { _custom_widget_enabled = yes_or_no; }

    /**
     * @brief Sets the camera manipulator to tracking
     * @details OSG has several different types of camera manipulators, the primary one
     * used in Jovian is the node tracker. However, through some custom bindings we
     * allow the user to switch manipulators to move about the scene for exploration.
     * This programatically set the manipulator back to the node tracker.
     */
    void set_manipulator_to_tracking_mode() { _key_switch_manipulator->selectMatrixManipulator( 2 ); }

    /**
     * @brief Enables high-speed frame packing mode
     * @details High-speed mode or frame packing is used with specially modified projectors
     * that have had their color wheel removed. We pack three steps of animation into
     * one frame by sticking each frame into a separate color channel. This enables drawing
     * at three times the normal refresh rate.
     *
     * @param value - bool, enable or disable frame packing.
     */
    void set_frame_packing( bool value ) { _packFrames = value; _frame_switch.clear(); setup_slaves(); average_frames(); }

    /**
     * @brief Should we export the raw input from the treadmill to the output streams
     * @details In the raw input from the data server (implemented as the Janelia Treadmill),
     * is the accumulated dx and dy from the treadmill cameras. This option enables passing
     * that data to the output streams.
     *
     * @param yes_or_no - bool
     */
    void set_export_treadmill( bool yes_or_no ) { _output_treadmill_data = yes_or_no; }

    /**
     * @brief Reduce the amount of data sent to the output streams
     * @details Instead of sending the normal boat-load of data to the output streams, setting
     * this flag reduces the output to the x,y location of the camera and any collision data
     *
     * @param yes_or_no - bool
     */
    void set_reduced_output( bool yes_or_no ) { _use_reduced_output = yes_or_no; }
    ///@}

    /**
     * @brief Reduce the amount of data sent to the output streams
     * @details Instead of sending the normal boat-load of data to the output streams, setting
     * this flag reduces the output to the x,y location of the camera and any collision data
     *
     * @param yes_or_no - bool
     */
    void set_texture_resolution( int tex_width, int tex_height ) { _tex_width = tex_width; _tex_height = tex_height; }
    ///@}

    /// @name Cursor movement
    ///@{
    ///@}

    /// @name Element change
    ///@{
    void load_osg( std::string filename );
    void load_model( osg::ArgumentParser& arguments );
    void load_image( std::string filename, bool flip = false, bool use_texture_rectangle = true );
    void load_data( );

    /**
     * @brief Reset the mainpulator to the OSG default
     * @details
     */
    void reset_manipulator_mode() { _key_switch_manipulator->selectMatrixManipulator( 0 ); }
    ///@}

    /// @name Removal
    ///@{
    /**
     * @brief Deletes the connection to the serial port
     * @details Deletes the connection to the serial port. It's easier to delete and create
     * a new port than to try and reset the active one.
     */
    void clear_serial_port() { if ( _port ) delete _port; _port = 0; }
    ///@}

    /// @name Resizing
    ///@{
    ///@}

    /// @name Transformation
    ///@{
    ///@}

    /// @name Conversion
    ///@{
    ///@}

    /// @name Basic operations
    ///@{
    /**
     * @brief Create all of the graphics windows for display
     * @details Create the cameras and window for displaying the scene graph. The first
     * camera goes on `starting_display' with subsequent camera going to display
     * `starting_display' + camera_num % `num_displays'
     *
     * @param nCameras int - The number of cameras. Does not have to equal `num_displays'
     * @param num_displays int - The number of displays to use
     * @param starting_display int - The index of the starting display
     */
    void initialize_cameras( int nCameras, int num_displays, int starting_display );

    /**
     * @brief Update the bounds of viewport i to leftOffset, rightOffset
     * @details Fit viewport i between leftOffset and rightOffset
     *
     * @param i int - viewport index [0, num_cameras]
     * @param leftOffset int - left bound of the viewport [0, rightOffset]
     * @param rightOffset int - right bound of the viewport [leftOffset, window_size]
     */
    void update_viewport( int i, int leftOffset, int rightOffset );

    /**
     * @brief Setup the cameras and graphics windows based on previously set parameters
     * @details The workhorse of graphics initialization. After all the parameters are set
     * from previous routines or externally (through a GUI or config file) this method
     * finishes the graphic window initialization by setting up the render to texture
     * nodes and camera offsets. Called every time some aspect of the display parameters
     * are changed.
     */
    void setup_slaves();

    /**
     * @brief Set up for computing averaging frame time
     * @details Sets up variables for computing the average frame time. If called
     * more than once it's equivilent to reset the variables
     */
    void average_frames() { _average_frames = true; _avg_frame_count = 0; _avg_frame_time = 0.0; _ignored_frames_count = 50; _avg_frame_buffer.clear(); }

    /**
     * @brief Enable distortion computation (inverse cylindrical projection)
     * @details If enabled, this sets the render-to-texture to be pre-distorted with an
     * inverse cylindrical projection based off the parameters set through set_center,
     * set_x_focal_length, and set_y_focal_length
     *
     * @param i int - camera index
     * @param on_or_off bool - enable/disable distortion
     */
    void enable_distortion( int i, bool on_or_off ) { _use_distortion[i] = on_or_off; }

    /**
     * @brief Writes an event to the output stream
     * @details Writes an event to the output stream prepending a time stamp, appending
     * a newline
     *
     * @param event_string,
     * @param float_event_string,
     * @param force_write = false,
     * @param time_stamp = -1
     */
    void output_event( std::ostringstream const& event_string,
                       std::ostringstream const& float_event_string,
                       bool force_write = false,
                       float time_stamp = -1 ) const;
    ///@}

    /// @name Miscellaneous
    ///@{
    ///@}

    /// @name Obsolete
    ///@{
    ///@}

    /// @name Inapplicable
    ///@{
    ///@}

  protected:
    ///@{
    QTimer _timer;

    QWidget* addViewWidget( osg::Camera* camera );

    osg::Node* createDistortionSubgraph( int index, osg::Node* subgraph,
                                         const osg::Vec4& clearColour,
                                         double theta, bool addHud );

    osg::Node* createDistortionSubgraphWithPacking( int index, int idx,
            osg::Node* subgraph,
            const osg::Vec4& clearColour,
            double theta, bool addHud );

    osg::Camera* multipleWindowWithDistortion( int i, int numCameras, int num_displays,
            int starting_display );

    void myRenderingTraversals();
    ///@}

  private Q_SLOTS:

    void paintEvent( );

  private:

    QWidget** widgets;
    Scene_Model_Ptr _model;
    osg::ref_ptr<osg::GraphicsContext::Traits> _traits;
    bool_ptr _swapBuffers;
    int numCameras;
    bool _multipleScreens;
    unsigned int _width, _height;
    float _fov, _fov_offset;
    bool _rotate_camera, _initiated;
    bool _x_center_distorts_y;
    std::vector< float > _x_focal_length;
    std::vector< float > _y_focal_length;
    std::vector< bool > _use_distortion;
    osg::Vec4 _clear_color;
    std::vector< osg::Vec4 > _global_brightness;
    std::vector< osg::Vec2 > _bottom_texcoord;
    osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> _key_switch_manipulator;
    int _childNum;
    osg::Switch* _indicator;
    int _indicator_size;
    int _indicator_border_size;
    int _indicator_on;
    int _whichScreen;
    int _screenNum;
    osg::Vec2 _indicator_pos;
    osg::Camera* _hud;
    btVector3 _lastPosition;
    int _buffer_samples;
    boost::circular_buffer< float > _avg_frame_buffer;
    double _lastRefTime, _startRefTime, _avg_frame_time;
    bool _average_frames;
    int _avg_frame_count, _ignored_frames_count;
    boost::asio::io_service _io;
    boost::asio::serial_port* _port;
    std::vector< Graph_Widget* > _graph;
    double _currSimTime, _prevSimTime;
    bool _packFrames;
    std::vector< osg::Texture2D* > _texture;
    std::vector< osg::Switch* > _frame_switch;
    mutable double _last_output_time, _output_rate;
    int _output_format;
    bool _output_treadmill_data, _use_reduced_output;
    bool _custom_widget_enabled;
    int _tex_width, _tex_height;
};  // end of class Viewing_Window_Qt

#endif

