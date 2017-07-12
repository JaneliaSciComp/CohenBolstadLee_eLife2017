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

#ifndef CONSOLE_H
#define CONSOLE_H

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <QMainWindow>
#include <QtSerialPort/QSerialPortInfo>

#include <btBulletDynamicsCommon.h>

#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/ArgumentParser>

#include "Communicator.h"
#include "CameraUpdateCallback.h"
#include "Config_Memento.h"
#include "Graph_Evaluator.h"
#include "graphwidget.h"
#include "RadioButtonGroup.h"
#include "QCustomPlot/qcustomplot.h"
#include "scene_model.h"
#include "SerialPortDialog/settingsdialog.h"
#include "ViewingWindowQt.h"

namespace Ui
{
    class MainWindow;
}

/** @brief Console.
 * @details
 */

class Console: public QMainWindow
{
    Q_OBJECT

  public:

    /// @name Initialization
    ///@{
    Console( osg::ArgumentParser& args, QWidget* parent = 0 );
    ///@}

    /// @name Duplication
    ///@{
    ///@}

    /// @name Destruction
    ///@{
    ~Console() {}
    ///@}

    /// @name Access
    ///@{
    Scene_Model_Ptr model() { return _model; }
    int retries( void ) const;
    std::string port( void ) const;
    std::string hostname( void ) const;
    Communicator* get_communicator( void ) { return _comm; }

    ///@}
    /// @name Measurement
    ///@{
    ///@}
    /// @name Comparison
    ///@{
    ///@}
    /// @name Status report
    ///@{
    bool motion_and_display_ganged();
    bool open_field_turning_enabled();
    ///@}
    /// @name Status setting
    ///@{
    ///@}
    /// @name Cursor movement
    ///@{
    ///@}
    /// @name Element change
    ///@{
    void set_viewer( Viewing_Window_Qt* v ) { _viewer = v; };
    ///@}
    /// @name Removal
    ///@{
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
    void blank_display();
    void disable_motion();
    void load_config( Config_Memento const* cm );
    void manual_speed_down();
    void manual_speed_up();
    void reset_position();
    void select_scene();
    ///@}
    /// @name Miscellaneous
    ///@{
    bool move_to_object( std::string name );
    void output_string( std::string output_text );
    bool switch_scene( std::string name );
    ///@}
    /// @name Obsolete
    ///@{
    ///@}
    /// @name Inapplicable
    ///@{
    ///@}

  protected:
    ///@{
    void keyPressEvent( QKeyEvent* event );
    void move_to_start();
    void populate_start_locations();
    void save_config( Config_Memento* cm );
    ///@}

  private Q_SLOTS:

    void about();
    void do_open();
    void do_close();
    void do_open_config();
    void do_save_config();
    void do_image_open();
    void do_OSG_open();
    void reset_sliders( int value );
    void set_camera_maximums( int newMax );
    void start_viewer();
    void update_viewport_left( int value );
    void update_viewport_right( int value );
    void set_x_focal_length( int value );
    void set_y_focal_length( int value );
    void set_gang_motion( bool value );
    void set_distortion_enabled( bool value );
    void set_center();
    void setup_connection( bool reset = true );
    void motion_and_blanking_callback();
    void disable_motion_callback();
    void set_start_location( int value );
    void jump_to_location();

    ///@{ @name Display Callbacks
    void reset_display();
    void distribute_horizontally();
    void distribute_vertically();
    void smooth_display();
    void linearize_edges();
    void set_frame_packing( bool value );
    ///@}

    ///@{ @name Lighting Callbacks
    void set_ambient_color_callback();
    void set_global_brightness_callback();
    void set_display_blanking_callback();
    void set_global_brightness_target( int value );
    void set_background_color_callback();
    void set_diffuse_color_callback();
    void set_diffuse_power( int power );
    ///@}

    ///@{ @name Indicator Callbacks
    void set_indicator_size( int new_size );
    void set_indicator_border_size( int new_size );
    void set_indicator_position( int ignored );
    void set_indicator_screen( int screen );
    void set_indicator_mode( int index );
    ///@}

    ///@{ @name Field of View Callbacks
    void set_field_of_view( int fov );
    void set_field_of_view_offset( int fov_offset );
    void set_field_of_view_offset( double fov_offset );
    void set_rotated_cameras( bool yes_or_no );
    ///@}

    ///@{ @name Calibration Slots
    void calibrate_x();
    void calibrate_y();
    void calibrate_z();
    void enable_editing( bool value );
    void update_calibration_values_callback();
    ///@}

    ///@{ @name Configuration Widgets
    void enable_velocity_smoothing( bool yes_or_no );
    void set_interval_for_velocity_smoothing( int value );
    void enable_pre_blend_heading_smoothing( bool yes_or_no );
    void enable_intermediary_heading_smoothing( bool yes_or_no );
    void enable_post_blend_heading_smoothing( bool yes_or_no );
    void set_interval_for_pre_blending_smoothing( int value );
    void set_interval_for_intermediary_smoothing( int value );
    void set_interval_for_post_blending_smoothing( int value );
    void set_output_serial_port_name( int index );
    void set_data_server_port_name( int index );
    void enable_file_export_callback( bool yes_or_no );
    void do_set_export_file_name();
    void set_output_format( int value );
    void enable_treadmill_output_callback( bool yes_or_no ) { _viewer->set_export_treadmill( yes_or_no ); };
    void enable_reduced_output_callback( bool yes_or_no ) { _viewer->set_reduced_output( yes_or_no ); };
    void auto_heading_callback();
    void set_crossbar_width( double value );
    void restrict_vertical_motion( bool yes_or_no );
    void set_minimum_velocity_thresold( double value );
    void set_output_rate( int value );
    void toggle_physics_debugging( bool yes_or_no ) { _model->debug_physics( yes_or_no ); };
    void show_invisible_objects( bool yes_or_no );
    void change_shader( int value );
    void update_offset( double value );
    ///@}

    ///@{ @name Turning Widgets
    void enable_threshold_turning( bool value );
    void update_plot( int row, int column );
    void update_velocity_plot( int row, int column );
    void add_row();
    void delete_row();
    void add_row_velocity();
    void delete_row_velocity();
    void update_threshold_weight( int val );
    void update_threshold_scale( double val );
    void update_auto_heading_turn_rate( double val );
    ///@}

    ///@{ @name File Widget Slot
    void switch_scene( int id );
    void update_timer();
    void update_ratio_line( );
    ///@}

  private:
    friend class Config_Memento;

    void initialize_plot_widgets( std::vector< double > x, std::vector< double > y,
                                  std::vector< double > vx, std::vector< double > vy,
                                  bool create_plot = true );
    void update_plot( );
    void update_velocity_plot();
    void enable_blanking_widgets( bool onOrOff );
    void update_viewport( int value, int lower, int upper );
    void do_calibration( QPushButton* button, float* rx, float* ry, float* rz );
    void set_ambient_color( QColor color );
    void set_global_brightness( int i, QColor color );
    void set_background_color( QColor color );
    void set_diffuse_color( QColor color );
    void set_export_filename( QString const& fileName );
    float compute_turning_mixture( float val );

    Ui::MainWindow* _ui;
    osg::ArgumentParser& _arguments;
    Scene_Model_Ptr _model;
    Viewing_Window_Qt* _viewer;
    Communicator* _comm;
    Camera_Update_Callback* _callback;
    bool _firstTime;
    bool _connected;
    bool _collada_loaded;
    int _connection_counter;
    bool _use_image;
    int _last_camera;
    bool _calibrating;
    int _activeScene;
    std::string _image_file;
    std::string _osg_file;
    std::vector< std::pair< int, int > > _frame;
    std::vector< triple< bool, int, int > > _distort;
    std::vector< std::pair< int, int > > _center;
    float _Vfwdf;
    float _Vssf;
    float _Omegaf;
    int _calibration_runs[3];
    osg::Vec4 _clear_color;
    std::vector< QColor > _brightness;
    std::vector< Graph_Widget* > _graphics_view;
    bool _blank_display;
    bool _use_auto_heading;
    bool _motion_disabled;
    Radio_Button_Group* _file_widget;
    Radio_Button_Group* _start_location_widgets;
    int _start_index;
    osg::Vec3d _manual_speed;
    QCustomPlot* _plot_ratio_widget;
    QCustomPlot* _plot_velocity_widget;
    double _plot_ratio_max_y, _plot_velocity_max_y;
    QCPItemLine* _ratio_line;
    QCPItemLine* _smoothed_ratio_line;
    QCPItemLine* _velocity_ratio_line;
    QCPItemLine* _smoothed_velocity_ratio_line;
    Graph_Evaluator _plot, _v_plot;
    QTimer* _clock_timer;
    QTimer* _plot_timer;
    QTime _time;
    QList<QSerialPortInfo> _available_ports;
    SettingsDialog* _settings;
    SettingsDialog::Settings _output_port_settings;
    SettingsDialog::Settings _server_port_settings;
};

#endif
