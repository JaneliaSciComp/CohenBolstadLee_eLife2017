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

#include <cmath>
#include <cstring>
#include <cstdlib>
#include <fstream>
using std::ofstream;
using std::ifstream;
#include <iostream>
using std::cout;
using std::endl;
#include <algorithm>
using std::min;
using std::max;

#include <boost/regex.hpp>
#include <boost/date_time/local_time/local_time.hpp>

#include <QtCore/QFileInfo>
#include <QColorDialog>
#include <QDesktopWidget>
#include <QErrorMessage>
#include <QFileDialog>
#include <QKeyEvent>
#include <QMainWindow>
#include <QMessageBox>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Material>
#include <osg/Node>
#include <osg/PolygonMode>
#include <osg/PolygonOffset>
#include <osg/ShapeDrawable>
#include <osgUtil/SmoothingVisitor>

#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/RigidBodyAnimation.h>

#include <ui_console.h>

#include <ColladaVisitor.h>
#include <Console.h>
#include <Globals.h>

#define radians( _x_ ) _x_ * M_PI / 180.f
#define degrees( _x_ ) _x_ * 180.f / M_PI

const float scaling_val = 10000.f;

class Smooth_Node_Visitor : public osg::NodeVisitor
{
  public:

    Smooth_Node_Visitor()
        : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
    {
        _smoother = new osgUtil::SmoothingVisitor;
    }

    ~Smooth_Node_Visitor( )
    {
        delete _smoother;
    }

    void apply( osg::Node& node )
    {
        traverse( node );
    }

    void apply( osg::Geode& node )
    {
        _smoother->apply( node );
    }

  private:
    osgUtil::SmoothingVisitor* _smoother;
};

class Keyboard_Event_Handler : public osgGA::GUIEventHandler
{
  public:

    Keyboard_Event_Handler( Console* console ): _console( console ) {}

    virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& )
    {
        switch ( ea.getEventType() )
        {
            case ( osgGA::GUIEventAdapter::KEYDOWN ):
            {
                if ( ea.getKey() == ' ' )
                {
                    _console->reset_position();
                    return true;
                }
                if ( ea.getKey() == 'd' )
                {
                    _console->disable_motion();
                    if ( _console->motion_and_display_ganged() )
                        _console->blank_display();
                    return true;
                }
                if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Up )
                {
                    _console->manual_speed_up();
                    return true;
                }
                if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Down )
                {
                    _console->manual_speed_down();
                    return true;
                }

                break;
            }
            default:
                break;
        }
        return false;
    }

    Console* _console;

};

Console::Console( osg::ArgumentParser& args, QWidget* parent ):
    QMainWindow( parent ), _ui( new Ui::MainWindow() ), _model( new Scene_Model ),
    _arguments( args ), _viewer( 0 ), _callback( 0 ), _firstTime( true ),
    _connected( false ), _collada_loaded( false ), _connection_counter( 1 ), _use_image( false ), _last_camera( 1 ),
    _calibrating( false ), _activeScene( 0 ), _osg_file( "cow.osg" ), _Vfwdf( 1.f ), _Vssf( 1.f ),
    _Omegaf( 1.f ), _blank_display( false ), _use_auto_heading( true ), _motion_disabled( false ),
    _start_index( 0 ), _manual_speed( 0.f, 0.f, 0.f ),
    _plot_ratio_widget( 0 ), _plot_velocity_widget( 0 ),
    _plot_ratio_max_y( 0 ), _plot_velocity_max_y( 0 ), _plot_timer( 0 ),
    _settings( nullptr )
{
    _ui->setupUi( this );

    connect( _ui->actionOpen, SIGNAL( triggered() ), this, SLOT( do_open() ) );
    connect( _ui->actionOpenConfig, SIGNAL( triggered() ), this, SLOT( do_open_config() ) );
    connect( _ui->actionSave, SIGNAL( triggered() ), this, SLOT( do_save_config() ) );
    connect( _ui->actionQuit, SIGNAL( triggered() ), this, SLOT( do_close() ) );
    connect( _ui->actionAbout, SIGNAL( triggered() ), this, SLOT( about() ) );
    connect( _ui->imageCalibrationButton, SIGNAL( clicked() ), this, SLOT( do_image_open() ) );
    connect( _ui->osgDataButton, SIGNAL( clicked() ), this, SLOT( do_OSG_open() ) );

    connect( _ui->cameraSelectSpinBox, SIGNAL( valueChanged( int ) ), this,
             SLOT( reset_sliders( int ) ) );
    connect( _ui->cameraSelectSpinBox, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_global_brightness_target( int ) ) );

    connect( _ui->numCamerasSpinBox, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_camera_maximums( int ) ) );
    connect( _ui->initializeCameraButton, SIGNAL( clicked() ), this, SLOT( start_viewer() ) );
    connect( _ui->horizontalBlankingSlider, SIGNAL( valueChanged( int ) ), this,
             SLOT( update_viewport_left( int ) ) );
    connect( _ui->horizontalBlankingSlider_2, SIGNAL( valueChanged( int ) ), this,
             SLOT( update_viewport_right( int ) ) );
    connect( _ui->networkConnectButton, SIGNAL( clicked() ), this, SLOT( setup_connection() ) );
    connect( _ui->disableMotionButton, SIGNAL( clicked() ), this, SLOT( disable_motion_callback() ) );
    connect( _ui->displayBlankingButton, SIGNAL( clicked() ), this,
             SLOT( set_display_blanking_callback() ) );
    connect( _ui->gangMotionCheckBox, SIGNAL( toggled( bool ) ), this,
             SLOT( set_gang_motion( bool ) ) );
    connect( _ui->jumpToStartButton, SIGNAL( clicked() ), this, SLOT( jump_to_location() ) );

    // Display Callbacks
    connect( _ui->resetButton, SIGNAL( clicked() ), this, SLOT( reset_display() ) );
    connect( _ui->distributeHoriz, SIGNAL( clicked() ), this, SLOT( distribute_horizontally() ) );
    connect( _ui->distributeVert, SIGNAL( clicked() ), this, SLOT( distribute_vertically() ) );
    connect( _ui->smoothButton, SIGNAL( clicked() ), this, SLOT( smooth_display() ) );
    connect( _ui->linearizeEdgesButton, SIGNAL( clicked() ), this, SLOT( linearize_edges() ) );
    connect( _ui->framePackingEnabled, SIGNAL( toggled( bool ) ),
             this, SLOT( set_frame_packing( bool ) ) );

    // Lighting Callbacks
    connect( _ui->ambientLightButton, SIGNAL( clicked() ), this,
             SLOT( set_ambient_color_callback() ) );
    connect( _ui->backgroundColorButton, SIGNAL( clicked() ), this,
             SLOT( set_background_color_callback() ) );
    connect( _ui->diffuseLightButton, SIGNAL( clicked() ), this,
             SLOT( set_diffuse_color_callback() ) );
    connect( _ui->diffusePowerLevel, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_diffuse_power( int ) ) );
    connect( _ui->globalBrightnessButton, SIGNAL( clicked() ), this,
             SLOT( set_global_brightness_callback() ) );
    connect( _ui->shaderComboBox, SIGNAL( currentIndexChanged( int ) ), this,
             SLOT( change_shader( int ) ) );

    connect( _ui->fovSlider, SIGNAL( valueChanged( int ) ), this, SLOT( set_field_of_view( int ) ) );
    connect( _ui->fovOffsetSlider, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_field_of_view_offset( int ) ) );
    connect( _ui->fovOffsetSpinBox, SIGNAL( valueChanged( double ) ), this,
             SLOT( set_field_of_view_offset( double ) ) );
    connect( _ui->rotatedCamerasCheckBox, SIGNAL( toggled( bool ) ), this,
             SLOT( set_rotated_cameras( bool ) ) );

    // Indicator Widgets
    connect( _ui->indicatorSizeSlider, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_indicator_size( int ) ) );
    connect( _ui->borderSizeSlider, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_indicator_border_size( int ) ) );
    connect( _ui->indicatorHorizontalPositionSlider, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_indicator_position( int ) ) );
    connect( _ui->indicatorVerticalPositionSlider, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_indicator_position( int ) ) );
    connect( _ui->screenSelectSpinBox, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_indicator_screen( int ) ) );
    connect( _ui->frameModeComboBox, SIGNAL( currentIndexChanged( int ) ), this,
             SLOT( set_indicator_mode( int ) ) );

    connect( _ui->focalLengthSlider_x, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_x_focal_length( int ) ) );
    connect( _ui->focalLengthSlider_y, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_y_focal_length( int ) ) );
    connect( _ui->xCenterSlider, SIGNAL( valueChanged( int ) ), this, SLOT( set_center( ) ) );
    connect( _ui->yCenterSlider, SIGNAL( valueChanged( int ) ), this, SLOT( set_center( ) ) );
    connect( _ui->enableDistortion, SIGNAL( toggled( bool ) ), this,
             SLOT( set_distortion_enabled( bool ) ) );

    // Calibration Widgets
    connect( _ui->calibrateXButton, SIGNAL( clicked() ), this, SLOT( calibrate_x( ) ) );
    connect( _ui->calibrateYButton, SIGNAL( clicked() ), this, SLOT( calibrate_y( ) ) );
    connect( _ui->calibrateZButton, SIGNAL( clicked() ), this, SLOT( calibrate_z( ) ) );
    connect( _ui->editValues, SIGNAL( toggled( bool ) ), this, SLOT( enable_editing( bool ) ) );
    connect( _ui->updateCalibrationValuesButton, SIGNAL( clicked() ), this, SLOT( update_calibration_values_callback( ) ) );

    // Configuration Widgets
    connect( _ui->velocitySmoothingEnabled, SIGNAL( toggled( bool ) ), this,
             SLOT( enable_velocity_smoothing( bool ) ) );
    connect( _ui->velocitySmoothingInterval, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_interval_for_velocity_smoothing( int ) ) );
    connect( _ui->intermediarySmoothingEnabled, SIGNAL( toggled( bool ) ), this,
             SLOT( enable_intermediary_heading_smoothing( bool ) ) );
    connect( _ui->outputSmoothingEnabled, SIGNAL( toggled( bool ) ), this,
             SLOT( enable_post_blend_heading_smoothing( bool ) ) );
    connect( _ui->inputSmoothingInterval, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_interval_for_pre_blending_smoothing( int ) ) );
    connect( _ui->intermediarySmoothingInterval, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_interval_for_intermediary_smoothing( int ) ) );
    connect( _ui->outputSmoothingInterval, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_interval_for_post_blending_smoothing( int ) ) );
    connect( _ui->outputPortName, SIGNAL( currentIndexChanged( int ) ), this,
             SLOT( set_output_serial_port_name( int ) ) );
    connect( _ui->dataServerPortName, SIGNAL( currentIndexChanged( int ) ), this,
             SLOT( set_data_server_port_name( int ) ) );
    connect( _ui->fileExportToggle, SIGNAL( toggled( bool ) ), this,
             SLOT( enable_file_export_callback( bool ) ) );
    connect( _ui->exportFileNameSelector, SIGNAL( clicked() ), this, SLOT( do_set_export_file_name() ) );

    connect( _ui->outputFormatSelector, SIGNAL( currentIndexChanged( int ) ), this,
             SLOT( set_output_format( int ) ) );
    connect( _ui->treadmillOutputToggle, SIGNAL( toggled( bool ) ), this,
             SLOT( enable_treadmill_output_callback( bool ) ) );
    connect( _ui->reducedOutputToggle, SIGNAL( toggled( bool ) ), this,
             SLOT( enable_reduced_output_callback( bool ) ) );
    connect( _ui->headingStatusButton, SIGNAL( clicked() ), this,
             SLOT( auto_heading_callback() ) );
    connect( _ui->crossbarWidthSpinBox, SIGNAL( valueChanged( double ) ), this,
             SLOT( set_crossbar_width( double ) ) );
    connect( _ui->minimumVelocityThreshold, SIGNAL( valueChanged( double ) ), this,
             SLOT( set_minimum_velocity_thresold( double ) ) );
    connect( _ui->restrictVerticalMotion, SIGNAL( toggled( bool ) ), this,
             SLOT( restrict_vertical_motion( bool ) ) );
    connect( _ui->outputRateSpinBox, SIGNAL( valueChanged( int ) ), this,
             SLOT( set_output_rate( int ) ) );

    // Debugging Widgets
    connect( _ui->debugPhysicsCheckBox, SIGNAL( toggled( bool ) ), this,
             SLOT( toggle_physics_debugging( bool ) ) );
    connect( _ui->showInvisibleObjectsCheckBox, SIGNAL( toggled( bool ) ), this,
             SLOT( show_invisible_objects( bool ) ) );
    connect( _ui->xoffset, SIGNAL( valueChanged( double ) ), this,
             SLOT( update_offset( double ) ) );
    connect( _ui->yoffset, SIGNAL( valueChanged( double ) ), this,
             SLOT( update_offset( double ) ) );
    connect( _ui->zoffset, SIGNAL( valueChanged( double ) ), this,
             SLOT( update_offset( double ) ) );

    // Turning Widgets
    connect( _ui->thresholdTurningEnabled, SIGNAL( toggled( bool ) ), this,
             SLOT( enable_threshold_turning( bool ) ) );
    connect( _ui->plotTableWidget, SIGNAL( cellChanged ( int, int ) ), this,
             SLOT( update_plot( int, int ) ) );
    connect( _ui->plotVelocityTableWidget, SIGNAL( cellChanged ( int, int ) ), this,
             SLOT( update_velocity_plot( int, int ) ) );
    connect( _ui->addRowButton, SIGNAL( clicked() ), this,
             SLOT( add_row() ) );
    connect( _ui->deleteRowButton, SIGNAL( clicked() ), this,
             SLOT( delete_row() ) );
    connect( _ui->addRowButton_2, SIGNAL( clicked() ), this,
             SLOT( add_row_velocity() ) );
    connect( _ui->deleteRowButton_2, SIGNAL( clicked() ), this,
             SLOT( delete_row_velocity() ) );
    connect( _ui->thresholdTurningSlider, SIGNAL( valueChanged( int ) ), this,
             SLOT( update_threshold_weight( int ) ) );
    connect( _ui->minThresholdScale, SIGNAL( valueChanged( double ) ), this,
             SLOT( update_threshold_scale( double ) ) );
    connect( _ui->maxThresholdScale, SIGNAL( valueChanged( double ) ), this,
             SLOT( update_threshold_scale( double ) ) );
    connect( _ui->autoHeadingTurnRate, SIGNAL( valueChanged( double ) ), this,
             SLOT( update_auto_heading_turn_rate( double ) ) );

    // Disable calibration widgets until a port is selected
    _ui->calibrateXButton->setEnabled( false );
    _ui->calibrateYButton->setEnabled( false );
    _ui->calibrateZButton->setEnabled( false );

    // Setup a null communicator
    _comm = new VoidCommunicator( hostname(), port(), retries() );
    enable_blanking_widgets( false );
    _ui->actionOpen->setEnabled( false );
    _ui->headingStatusButton->setEnabled( false );

    for ( int i = 0; i < 3; i++ )
        _calibration_runs[i] = 0;

    // Set the color labels
    _clear_color = osg::Vec4( 0.f, 0.f, 0.f, 1.0 );

    QColor color( 25, 25, 25 );
    _ui->ambientLabel->setText( "#191919" );
    _ui->ambientLabel->setPalette( QPalette( color ) );
    _ui->ambientLabel->setAutoFillBackground( true );

    _ui->diffuseLabel->setText( "#191919" );
    _ui->diffuseLabel->setPalette( QPalette( color ) );
    _ui->diffuseLabel->setAutoFillBackground( true );

    color.setRgb( 0, 0, 0 );
    _ui->backgroundLabel->setText( "#000000" );
    _ui->backgroundLabel->setPalette( QPalette( color ) );
    _ui->backgroundLabel->setAutoFillBackground( true );

    color.setRgb( 255, 255, 255 );
    _ui->brightnessLabel->setText( "#ffffff" );
    _ui->brightnessLabel->setPalette( QPalette( color ) );
    _ui->brightnessLabel->setAutoFillBackground( true );

    for ( int i = 0; i < _ui->numCamerasSpinBox->value(); i++ )
        _brightness.push_back( color );

    osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();

    // Ensure there is one display widget before we add new ones. The camera
    // selection widget is 1-based so we need the extra padding
    if ( _ui->displayWidget->count() == 0 )
    {
        QWidget* widget = new QWidget();
        _ui->displayWidget->addWidget( widget );
    }
    else
        // Remove all current layout items in `_ui->displayWidget'
        while ( _ui->displayWidget->count() > 1 )
        {
            QWidget* widget = _ui->displayWidget->currentWidget();
            _ui->displayWidget->removeWidget( widget );
        }

    int num_screens = wsi->getNumScreens();
    char s[256];
    sprintf( s, "Number of Displays(%d)", num_screens );
    _ui->displayCountLabel->setText( s );
    _ui->startingDisplaySpinBox->setValue( num_screens == 1 ? 1 : 2 );
    num_screens = num_screens > 1 ? num_screens - 1 : num_screens;
    _ui->numCamerasSpinBox->setValue( num_screens );
    _ui->numDisplaysSpinBox->setValue( num_screens );

    // In case setting `_ui->numCamerasSpinBox' above didn't trigger the callback
    set_camera_maximums( num_screens );

    QDesktopWidget desktopWidget;
    QRect r( desktopWidget.screenGeometry( 0 ) );
    _ui->indicatorHorizontalPositionSlider->setMaximum( r.right() );
    _ui->indicatorHorizontalPositionSpinBox->setMaximum( r.right() );
    _ui->indicatorVerticalPositionSlider->setMaximum( r.bottom() );
    _ui->indicatorVerticalPositionSpinBox->setMaximum( r.bottom() );

    _ui->screenSelectSpinBox->setValue( _ui->numCamerasSpinBox->value() );

    _ui->displayWidget->setCurrentIndex( 1 );

    // The file status widget
    _file_widget = new Radio_Button_Group( _ui->fileGroupBox, _ui->fileGroupBoxLayout, _ui->fileGroupSpacer );
    QObject::connect( _file_widget, SIGNAL( activated( int ) ),
                      this, SLOT( switch_scene( int ) ) );

    _start_location_widgets = new Radio_Button_Group( _ui->startingLocationsBox,
                                                      _ui->startingLocationsBoxLayout, _ui->startingLocationsSpacer );
    QObject::connect( _start_location_widgets, SIGNAL( activated( int ) ),
                      this, SLOT( set_start_location( int ) ) );


    this->setFocusPolicy( Qt::StrongFocus );

    //    _ui->startingLocationsBox->hide();
    _ui->headingStatusButton->hide();

    std::vector< double > x, y, vx, vy;
    x.push_back( 0. ); y.push_back( 60. );
    x.push_back( 1. ); y.push_back( 60. );
    x.push_back( 3. ); y.push_back( 30. );
    x.push_back( 5. ); y.push_back( 6. );
    x.push_back( 10. ); y.push_back( 3. );
    x.push_back( 100. ); y.push_back( 0. );
    x.push_back( 101. ); y.push_back( 0. );

    vx.push_back( 0. );  vy.push_back( 0. );
    vx.push_back( 0.1 ); vy.push_back( 0.01 );
    vx.push_back( 1. );  vy.push_back( 0.1 );
    vx.push_back( 10. ); vy.push_back( 0.5 );
    vx.push_back( 50. ); vy.push_back( 1. );
    vx.push_back( 100. ); vy.push_back( 1. );

    initialize_plot_widgets( x, y, vx, vy, true );

    // Magic values to align home position with tracker
    _ui->debugPanel->hide();
    _ui->xoffset->setValue( 0.0 );
    _ui->yoffset->setValue( 0.0 );
    _ui->zoffset->setValue( 0.0 );

    // Qt 4.8.5 will crash on OS X if we don't use a custom OpenGL widget
#ifdef __APPLE__
    _ui->useCustomGLWidget->setChecked( true );
#endif

    // Query serial ports
    _available_ports = QSerialPortInfo::availablePorts();
    QList<QSerialPortInfo>::iterator port;
    for ( port = _available_ports.begin();
            port != _available_ports.end(); port++ )
    {
        _ui->dataServerPortName->addItem( port->portName() );
        _ui->outputPortName->addItem( port->portName() );
    }

    _ui->dataServerPortName->setCurrentIndex( 0 );
    _ui->outputPortName->setCurrentIndex( 0 );
}

bool
Console::motion_and_display_ganged() { return _ui->gangMotionCheckBox->isChecked(); }

bool
Console::open_field_turning_enabled()
{
    return !_model->has_path() && _ui->thresholdTurningEnabled->isChecked();
}

void
Console::setup_connection( bool reset )
{
    Communicator* new_comm;

    if ( !_connected )
    {
        if ( _ui->TCP_radioBtn->isChecked() )
        {
            new_comm = new TCPCommunicator( hostname(), port(), retries() );
        }
        else if ( _ui->UDP_radioBtn->isChecked() )
        {
            new_comm = new UDPCommunicator( hostname(), port(), retries() );
        }

        if ( new_comm->success )
        {
            delete _comm;
            _comm = new_comm;
            //_comm->reset();

            // Clear out old data
            _comm->read();
            _comm->write( _ui->dataServerPortName->currentText().toStdString() );

            _connected = true;

            if ( _ui->exportFileNameStyle->currentIndex() == 1 )
            {
                char s[40];
                sprintf( s, "Disconnect (%d)", _connection_counter );
                _ui->networkConnectButton->setText( s );
            }
            else
                _ui->networkConnectButton->setText( "Disconnect" );

            if ( reset )
                if ( _ui->fileExportToggle->isChecked() )
                    enable_file_export_callback( true );

            _Vfwdf = _ui->calibrateXValue->value();
            _Omegaf = _ui->calibrateYValue->value();
            _Vssf = _ui->calibrateZValue->value();

            _Vfwdf /= _ui->calibrateXGain->value();
            _Omegaf /= _ui->calibrateYGain->value();
            _Vssf /= _ui->calibrateZGain->value();

            // Potential problem here:
            // It is unlikely, but it is remotely possible that a call to connect
            // to the server is made before the viewer has finished averaging the
            // frames. This will occur if we switch to scripting, but for now the
            // scenario is unlikely.
            if ( _model->tracker() == 0 )
            {
                if ( open_field_turning_enabled() )
                    _callback = new Threshold_Camera_Update_Callback( _viewer->getCameraManipulator(),
                                                                      _comm, _Vfwdf, _Omegaf, _Vssf,
                                                                      _ui->ballRadiusBox->value(),
                                                                      1, _model->contact_results() );
                else
                    _callback = new Remote_Camera_Update_Callback( _viewer->getCameraManipulator(),
                                                                   _comm, _Vfwdf, _Omegaf, _Vssf,
                                                                   _ui->ballRadiusBox->value(),
                                                                   1, _model->contact_results(),
                                                                   _model->has_path() );
            }
            else
            {
                if ( open_field_turning_enabled() )
                    _callback = new Threshold_Camera_Update_Callback( _model->tracker(),
                                                                      _comm, _Vfwdf, _Omegaf, _Vssf,
                                                                      _ui->ballRadiusBox->value(),
                                                                      1, _model->contact_results() );
                else
                    _callback = new Remote_Camera_Update_Callback( _model->tracker(),
                                                                   _comm, _Vfwdf, _Omegaf, _Vssf,
                                                                   _ui->ballRadiusBox->value(),
                                                                   1, _model->contact_results(),
                                                                   _model->has_path() );
            }

            if ( _model->motion_disabled() )
                _callback->toggleMotion();

            _callback->addNestedCallback( _model->camera_callback() );

            _viewer->set_update_callback( _callback, reset );

            _model->move_offset( _ui->xoffset->value(), _ui->yoffset->value(), _ui->zoffset->value() );

            if ( _model->is_tracking() )
            {
                _model->camera_callback()->interval_for_velocity_smoothing( _ui->velocitySmoothingInterval->value() );
                _model->camera_callback()->enable_input_heading_smoothing( _ui->inputSmoothingEnabled->isChecked() );
                _model->camera_callback()->enable_intermediary_heading_smoothing( _ui->intermediarySmoothingEnabled->isChecked() );
                _model->camera_callback()->enable_output_heading_smoothing( _ui->outputSmoothingEnabled->isChecked() );
                _model->camera_callback()->enable_velocity_smoothing( _ui->velocitySmoothingEnabled->isChecked() );
                _model->camera_callback()->interval_for_heading_smoothing( _ui->inputSmoothingInterval->value(),
                                                                           _ui->intermediarySmoothingInterval->value(),
                                                                           _ui->outputSmoothingInterval->value() );
                _model->camera_callback()->set_plot_data( _plot, _v_plot );
                _model->camera_callback()->reset_accumulators();
                _model->set_minimum_velocity_thresold( _ui->minimumVelocityThreshold->value() );
            }

            _model->engage_physics_engine( true );

            if ( _ui->enableTimedTrialCheckBox->isChecked() )
            {
                _time = _ui->timeTrialEditor->time();
                int ms = _time.hour() * 3600000 + _time.minute() * 60000 + _time.second() * 1000;
                QTimer::singleShot( ms, this, SLOT( setup_connection() ) );
                _clock_timer = new QTimer( this );
                connect( _clock_timer, SIGNAL( timeout() ), this, SLOT( update_timer() ) );
                _clock_timer->start( 1000 );
                _ui->enableTimedTrialCheckBox->setEnabled( false );
                _ui->timeTrialEditor->setEnabled( false );
                // Connection button needs to be disabled during a time trial
                _ui->networkConnectButton->setEnabled( false );
            }
        }
        else
        {
            _viewer->set_update_callback( 0, reset );

            _callback = 0;
            _model->engage_physics_engine( false );
        }
    }
    else
    {
        if ( _ui->enableTimedTrialCheckBox->isChecked() )
        {
            _clock_timer->stop();
            _ui->timeTrialEditor->setTime( _time );
            delete _clock_timer;
            _ui->enableTimedTrialCheckBox->setEnabled( true );
            _ui->timeTrialEditor->setEnabled( true );
            _ui->networkConnectButton->setEnabled( true );

            std::ostringstream command;
            command << "Trial_Ended";
            _viewer->output_event( command, command, true );
            _viewer->reset_manipulator_mode(); // Not sure if this is necessary
            move_to_start();
        }

        _callback->removeNestedCallback( _model->camera_callback() );
        _model->clear_tracker_data();

        _connected = false;
        _callback = 0;
        _viewer->set_update_callback( 0, reset );
        _ui->networkConnectButton->setText( "Connect" );
        _model->engage_physics_engine( false );
        if ( reset )
            if ( _ui->exportFileNameStyle->currentIndex() == 1 )
                _connection_counter++;

    }
}

void
Console::motion_and_blanking_callback()
{
    disable_motion();
    set_display_blanking_callback();
}

void
Console::disable_motion_callback()
{
    disable_motion();
}

void
Console::set_start_location( int value )
{
    _start_index = value;
}

void
Console::jump_to_location()
{
    _model->set_start_location( _start_index );
    reset_position();
}

std::string
Console::port( void ) const
{
    return std::string( _ui->portCtrl->text().toStdString() );
}

int
Console::retries( void ) const
{
    int numRetries = 3;

    numRetries = _ui->retriesCtrl->text().toInt( );

    return numRetries;
}

std::string
Console::hostname( void ) const
{
    return std::string( _ui->hostnameCtrl->text().toStdString() );
}

void
Console::reset_position()
{
    _model->move_offset( _ui->xoffset->value(), _ui->yoffset->value(), _ui->zoffset->value() );
    _model->reset_motion();
    if ( !_model->eye_point_set() && _viewer ) _viewer->home();
}

void
Console::manual_speed_up()
{
    if ( _motion_disabled && _callback )
    {
        osg::Vec3d vec;
        // 1.414 is the smallest unit of forward motion based on cameras
        // with a 45 degree offset
        vec.set( ( ( 1.414 / ( float )_Vfwdf )  * M_PI ) * _ui->ballRadiusBox->value(), 0.f, 0.f );
        _manual_speed += vec;
        _callback->set_manual_speed_vector( _manual_speed );
    }
}

void
Console::manual_speed_down()
{
    if ( _motion_disabled && _callback )
    {
        osg::Vec3d vec;
        vec.set( ( ( -1.414 / ( float )_Vfwdf )  * M_PI ) * _ui->ballRadiusBox->value(), 0.f, 0.f );
        _manual_speed += vec;
        _callback->set_manual_speed_vector( _manual_speed );
    }
}

void
Console::disable_motion()
{
    std::ostringstream command;

    _model->toggle_motion_disabled();

    if ( _callback )_callback->toggleMotion();

    if ( _model->motion_disabled() )
    {
        _ui->disableMotionButton->setText( "Enable Motion" );
        command << "Enable_Motion_Off";
    }
    else
    {
        _ui->disableMotionButton->setText( "Disable Motion" );
        command << "Enable_Motion_On";
    }

    _viewer->output_event( command, command, true );
}

void
Console::blank_display()
{
    std::ostringstream command;

    if ( _blank_display )
    {
        _blank_display = false;
        _ui->displayBlankingButton->setText( "Enable Display Blanking" );
        command << "Display_Blanking_Off";
        for ( int i = 0; i < _ui->numCamerasSpinBox->value(); i++ )
            set_global_brightness_target( i + 1 );
    }
    else
    {
        _blank_display = true;
        command << "Display_Blanking_On";
        _ui->displayBlankingButton->setText( "Disable Display Blanking" );
        osg::Vec4 col = osg::Vec4( 0.f, 0.f, 0.f, 1.0 );

        for ( int i = 0; i < _ui->numCamerasSpinBox->value(); i++ )
            _viewer->set_global_brightness( i, col );
    }

    _viewer->output_event( command, command, true );
}

void
Console::keyPressEvent( QKeyEvent* event )
{
    switch ( event->key() )
    {
        case Qt::Key_Space:
            if ( _model->is_tracking() )
                reset_position();
            else
                _viewer->home();
            break;
        case Qt::Key_D:
        {
            disable_motion();
            if ( motion_and_display_ganged() )
                blank_display();
        }
        break;
        case Qt::Key_S:
        {
            Smooth_Node_Visitor smoother;
            smoother.apply( *( _model->scene_root() ) );
        }

        break;

        // These two cases are for when the user wants to manually drive through
        // the world without using the ball as input
        case Qt::Key_Up:
            manual_speed_up();
            break;
        case Qt::Key_Down:
            manual_speed_down();
            break;
        default:
            return;
    }

}

void
Console::move_to_start()
{
    _model->switch_scene( _model->active_scene() );

    _viewer->set_track_node();

    _ui->headingStatusButton->setEnabled( true );

    _viewer->getCameraManipulator()->setHomePosition( _model->start_center(),
                                                      _model->start_direction(),
                                                      osg::Vec3f( 0, 0, 1 ) );

    _viewer->set_manipulator_to_tracking_mode();
    reset_position();

}

void
Console::populate_start_locations()
{
    // List all the start starting locations
    _start_location_widgets->clear();
    std::vector< Node_Offset_Pair > locations = _model->start_locations();
    std::vector< Node_Offset_Pair >::iterator it;
    for ( it = locations.begin(); it != locations.end(); it++ )
    {
        boost::regex e( "_start_(.*)_$" );
        boost::smatch what;

        // Try to pick off the trailing string of the start name, e.g., _start_xxxx_
        // If we can't find any trailing characters, just use start.
        if ( boost::regex_match( it->second->getName(), what, e ) )
            _start_location_widgets->load( QString::fromUtf8( what[1].str().c_str() ) );
        else
            _start_location_widgets->load( QString::fromUtf8( "start" ) );
    }
    _start_location_widgets->select( _model->start_location() );
}

void
Console::do_close()
{
    if ( _ui->thresholdTurningEnabled->isChecked() )
        enable_threshold_turning( false ); //Force it to be off so that certain widgets
    //aren't updated after we delete the model.

    if ( _viewer )
        delete _viewer;

    _model.reset();

    close();
}

void
Console::about()
{
    QMessageBox::about( this, "About Jovian", "Jovian version: $Rev::              $\n$Date::                    $" );
}

void
Console::do_open()
{
    bool reset_connection = false;

    _viewer->reset_manipulator_mode();

    if ( _model->is_tracking() )
    {
        // Turn off connection to remoteDataServer if active
        if ( _connected )
        {
            reset_connection = true;
            setup_connection();
        }
    }

    Config_Memento* cm = _model->current_configuration();
    if ( !cm->initialized() )
        cm->initialize( this );

    save_config( cm );

    QString fileName = QFileDialog::getOpenFileName( this, tr( "Open File" ),
                                                     "", tr( "Collada (*.dae);; All Files (*.*)" ) );
    if ( !fileName.isEmpty() )
    {
        // Stick the file name (minus path and extension into the widget )
        QFileInfo info( fileName );
        _file_widget->load( info.completeBaseName() );
        _model->open_field_turning( _ui->thresholdTurningEnabled->isChecked() );
        _model->load_collada( fileName );
        _ui->networkConnectButton->setEnabled( true );
        _collada_loaded = true;

        if ( _ui->exportFileNameBox->text().count() == 0 )
        {
            QString s( fileName );
            s.replace( ".dae", "" );
            set_export_filename( s );
        }

        // List all the start starting locations
        _start_location_widgets->clear();
        set_start_location( 0 );
        populate_start_locations();

        // Set the bounds of the offset widgets to valid values based on the ball radius
        _ui->xoffset->setMinimum( -_model->camera_mount_radius() );
        _ui->xoffset->setMaximum( _model->camera_mount_radius() );

        _ui->yoffset->setMinimum( -_model->camera_mount_radius() );
        _ui->yoffset->setMaximum( _model->camera_mount_radius() );

        _ui->zoffset->setMinimum( -_model->camera_mount_radius() );
        // zoffset maximum is arbitrary

        _ui->headingStatusButton->setEnabled( true );

        _viewer->getCameraManipulator()->setHomePosition( _model->start_center(),
                                                          _model->start_direction(),
                                                          osg::Vec3f( 0, 0, 1 ) );

        _viewer->set_track_node();
        _viewer->set_manipulator_to_tracking_mode();
        reset_position();

        // Force a reset of the diffuse lighting values on a load as lights are
        // created anew each read.
        QColor color( _ui->ambientLabel->text() );
        set_ambient_color( color );
        color = QColor( _ui->diffuseLabel->text() );
        set_diffuse_color( color );
        set_diffuse_power( _ui->diffusePowerLevel->value() );

        // Reenable connection to remoteDataServer if it was active
        if ( reset_connection )
            setup_connection();

        _viewer->load_data();

        cm = _model->current_configuration();
        if ( !cm->initialized() )
            cm->initialize( this );

        load_config( cm );

        // Set this after we reload the config file so that it's not overwritten
        _ui->crossbarWidthSpinBox->setValue( _model->crossbar_width() );
    }
}

void
Console::do_open_config()
{
    QString fileName = QFileDialog::getOpenFileName( this, tr( "Open Config File" ),
                                                     "", tr( "MouseOver Config (*.mvc);; All Files (*.*)" ) );
    if ( !fileName.isEmpty() )
    {
        QByteArray ba = fileName.toLatin1();
        ifstream config( ba.data() );
        Config_Memento* cm = _model->current_configuration();
        cm->initialize( this );
        cm->load( config );

        load_config( cm );

        config.close();
    }
}

void
Console::do_save_config()
{
    QString fileName = QFileDialog::getSaveFileName( this, tr( "Save Config File" ),
                                                     "", tr( "Collada (*.mvc);; All Files (*.*)" ) );
    if ( !fileName.isEmpty() )
    {
        QByteArray ba = fileName.toLatin1();
        ofstream config( ba.data() );
        Config_Memento* cm = _model->current_configuration();
        if ( !cm->initialized() )
            cm->initialize( this );

        save_config( cm );

        cm->save( config );

        config.close();
    }
}

void
Console::set_x_focal_length( int value )
{
    if ( _viewer )
    {
        if ( _ui->gangedCheckBox->isChecked() )
        {
            int old_last = _last_camera;

            for ( int i = 0; i < _ui->numCamerasSpinBox->value(); i++ )
            {
                _last_camera = i + 1;
                reset_sliders( _last_camera );
                _viewer->set_x_focal_length( i, value / 100.f );
            }
            _last_camera = old_last;
        }
        else
            _viewer->set_x_focal_length( _ui->cameraSelectSpinBox->value() - 1, value / 100.f );
    }
}


void
Console::set_y_focal_length( int value )
{
    if ( _viewer )
    {
        if ( _ui->gangedCheckBox->isChecked() )
        {
            int old_last = _last_camera;

            for ( int i = 0; i < _ui->numCamerasSpinBox->value(); i++ )
            {
                _last_camera = i + 1;
                reset_sliders( _last_camera );
                _viewer->set_y_focal_length( i, value / 100.f );
            }
            _last_camera = old_last;
        }
        else
            _viewer->set_y_focal_length( _ui->cameraSelectSpinBox->value() - 1, value / 100.f );
    }
}


void
Console::set_distortion_enabled( bool value )
{
    if ( _viewer )
    {
        if ( _ui->gangedCheckBox->isChecked() )
        {
            int old_last = _last_camera;

            for ( int i = 0; i < _ui->numCamerasSpinBox->value(); i++ )
            {
                _last_camera = i + 1;
                reset_sliders( _last_camera );
                _viewer->enable_distortion( i, value );
            }
            _last_camera = old_last;
        }
        else
            _viewer->enable_distortion( _ui->cameraSelectSpinBox->value() - 1, value );

        _viewer->setup_slaves();
    }
}

void
Console::set_gang_motion( bool value )
{
    if ( _viewer )
    {
        _ui->disableMotionButton->disconnect();
        _ui->displayBlankingButton->disconnect();

        if ( value )
        {
            connect( _ui->disableMotionButton, SIGNAL( clicked() ), this, SLOT( motion_and_blanking_callback() ) );
            connect( _ui->displayBlankingButton, SIGNAL( clicked() ), this, SLOT( motion_and_blanking_callback() ) );
            // If we have a callback it mean we are hooked to the ball
            // so, motion status sets the value
            if ( _callback )
            {
                // If motion is enabled and the screen is blank or motion is disabled
                // and the screen is active, flip the display blanking
                if ( ( !_callback->motion_state() && _blank_display ) || ( _callback->motion_state() && !_blank_display ) )
                    set_display_blanking_callback();
            }
        }
        else
        {
            connect( _ui->disableMotionButton, SIGNAL( clicked() ), this, SLOT( disable_motion_callback() ) );
            connect( _ui->displayBlankingButton, SIGNAL( clicked() ), this,
                     SLOT( set_display_blanking_callback() ) );
        }
    }
}

void
Console::set_center( )
{
    if ( _viewer )
    {
        if ( _ui->gangedCheckBox->isChecked() )
        {
            int old_last = _last_camera;

            for ( int i = 0; i < _ui->numCamerasSpinBox->value(); i++ )
            {
                _last_camera = i + 1;
                reset_sliders( _last_camera );
                _viewer->set_center( i, _ui->xCenterDistortsBox->isChecked(),
                                     -_ui->xCenterSlider->value() / 100.f, -_ui->yCenterSlider->value() / 100.f );
            }
            _last_camera = old_last;
        }
        else
            _viewer->set_center( _ui->cameraSelectSpinBox->value() - 1, _ui->xCenterDistortsBox->isChecked(),
                                 -_ui->xCenterSlider->value() / 100.f, -_ui->yCenterSlider->value() / 100.f );

        _viewer->setup_slaves();
    }
}

void
Console::do_image_open()
{
    QString fileName = QFileDialog::getOpenFileName( this, tr( "Open Image File" ),
                                                     "", tr( "Image Files (*.jpg *.png *.tif *.bmp);; Movie Files (*.avi *.mov *mpg *mp4);;All Files (*.*)" ) );
    if ( !fileName.isEmpty() )
    {
        QFileInfo info( fileName );

        _ui->imageCalibrationButton->setText( info.fileName() );
        _image_file = fileName.toStdString();
        _use_image = true;
    }
}

void
Console::do_OSG_open()
{
    QString directory( "" );
    char* osg_path = getenv( "OSG_FILE_PATH" );
    if ( osg_path != NULL )
        directory = osg_path;

    QString fileName = QFileDialog::getOpenFileName( this, tr( "Open OSG File" ),
                                                     directory, tr( "OSG Files (*.osg);; All Files (*.*)" ) );
    if ( !fileName.isEmpty() )
    {
        QFileInfo info( fileName );

        _ui->osgDataButton->setText( info.fileName() );
        _osg_file = fileName.toStdString();
    }
}

void
Console::enable_file_export_callback( bool yes_or_no )
{
    _model->set_export_status( yes_or_no );
    if ( yes_or_no )
    {
        if ( _ui->exportFileNameBox->text().count() == 0 )
        {
            if ( _viewer )
            {
                QString s( _model->scene_file_name() );
                s.replace( ".dae", "" );
                set_export_filename( s );
            }
            else
                set_export_filename( QString( "foo" ) );
        }
        else
            set_export_filename( _ui->exportFileNameBox->text() );
    }
}

void
Console::set_output_format( int format_id )
{
    if ( _viewer )
        _viewer->set_output_format( format_id );
}

void
Console::do_set_export_file_name()
{
    QString directory( "" );

    QString fileName = QFileDialog::getSaveFileName( this, tr( "Open Export File" ),
                                                     directory, tr( "All Files (*.txt)" ) );
    if ( !fileName.isEmpty() )
    {
        QFileInfo info( fileName );

        QString file = info.filePath();
        file.replace( "." + info.completeSuffix(), "" );
        _ui->exportFileNameBox->setText( file );
    }
}

void
Console::reset_display()
{
    _graphics_view[ _ui->cameraSelectSpinBox->value() - 1 ]->reset();
}

void
Console::distribute_horizontally()
{
    _graphics_view[ _ui->cameraSelectSpinBox->value() - 1 ]->distribute_horizontally();
}

void
Console::distribute_vertically()
{
    _graphics_view[ _ui->cameraSelectSpinBox->value() - 1 ]->distribute_vertically();
}

void
Console::smooth_display()
{
    _graphics_view[ _ui->cameraSelectSpinBox->value() - 1 ]->smooth();
}

void
Console::linearize_edges()
{
    _graphics_view[ _ui->cameraSelectSpinBox->value() - 1 ]->linearize_edges();
}

void
Console::set_frame_packing( bool value )
{
    _viewer->set_frame_packing( value );
}

void
Console::set_ambient_color_callback()
{
    QColor color = QColorDialog::getColor( QColor( _ui->ambientLabel->text() ), this );
    if ( color.isValid() )
        set_ambient_color( color );
}

void
Console::set_ambient_color( QColor color )
{
    _ui->ambientLabel->setText( color.name() );
    _ui->ambientLabel->setPalette( QPalette( color ) );
    _ui->ambientLabel->setAutoFillBackground( true );
    osg::Vec4 amb = osg::Vec4( color.red() / 255.0, color.green() / 255.0, color.blue() / 255.0, 1.0 );

    _model->set_ambient_color( amb );
}

void
Console::set_background_color_callback()
{
    QColor color = QColorDialog::getColor( QColor( _ui->backgroundLabel->text() ), this );
    if ( color.isValid() )
        set_background_color( color );
}

void
Console::set_background_color( QColor color )
{
    _ui->backgroundLabel->setText( color.name() );
    _ui->backgroundLabel->setPalette( QPalette( color ) );
    _ui->backgroundLabel->setAutoFillBackground( true );
    _clear_color = osg::Vec4( color.red() / 255.0, color.green() / 255.0, color.blue() / 255.0, 1.0 );

    if ( _viewer )
        _viewer->set_clear_color( _clear_color );
}

void
Console::set_diffuse_color_callback()
{
    QColor color = QColorDialog::getColor( QColor( _ui->diffuseLabel->text() ), this );
    if ( color.isValid() )
        set_diffuse_color( color );
}

void
Console::set_diffuse_color( QColor color )
{
    _ui->diffuseLabel->setText( color.name() );
    _ui->diffuseLabel->setPalette( QPalette( color ) );
    _ui->diffuseLabel->setAutoFillBackground( true );
    osg::Vec4 diff = osg::Vec4( color.red() / 255.0, color.green() / 255.0, color.blue() / 255.0, 1.0 );

    _model->set_diffuse_color( diff );
}

void
Console::set_diffuse_power( int power )
{
    if ( _ui->diffusePowerLevel->value() != power )
        _ui->diffusePowerLevel->setValue( power );

    _model->set_diffuse_power( _ui->diffusePowerLevel->value() );
}

void
Console::set_global_brightness_callback()
{
    QColor color = QColorDialog::getColor( QColor( _ui->brightnessLabel->text() ), this );
    if ( color.isValid() )
    {
        _brightness[ _ui->cameraSelectSpinBox->value() - 1 ] = color;
        set_global_brightness( _ui->cameraSelectSpinBox->value() - 1, color );
    }
}

void
Console::set_display_blanking_callback()
{
    blank_display();
}

void
Console::set_global_brightness_target( int value )
{
    set_global_brightness( value - 1, _brightness[ value - 1 ] );
}

void
Console::set_global_brightness( int i, QColor color )
{
    _ui->brightnessLabel->setText( color.name() );
    _ui->brightnessLabel->setPalette( QPalette( color ) );
    _ui->brightnessLabel->setAutoFillBackground( true );
    osg::Vec4 col = osg::Vec4( color.red() / 255.0, color.green() / 255.0, color.blue() / 255.0, 1.0 );

    if ( _viewer )
        _viewer->set_global_brightness( i, col );
}

// The sense of the FOV and the FOV offset are backwards from the widget names
// and controls. In this case, the FOV slider sets the rotated camera offset
// which in turn sets the actual FOV. Confusing!

void
Console::set_field_of_view( int fov )
{
    if ( _viewer )
    {
        if ( _ui->rotatedCamerasCheckBox->isChecked() )
            _viewer->set_field_of_view_offset( ( float )fov / _viewer->number_of_cameras() );
        else
            _viewer->set_field_of_view( ( float )fov );
    }
}

void
Console::set_field_of_view_offset( int fov_offset )
{
    _ui->fovOffsetSpinBox->setValue( fov_offset / 10.f );
}

void
Console::set_field_of_view_offset( double fov_offset )
{
    _ui->fovOffsetSlider->setValue( ( int )( fov_offset * 10.f ) );
    if ( _viewer )
        _viewer->set_field_of_view( ( float )fov_offset );
}

void
Console::set_rotated_cameras( bool yes_or_no )
{
    if ( _viewer )
        _viewer->set_rotated_cameras( yes_or_no );
    set_field_of_view( ( float )_ui->fovSlider->value() );
    if ( yes_or_no )
        set_field_of_view_offset( _ui->fovOffsetSpinBox->value() );
}

void
Console::set_indicator_size( int new_size )
{
    if ( _viewer )
        _viewer->set_indicator_size( new_size );
}

void
Console::set_indicator_border_size( int new_size )
{
    if ( _viewer )
        _viewer->set_indicator_border_size( new_size );
}

void
Console::set_indicator_position( int ignored )
{
    osg::Vec2 v( _ui->indicatorHorizontalPositionSlider->value(),
                 _ui->indicatorVerticalPositionSlider->value() );
    if ( _viewer )
        _viewer->set_indicator_position( v );
}

void
Console::set_indicator_screen( int screen )
{
    if ( _viewer )
        _viewer->set_indicator_screen( screen - 1 );
}

void
Console::set_indicator_mode( int index )
{
    if ( _viewer )
        _viewer->set_indicator_mode( index );
}

void
Console::set_camera_maximums( int newMax )
{
    _ui->cameraSelectSpinBox->setMaximum( newMax );
    _ui->screenSelectSpinBox->setMaximum( newMax );
    if ( newMax > _brightness.size() )
        for ( int i = _brightness.size(); i < newMax; i++ )
            _brightness.push_back( _brightness.back() );

    enable_blanking_widgets( false );

    if ( _viewer )
        if ( _viewer->number_of_cameras() == newMax )
            enable_blanking_widgets( true );

    if ( newMax > _graphics_view.size() )
    {
        for ( int i = _graphics_view.size(); i < newMax; i++ )
        {
            QWidget* widget = new QWidget();
            _ui->displayWidget->addWidget( widget );
        }

        for ( int i = _graphics_view.size(); i < newMax; i++ )
        {
            // Offset the _ui->displayWidget by 1 since the camera index is 1-based
            QVBoxLayout* layout = new QVBoxLayout( _ui->displayWidget->widget( i + 1 ) );
            Graph_Widget* gw = new Graph_Widget( _ui->displayWidget->widget( i + 1 ) );
            _graphics_view.push_back( gw );
            layout->addWidget( gw );
        }
    }

    for ( int i = _frame.size(); i < newMax; i++ )
    {
        _frame.push_back( std::pair< int, int >( 0, 0 ) );
        _distort.push_back( triple< bool, int, int >( true, 100, 100 ) );
        _center.push_back( std::pair< int, int >( 50, 50 ) );
    }

}

void
Console::reset_sliders( int value )
{
    _frame[ _last_camera - 1 ].first = _ui->horizontalBlankingSlider->value();
    _frame[ _last_camera - 1 ].second = _ui->horizontalBlankingSlider_2->value();

    _distort[ _last_camera - 1 ].first = _ui->enableDistortion->isChecked();
    _distort[ _last_camera - 1 ].second = _ui->focalLengthSlider_y->value();
    _distort[ _last_camera - 1 ].third = _ui->focalLengthSlider_x->value();

    _center[ _last_camera - 1 ].first = _ui->xCenterSlider->value();
    _center[ _last_camera - 1 ].second = _ui->yCenterSlider->value();

    _ui->horizontalBlankingSlider->setValue( _frame[ value - 1 ].first );
    _ui->horizontalBlankingSlider_2->setValue( _frame[ value - 1 ].second );

    _ui->enableDistortion->setChecked( _distort[ value - 1 ].first );
    _ui->focalLengthSlider_y->setValue( _distort[ value - 1 ].second );
    _ui->focalLengthSlider_x->setValue( _distort[ value - 1 ].third );

    _ui->xCenterSlider->setValue( _center[ value - 1 ].first );
    _ui->yCenterSlider->setValue( _center[ value - 1 ].second );

    update_viewport_left( _ui->horizontalBlankingSlider->value() );
    update_viewport_right( _ui->horizontalBlankingSlider_2->value() );

    _last_camera = value;
}

void
Console::start_viewer()
{
    if ( _viewer )
        delete _viewer;

    _viewer = new Viewing_Window_Qt( _ui->numCamerasSpinBox->value(), _graphics_view, _arguments, _model );
    if ( _ui->textureResComboBox->currentIndex() > 0 )
    {
        std::string::size_type sz;
        int resolution = std::stoi( _ui->textureResComboBox->currentText().toStdString(), &sz );
        _viewer->set_texture_resolution( resolution, resolution );
    }
    _viewer->use_custom_widget( _ui->useCustomGLWidget->isChecked() );

    _viewer->initialize_cameras( _ui->numCamerasSpinBox->value(),
                                 _ui->numDisplaysSpinBox->value(),
                                 _ui->startingDisplaySpinBox->value() - 1 );

    // Since the loading of the config file and/or setting widget values can
    // occur before the viewer is initialized, we need to pass all of the
    // widget settable parameters now.
    _viewer->set_output_format( _ui->outputFormatSelector->currentIndex() );
    _viewer->set_clear_color( _clear_color );
    _viewer->set_field_of_view_offset( ( float )_ui->fovSlider->value() / _ui->numCamerasSpinBox->value() );
    _viewer->set_field_of_view( ( float )_ui->fovOffsetSpinBox->value() );

    for ( int i = 0; i < _ui->numCamerasSpinBox->value(); i++ )
        _frame[i].second = _viewer->width();

    _ui->horizontalBlankingSlider->setMaximum( _viewer->width() );
    _ui->horizontalBlankingSpinBox->setMaximum( _viewer->width() );
    _ui->horizontalBlankingSlider_2->setMaximum( _viewer->width() );
    _ui->horizontalBlankingSpinBox_2->setMaximum( _viewer->width() );
    _ui->horizontalBlankingSlider_2->setValue( _viewer->width() );

    enable_blanking_widgets( true );

    int screen = _ui->cameraSelectSpinBox->value();

    for ( int i = 1; i <= _ui->numCamerasSpinBox->value(); i++ )
        set_global_brightness_target( i );

    set_global_brightness_target( screen );

    _viewer->set_x_focal_length( _ui->cameraSelectSpinBox->value() - 1, _ui->focalLengthSlider_x->value() / 100.f );
    _viewer->set_y_focal_length( _ui->cameraSelectSpinBox->value() - 1, _ui->focalLengthSlider_y->value() / 100.f );
    _viewer->set_center( _ui->cameraSelectSpinBox->value() - 1, _ui->xCenterDistortsBox->isChecked(),
                         -_ui->xCenterSlider->value() / 100.f, -_ui->yCenterSlider->value() / 100.f );

    _ui->indicatorHorizontalPositionSlider->setValue( ( _ui->numCamerasSpinBox->value() * _viewer->width() ) -
                                                      _ui->numCamerasSpinBox->value() * ( _ui->indicatorSizeSlider->value() + _ui->borderSizeSlider->value() ) );
    _ui->indicatorVerticalPositionSlider->setValue( _ui->borderSizeSlider->value() );

    if ( _ui->imageRadioButton->isChecked() && _use_image )
    {
        _viewer->load_image( _image_file );
        _viewer->home();
        MovieEventHandler* meh = new MovieEventHandler();
        meh->set( _viewer->getSceneData() );
        _viewer->addEventHandler( meh );

    }
    else
    {
        _viewer->addEventHandler( new Keyboard_Event_Handler( this ) );
        if ( _ui->osgRadioButton->isChecked() )
        {
            _viewer->load_osg( _osg_file );
            _viewer->home();
        }
    }

    _ui->actionOpen->setEnabled( true );
    _ui->displayBlankingButton->setEnabled( true );
    _ui->gangMotionCheckBox->setEnabled( true );
    _ui->disableMotionButton->setEnabled( true );

    _viewer->set_output_rate( _ui->outputRateSpinBox->value() );
}

void
Console::update_viewport_left( int value )
{
    if ( _ui->horizontalBlankingSlider->value() <= _ui->horizontalBlankingSlider_2->value() )
        update_viewport( _ui->cameraSelectSpinBox->value(),
                         _ui->horizontalBlankingSlider->value(),
                         _ui->horizontalBlankingSlider_2->value() );
    else
        _ui->horizontalBlankingSlider_2->setValue( _ui->horizontalBlankingSlider->value() );
}

void
Console::update_viewport_right( int value )
{
    if ( _ui->horizontalBlankingSlider->value() <= _ui->horizontalBlankingSlider_2->value() )
        update_viewport( _ui->cameraSelectSpinBox->value(),
                         _ui->horizontalBlankingSlider->value(),
                         _ui->horizontalBlankingSlider_2->value() );
    else
        _ui->horizontalBlankingSlider->setValue( _ui->horizontalBlankingSlider_2->value() );
}

void
Console::enable_blanking_widgets( bool onOrOff )
{
    _ui->cameraSelectSpinBox->setEnabled( onOrOff );
    _ui->horizontalBlankingSlider->setEnabled( onOrOff );
    _ui->horizontalBlankingSpinBox->setEnabled( onOrOff );
    _ui->horizontalBlankingSlider_2->setEnabled( onOrOff );
    _ui->horizontalBlankingSpinBox_2->setEnabled( onOrOff );
    _ui->horizontalBlankingSlider_2->setEnabled( onOrOff );
}

void
Console::calibrate_x()
{
    float rx, ry, rz;

    if ( !_calibrating && !_ui->computeAverage->isChecked() )
    {
        _calibration_runs[0] = 0;
        _Vfwdf = 0.f;
    }

    do_calibration( _ui->calibrateXButton, &rx, &ry, &rz );
    if ( !_calibrating )
    {
        _calibration_runs[0] += 1;
        float count = ( float )_calibration_runs[0];

        _Vfwdf = _Vfwdf * ( count - 1.f ) / count + rx / count;
        if ( std::abs( _Vfwdf ) < 1.f )
            _Vfwdf = 1.f;
        _ui->calibrateXValue->setValue( _Vfwdf );
        _ui->calibrateYButton->setEnabled( true );
        _ui->calibrateZButton->setEnabled( true );
    }
    else
    {
        _ui->calibrateYButton->setEnabled( false );
        _ui->calibrateZButton->setEnabled( false );
    }
}

void
Console::calibrate_y()
{
    float rx, ry, rz;

    if ( !_calibrating && !_ui->computeAverage->isChecked() )
    {
        _calibration_runs[1] = 0;
        _Omegaf = 0.f;
    }

    do_calibration( _ui->calibrateYButton, &rx, &ry, &rz );
    if ( !_calibrating )
    {
        _calibration_runs[1] += 1;
        float count = ( float )_calibration_runs[1];

        _Omegaf = _Omegaf * ( count - 1.f ) / count + rz / count;
        if ( std::abs( _Omegaf ) < 1.f )
            _Omegaf = 1.f;
        _ui->calibrateYValue->setValue( _Omegaf );
        _ui->calibrateXButton->setEnabled( true );
        _ui->calibrateZButton->setEnabled( true );
    }
    else
    {
        _ui->calibrateXButton->setEnabled( false );
        _ui->calibrateZButton->setEnabled( false );
    }
}


void
Console::calibrate_z()
{
    float rx, ry, rz;

    if ( !_calibrating && !_ui->computeAverage->isChecked() )
    {
        _calibration_runs[2] = 0;
        _Vssf = 0.f;
    }

    do_calibration( _ui->calibrateZButton, &rx, &ry, &rz );
    if ( !_calibrating )
    {
        _calibration_runs[2] += 1;
        float count = ( float )_calibration_runs[2];

        _Vssf = _Vssf * ( count - 1.f ) / count + ry / count;
        if ( std::abs( _Vssf ) < 1.f )
            _Vssf = 1.f;
        _ui->calibrateZValue->setValue( _Vssf );
        _ui->calibrateYButton->setEnabled( true );
        _ui->calibrateXButton->setEnabled( true );
    }
    else
    {
        _ui->calibrateYButton->setEnabled( false );
        _ui->calibrateXButton->setEnabled( false );
    }
}

void
Console::do_calibration( QPushButton* button, float* rx, float* ry, float* rz )
{
    Communicator* new_comm;

    if ( _connected )
    {
        _connected = false;
    }

    if ( !_calibrating )
    {
        if ( _ui->TCP_radioBtn->isChecked() )
        {
            new_comm = new TCPCommunicator( hostname(), port(), retries() );
        }
        else if ( _ui->UDP_radioBtn->isChecked() )
        {
            new_comm = new UDPCommunicator( hostname(), port(), retries() );
        }

        if ( new_comm->success )
        {
            delete _comm;
            _comm = new_comm;
            // _comm->reset();

            _connected = true;
        }

        _comm->read();
        _comm->write( _ui->dataServerPortName->currentText().toStdString() );
        _calibrating = true;
        button->setText( "Stop Calibration ..." );
    }
    else
    {
        _calibrating = false;
        button->setText( "Start Calibration ..." );
        _comm->read();

        if ( _comm->message_length() > 0 )
        {
            try
            {
                float Vfwd, Vss, Omega;
                sscanf( _comm->data(), "%f,%f,%f", &Vfwd, &Vss, &Omega );
                *rx = Vfwd;
                *ry = Vss;
                *rz = Omega;

                _comm->close();
            }
            catch ( std::exception& e )
            {
                std::cout << "Caught exception\n";
                // Reset the connection
                _comm->close();
                _comm->reset();
            }
        }
    }
}

void
Console::enable_editing( bool onOrOff )
{
    // Invert, for when we want to edit, we need to turn read only off
    _ui->calibrateXValue->setReadOnly( !onOrOff );
    _ui->calibrateYValue->setReadOnly( !onOrOff );
    _ui->calibrateZValue->setReadOnly( !onOrOff );
}

void
Console::update_calibration_values_callback()
{
    _Vfwdf = _ui->calibrateXValue->value();
    _Omegaf = _ui->calibrateYValue->value();
    _Vssf = _ui->calibrateZValue->value();

    _Vfwdf /= _ui->calibrateXGain->value();
    _Omegaf /= _ui->calibrateYGain->value();
    _Vssf /= _ui->calibrateZGain->value();

    if ( _callback != 0 )
        _callback->update_scale_factors( _Vfwdf, _Omegaf, _Vssf );
}

void
Console::enable_velocity_smoothing( bool yes_or_no )
{
    if ( _model->is_tracking() )
    {
        _model->camera_callback()->enable_velocity_smoothing( yes_or_no );
        _model->camera_callback()->interval_for_velocity_smoothing( _ui->velocitySmoothingInterval->value() );
    }
}

void
Console::set_interval_for_velocity_smoothing( int value )
{
    if ( _model->is_tracking() )
        _model->camera_callback()->interval_for_velocity_smoothing( value );
}

void
Console::enable_pre_blend_heading_smoothing( bool yes_or_no )
{
    if ( _model->camera_callback() )
    {
        _model->camera_callback()->enable_input_heading_smoothing( yes_or_no );
        if ( _model->is_tracking() )
            _model->camera_callback()->interval_for_heading_smoothing( _ui->inputSmoothingInterval->value(),
                                                                       _ui->intermediarySmoothingInterval->value(),
                                                                       _ui->outputSmoothingInterval->value() );
    }
}

void
Console::enable_intermediary_heading_smoothing( bool yes_or_no )
{
    if ( _model->camera_callback() )
    {
        _model->camera_callback()->enable_intermediary_heading_smoothing( yes_or_no );
        if ( _model->is_tracking() )
            _model->camera_callback()->interval_for_heading_smoothing( _ui->inputSmoothingInterval->value(),
                                                                       _ui->intermediarySmoothingInterval->value(),
                                                                       _ui->outputSmoothingInterval->value() );
    }
}

void
Console::enable_post_blend_heading_smoothing( bool yes_or_no )
{
    if ( _model->camera_callback() )
    {
        _model->camera_callback()->enable_output_heading_smoothing( yes_or_no );
        if ( _model->is_tracking() )
            _model->camera_callback()->interval_for_heading_smoothing( _ui->inputSmoothingInterval->value(),
                                                                       _ui->intermediarySmoothingInterval->value(),
                                                                       _ui->outputSmoothingInterval->value() );
    }
}

void
Console::set_interval_for_pre_blending_smoothing ( int value )
{
    if ( _model->is_tracking() )
        _model->camera_callback()->interval_for_heading_smoothing( value,
                                                                   _ui->intermediarySmoothingInterval->value(),
                                                                   _ui->outputSmoothingInterval->value() );
}

void
Console::set_interval_for_intermediary_smoothing ( int value )
{
    if ( _model->is_tracking() )
        _model->camera_callback()->interval_for_heading_smoothing( _ui->inputSmoothingInterval->value(),
                                                                   value,
                                                                   _ui->outputSmoothingInterval->value() );
}

void
Console::set_interval_for_post_blending_smoothing( int value )
{
    if ( _model->is_tracking() )
        _model->camera_callback()->interval_for_heading_smoothing( _ui->inputSmoothingInterval->value(),
                                                                   _ui->intermediarySmoothingInterval->value(),
                                                                   value );
}

void
Console::set_output_serial_port_name( int index )
{
    if ( _viewer )
    {
        _viewer->clear_serial_port();
        if ( _ui->outputPortName->currentText() != "None" )
            _viewer->set_serial_port( _ui->outputPortName->currentText().toStdString() );
    }
}

void
Console::set_data_server_port_name( int index )
{
    if ( index > 0 ) // Not 'None'
    {
        if ( _collada_loaded )
            _ui->networkConnectButton->setEnabled( true );

        _ui->calibrateXButton->setEnabled( true );
        _ui->calibrateYButton->setEnabled( true );
        _ui->calibrateZButton->setEnabled( true );
    }
    else
    {
        _ui->networkConnectButton->setEnabled( false );
        _ui->calibrateXButton->setEnabled( false );
        _ui->calibrateYButton->setEnabled( false );
        _ui->calibrateZButton->setEnabled( false );
    }
}

void
Console::switch_scene( int id )
{
    QColor color;
    bool resetConnection = _connected;

    _viewer->reset_manipulator_mode();

    if ( _model->is_tracking() )
    {
        // Turn off connection to remoteDataServer if active
        if ( _connected )
            setup_connection();
    }

    Config_Memento* cm = _model->current_configuration();
    if ( !cm->initialized() )
        cm->initialize( this );

    save_config( cm );

    _model->switch_scene( id );

    cm = _model->current_configuration();
    if ( !cm->initialized() )
        cm->initialize( this );

    _viewer->set_track_node();

    _ui->headingStatusButton->setEnabled( true );

    _viewer->getCameraManipulator()->setHomePosition( _model->start_center(),
                                                      _model->start_direction(),
                                                      osg::Vec3f( 0, 0, 1 ) );

    _viewer->set_manipulator_to_tracking_mode();

    _start_location_widgets->clear();
    populate_start_locations();

    load_config( cm );

    jump_to_location();

    /*
        color = QColor( _ui->ambientLabel->text() );
        set_ambient_color( color );

        color = QColor( _ui->diffuseLabel->text() );
        set_diffuse_color( color );
        set_diffuse_power( _ui->diffusePowerLevel->value() );

        color = QColor( _ui->backgroundLabel->text() );
        set_background_color( color );
    */
    // Reenable connection to remoteDataServer if it was active
    if ( resetConnection )
        setup_connection();
}

void
Console::select_scene()
{
    bool resetConnection = _connected;

    _viewer->reset_manipulator_mode();

    if ( _model->is_tracking() )
    {
        // Turn off connection to remoteDataServer if active
        if ( _connected )
            setup_connection( false );

    }

    move_to_start();

    // Reenable connection to remoteDataServer if it was active
    if ( resetConnection )
    {
        setup_connection( false );
        if ( _ui->thresholdTurningEnabled->isChecked() )
        {
            float scale_fac = compute_turning_mixture( _ui->thresholdTurningSlider->value() );
            _model->camera_callback()->set_turning_mixture( scale_fac );
        }
        else
        {
            _model->camera_callback()->set_turning_mixture( 0.f );
        }

        _model->camera_callback()->enable_velocity_smoothing( _ui->velocitySmoothingEnabled->isChecked() );
        _model->camera_callback()->enable_input_heading_smoothing( _ui->inputSmoothingEnabled->isChecked() );
        _model->camera_callback()->enable_output_heading_smoothing( _ui->outputSmoothingEnabled->isChecked() );
        if ( _model->is_tracking() )
        {
            _model->camera_callback()->interval_for_heading_smoothing( _ui->inputSmoothingInterval->value(),
                                                                       _ui->intermediarySmoothingInterval->value(),
                                                                       _ui->outputSmoothingInterval->value() );
            _model->camera_callback()->interval_for_velocity_smoothing( _ui->velocitySmoothingInterval->value() );
        }
    }
}

void
Console::auto_heading_callback()
{
    if ( _use_auto_heading )
        _ui->headingStatusButton->setText( "Manual Heading Enabled" );
    else
        _ui->headingStatusButton->setText( "Auto Heading Enabled" );

    _use_auto_heading = !_use_auto_heading;
    _model->use_rotations( _use_auto_heading );
}

/******************************************************************************
PURPOSE: enable_threshold_turning - Callback for enabling threshold based turning
INPUTS:  bool value - passed from Qt callback on _ui->thresholdTurningEnabled
OUTPUTS:  None
RETURNS: None
******************************************************************************/
void
Console::enable_threshold_turning( bool value )
{
    if ( _model->is_tracking() )
    {
        bool is_connected = _connected;
        _model->open_field_turning( value );
        if ( is_connected )
            setup_connection();

        if ( value  )
        {
            float scale_fac = compute_turning_mixture( _ui->thresholdTurningSlider->value() );
            _model->camera_callback()->set_turning_mixture( scale_fac );
            _ratio_line = new QCPItemLine( _plot_ratio_widget );
            _plot_ratio_widget->addItem( _ratio_line );
            _smoothed_ratio_line = new QCPItemLine( _plot_ratio_widget );
            _plot_ratio_widget->addItem( _smoothed_ratio_line );
            _velocity_ratio_line = new QCPItemLine( _plot_velocity_widget );
            _plot_velocity_widget->addItem( _velocity_ratio_line );
            _smoothed_velocity_ratio_line = new QCPItemLine( _plot_velocity_widget );
            QPen pen( Qt::green );
            pen.setWidth( 2 );
            _smoothed_velocity_ratio_line->setPen( pen );
            _smoothed_velocity_ratio_line->start->setCoords( 0.f, 0.f );
            _smoothed_velocity_ratio_line->end->setCoords( 0.f, _plot_velocity_max_y );
            _plot_velocity_widget->addItem( _smoothed_velocity_ratio_line );
            _smoothed_ratio_line->setPen( pen );
            _smoothed_ratio_line->start->setCoords( 0.f, 0.f );
            _smoothed_ratio_line->end->setCoords( 0.f, _plot_ratio_max_y );
            _plot_ratio_widget->addItem( _smoothed_ratio_line );

            _plot_timer = new QTimer( this );
            connect( _plot_timer, SIGNAL( timeout() ), this, SLOT( update_ratio_line() ) );
            _plot_timer->start( 10 );
        }
        else
        {
            _model->camera_callback()->set_turning_mixture( 0.f );
            // It appears that _ratio_line is deleted when removed.
            if ( _plot_timer )
            {
                _plot_ratio_widget->removeItem( _ratio_line );
                _plot_ratio_widget->removeItem( _smoothed_ratio_line );
                _plot_velocity_widget->removeItem( _velocity_ratio_line );
                _plot_velocity_widget->removeItem( _smoothed_velocity_ratio_line );

                _plot_timer->stop();
                delete _plot_timer;
                _plot_timer = 0;

                _plot_ratio_widget->replot();
                _plot_velocity_widget->replot();
            }
        }

        if ( is_connected )
            setup_connection();
    }
}

void
Console::update_plot( int row, int column )
{
    update_plot();
}

void
Console::update_velocity_plot( int row, int column )
{
    update_velocity_plot();
}

void
Console::initialize_plot_widgets( std::vector< double > x, std::vector< double > y,
                                  std::vector< double > vx, std::vector< double > vy,
                                  bool create_plot )
{
    // Threshold Turning widget
    // Ensure there is one threshold display widget.

    if ( _ui->ratioDisplayWidget->count() == 0 )
    {
        QWidget* widget = new QWidget();
        _ui->ratioDisplayWidget->addWidget( widget );
    }
    else
        // Remove all current layout items in `_ui->ratioDisplayWidget'
        while ( _ui->ratioDisplayWidget->count() > 1 )
        {
            QWidget* widget = _ui->ratioDisplayWidget->currentWidget();
            _ui->ratioDisplayWidget->removeWidget( widget );
        }

    if ( _ui->velocityDisplayWidget->count() == 0 )
    {
        QWidget* widget = new QWidget();
        _ui->velocityDisplayWidget->addWidget( widget );
    }
    else
        // Remove all current layout items in `_ui->ratioDisplayWidget'
        while ( _ui->velocityDisplayWidget->count() > 1 )
        {
            QWidget* widget = _ui->velocityDisplayWidget->currentWidget();
            _ui->velocityDisplayWidget->removeWidget( widget );
        }

    // Initialize plot values
    _ui->plotTableWidget->setRowCount( x.size() );
    for ( int i = 0; i < x.size(); i++ )
    {
        QTableWidgetItem* item = new QTableWidgetItem;
        item->setData( Qt::EditRole, x[i] );
        _ui->plotTableWidget->setItem( i, 0, item );

        item = new QTableWidgetItem;
        item->setData( Qt::EditRole, y[i] );
        _ui->plotTableWidget->setItem( i, 1, item );
    }

    _ui->plotVelocityTableWidget->setRowCount( vx.size() );
    for ( int i = 0; i < vx.size(); i++ )
    {
        QTableWidgetItem* item = new QTableWidgetItem;
        item->setData( Qt::EditRole, vx[i] );
        _ui->plotVelocityTableWidget->setItem( i, 0, item );

        item = new QTableWidgetItem;
        item->setData( Qt::EditRole, vy[i] );
        _ui->plotVelocityTableWidget->setItem( i, 1, item );
    }

    // Create the plot widgets
    if ( create_plot )
    {
        _plot_ratio_widget = new QCustomPlot( _ui->ratioDisplayWidget->widget( 0 ) );
        QVBoxLayout* layout = new QVBoxLayout( _ui->ratioDisplayWidget->widget( 0 ) );
        layout->addWidget( _plot_ratio_widget );

        _plot_velocity_widget = new QCustomPlot( _ui->velocityDisplayWidget->widget( 0 ) );
        QVBoxLayout* layout2 = new QVBoxLayout( _ui->velocityDisplayWidget->widget( 0 ) );
        layout2->addWidget( _plot_velocity_widget );
    }

    update_velocity_plot();
}

void
Console::update_plot()
{
    bool valid = _plot_ratio_widget && _ui->plotTableWidget->rowCount() > 0;
    for ( int i = 0; i < _ui->plotTableWidget->rowCount() && valid; i++ )
    {
        if ( _ui->plotTableWidget->item( i, 0 ) == 0 || _ui->plotTableWidget->item( i, 1 ) == 0 )
            valid = false;
        else
        {
            bool ok1, ok2;
            double d = _ui->plotTableWidget->item( i, 0 )->data( Qt::EditRole ).toDouble( &ok1 );
            d = _ui->plotTableWidget->item( i, 1 )->data( Qt::EditRole ).toDouble( &ok2 );
            if ( !( ok1 && ok2 ) )
                valid = false;
        }
    }

    if ( valid )
    {
        if ( _plot_ratio_widget->graphCount() > 0 )
            _plot_ratio_widget->clearGraphs();

        _plot_ratio_widget->addGraph();
        _plot_ratio_widget->graph( 0 )->setPen( QPen( Qt::blue ) ); // line color blue for first graph
        _plot_ratio_widget->graph( 0 )->setBrush( QBrush( QColor( 0, 0, 255, 20 ) ) );

        int rows = _ui->plotTableWidget->rowCount();
        int v_rows = _ui->plotVelocityTableWidget->rowCount();
        QVector<double> x, y, rad_y;
        x.resize( rows );
        y.resize( rows );
        rad_y.resize( rows );

        _plot_ratio_max_y = -100000.f;
        _plot_velocity_max_y = -100000.f;

        for ( int i = 0; i < rows; ++i )
        {
            x[i] = _ui->plotTableWidget->item( i, 0 )->data( Qt::EditRole ).toDouble();
            y[i] = _ui->plotTableWidget->item( i, 1 )->data( Qt::EditRole ).toDouble();
            rad_y[i] = radians( y[i] );

            _plot_ratio_max_y = std::max( _plot_ratio_max_y, y[i] );
        }

        _plot = Graph_Evaluator( x.toStdVector(), rad_y.toStdVector() );

        if ( _model->is_tracking() )
        {
            _model->camera_callback()->set_plot_data( _plot, _v_plot );
        }

        // configure right and top axis to show ticks but no labels (could've also just called _plot_ratio_widget->setupFullAxesBox):
        _plot_ratio_widget->xAxis->setScaleType( QCPAxis::stLogarithmic );
        _plot_ratio_widget->xAxis2->setVisible( true );
        _plot_ratio_widget->xAxis2->setTickLabels( false );
        _plot_ratio_widget->yAxis2->setVisible( true );
        _plot_ratio_widget->yAxis2->setTickLabels( false );
        // pass data points to graphs:
        _plot_ratio_widget->graph( 0 )->setData( x, y );
        // let the ranges scale themselves so graph 0 fits perfectly in the visible area:
        _plot_ratio_widget->graph( 0 )->rescaleAxes();
        // Note: we could have also just called _plot_ratio_widget->rescaleAxes(); instead
        // make range moveable by mouse interaction (click and drag):
        _plot_ratio_widget->axisRect()->setRangeDrag( Qt::Horizontal | Qt::Vertical );
        _plot_ratio_widget->axisRect()->setRangeZoom( Qt::Horizontal | Qt::Vertical );
        _plot_ratio_widget->setInteractions( QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                             QCP::iSelectLegend | QCP::iSelectPlottables );

        _plot_ratio_widget->replot();
    }
}

void
Console::update_velocity_plot()
{
    bool valid = _plot_velocity_widget && _ui->plotVelocityTableWidget->rowCount() > 0;
    for ( int i = 0; i < _ui->plotVelocityTableWidget->rowCount() && valid; i++ )
    {
        if ( _ui->plotVelocityTableWidget->item( i, 0 ) == 0 || _ui->plotVelocityTableWidget->item( i, 1 ) == 0 )
            valid = false;
        else
        {
            bool ok1, ok2;
            double d = _ui->plotVelocityTableWidget->item( i, 0 )->data( Qt::EditRole ).toDouble( &ok1 );
            d = _ui->plotVelocityTableWidget->item( i, 1 )->data( Qt::EditRole ).toDouble( &ok2 );
            if ( !( ok1 && ok2 ) )
                valid = false;
        }
    }

    if ( valid )
    {
        if ( _plot_velocity_widget->graphCount() > 0 )
            _plot_velocity_widget->clearGraphs();

        _plot_velocity_widget->addGraph();
        _plot_velocity_widget->graph( 0 )->setPen( QPen( Qt::blue ) ); // line color blue for first graph
        _plot_velocity_widget->graph( 0 )->setBrush( QBrush( QColor( 0, 0, 255, 20 ) ) );

        int rows = _ui->plotVelocityTableWidget->rowCount();
        QVector<double> x, y;
        double min_y = 10000, max_y = -10000;
        x.resize( rows );
        y.resize( rows );
        for ( int i = 0; i < rows; ++i )
        {
            x[i] = _ui->plotVelocityTableWidget->item( i, 0 )->data( Qt::EditRole ).toDouble();
            y[i] = _ui->plotVelocityTableWidget->item( i, 1 )->data( Qt::EditRole ).toDouble();
            min_y = std::min( min_y, y[i] );
            max_y = std::max( max_y, y[i] );
        }

        if ( max_y - min_y < 0.00001 )
        {
            max_y *= 1.01;
            min_y *= 0.99;
        }
        _plot_velocity_max_y = max_y;

        _v_plot = Graph_Evaluator( x.toStdVector(), y.toStdVector() );

        // configure right and top axis to show ticks but no labels (could've also just called _plot_velocity_widget->setupFullAxesBox):
        _plot_velocity_widget->xAxis->setScaleType( QCPAxis::stLogarithmic );
        _plot_velocity_widget->xAxis2->setVisible( true );
        _plot_velocity_widget->xAxis2->setTickLabels( false );
        _plot_velocity_widget->yAxis2->setVisible( true );
        _plot_velocity_widget->yAxis2->setTickLabels( false );
        // pass data points to graphs:
        _plot_velocity_widget->graph( 0 )->setData( x, y );
        _plot_velocity_widget->rescaleAxes();
        //      _plot_velocity_widget->xAxis->setRange( x.first(), x.last() );
        //      _plot_velocity_widget->yAxis->setRange( min_y, max_y );

        // make range moveable by mouse interaction (click and drag):
        _plot_velocity_widget->axisRect()->setRangeDrag( Qt::Horizontal | Qt::Vertical );
        _plot_velocity_widget->axisRect()->setRangeZoom( Qt::Horizontal | Qt::Vertical );
        _plot_velocity_widget->setInteractions( QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                                QCP::iSelectLegend | QCP::iSelectPlottables );


        _plot_velocity_widget->replot();
    }
}

void
Console::add_row()
{
    if ( _ui->plotTableWidget->currentRow() == -1 )
        _ui->plotTableWidget->insertRow( _ui->plotTableWidget->rowCount() );
    else
        _ui->plotTableWidget->insertRow( _ui->plotTableWidget->currentRow() + 1 );

    update_plot();
}

void
Console::delete_row()
{
    _ui->plotTableWidget->removeRow( _ui->plotTableWidget->currentRow() );

    update_plot();
}

void
Console::add_row_velocity()
{
    if ( _ui->plotVelocityTableWidget->currentRow() == -1 )
        _ui->plotVelocityTableWidget->insertRow( _ui->plotVelocityTableWidget->rowCount() );
    else
        _ui->plotVelocityTableWidget->insertRow( _ui->plotVelocityTableWidget->currentRow() + 1 );

    update_velocity_plot();
}

void
Console::delete_row_velocity()
{
    _ui->plotVelocityTableWidget->removeRow( _ui->plotVelocityTableWidget->currentRow() );

    update_velocity_plot();
}

void
Console::update_threshold_weight( int val )
{
    float scale_fac = compute_turning_mixture( val );

    _ui->thresholdWeight->setValue( ( ( int )( roundf( scaling_val * scale_fac ) ) ) / scaling_val );

    if ( _model->is_tracking() )
    {
        _model->camera_callback()->set_turning_mixture( scale_fac );
    }
}

void
Console::update_threshold_scale( double val )
{
    float range = roundf( scaling_val * ( _ui->maxThresholdScale->value() - _ui->minThresholdScale->value() ) ) / scaling_val;
    _ui->thresholdTurningSlider->setValue( ( int ) roundf( ( ( _ui->thresholdWeight->value() - _ui->minThresholdScale->value() ) / range ) * 100.f ) );

    float scale_fac = compute_turning_mixture( _ui->thresholdTurningSlider->value() );

    _ui->thresholdWeight->setValue( ( ( int )( roundf( scaling_val * scale_fac ) ) ) / scaling_val );

    if ( _model->is_tracking() )
    {
        _model->camera_callback()->set_turning_mixture( scale_fac );
    }
}

void
Console::update_auto_heading_turn_rate( double val )
{
    _model->set_auto_heading_turn_rate( radians( val ) );
}

void
Console::set_crossbar_width( double value )
{
    if ( _viewer && _model->physics_enabled() )
        _model->set_crossbar_width( value );
}

void
Console::restrict_vertical_motion( bool yes_or_no )
{
    _model->restrict_vertical_motion( yes_or_no );
}

void
Console::set_minimum_velocity_thresold( double value )
{
    _model->set_minimum_velocity_thresold( value );
}

void
Console::set_output_rate( int value )
{
    if ( _viewer )
        _viewer->set_output_rate( value );
}

void
Console::show_invisible_objects( bool yes_or_no )
{
    _model->show_invisible_objects( yes_or_no );
}

void
Console::update_offset( double value )
{
    _model->move_offset( _ui->xoffset->value(), _ui->yoffset->value(), _ui->zoffset->value() );
}

void
Console::change_shader( int value )
{
    _model->change_shader( value );
}

void
Console::update_viewport( int value, int lower, int upper )
{
    if ( _viewer )
        _viewer->update_viewport( value - 1, lower, upper );
}

void
Console::load_config( Config_Memento const* cm )
{
    int num_cameras = cm->_numCameras;
    if ( _viewer )
        num_cameras = _viewer->number_of_cameras();

    if ( cm->_numCameras != num_cameras )
    {
        std::string msg( "Mismatch between number of cameras in config file, " );
        msg += "and those used to initialize the display:\n\n config file = " + std::to_string( ( long long )cm->_numCameras ) +
               " versus initialized = " + std::to_string( ( long long )num_cameras ) + "\n\nignoring config file";

        int ret = QMessageBox::warning( this, tr( "Mismatch in number of cameras" ), tr( msg.c_str() ) );
    }
    else
    {
        _ui->numCamerasSpinBox->setValue( cm->_numCameras );
        _ui->numDisplaysSpinBox->setValue( cm->_num_displays );
        _ui->startingDisplaySpinBox->setValue( cm->_starting_display );

        // All thoughout this routine we set the check box to the negated value
        // and then back. We do this to force Qt to actually call the callback as
        // it won't do it if the current state and the new one are the same. It's
        // pointless if the widget doesn't have a callback, so we do it for
        // consistancy
        _ui->gangMotionCheckBox->setChecked( cm->_gangMotion );

        _ui->xCenterDistortsBox->setChecked( cm->_xCenterDistorts );

        for ( int i = 0; i < num_cameras; i++ )
        {
            _frame[ i ] = cm->_frame[i];
            _distort[ i ] = cm->_distort[i];
            _center[ i ] = cm->_center[i];

            if ( _viewer )
            {
                update_viewport( i + 1, _frame[ i ].first, _frame[ i ].second );
                _viewer->set_x_focal_length( i, _distort[ i ].second / 100.f );
                _viewer->set_y_focal_length( i, _distort[ i ].third / 100.f );
                _viewer->set_center( i, _ui->xCenterDistortsBox->isChecked(),
                                     -_center[ i ].first / 100.f, -_center[ i ].second / 100.f );
            }
        }

        _ui->enableDistortion->setChecked( _distort[ _ui->cameraSelectSpinBox->value() - 1 ].first );
        _ui->focalLengthSlider_y->setValue( _distort[ _ui->cameraSelectSpinBox->value() - 1 ].second );
        _ui->focalLengthSlider_x->setValue( _distort[ _ui->cameraSelectSpinBox->value() - 1 ].third );
        _ui->xCenterSlider->setValue( _center[ _ui->cameraSelectSpinBox->value() - 1 ].first );
        _ui->yCenterSlider->setValue( _center[ _ui->cameraSelectSpinBox->value() - 1 ].second );

        if ( _viewer )
            _viewer->setup_slaves();

        if ( cm->_connection_type == "TCP" )
            _ui->TCP_radioBtn->setChecked( true );
        else if ( cm->_connection_type == "UDP" )
            _ui->UDP_radioBtn->setChecked( true );
        else
            _ui->shmem_radioBtn->setChecked( true );

        char val[256];

        sprintf( val, "%d", cm->_retries );
        _ui->retriesCtrl->setText( val );
        sprintf( val, "%d", cm->_port );
        _ui->portCtrl->setText( val );
        _ui->hostnameCtrl->setText( cm->_host.c_str() );

        // Graph_Widget
        for ( int i = 0; i < num_cameras; i++ )
        {
            _graphics_view[i]->load( cm->_graph[i] );
        }

        _ui->horizontalBlankingSlider->setValue( _frame[ _last_camera - 1 ].first );
        _ui->horizontalBlankingSlider_2->setValue( _frame[ _last_camera - 1 ].second );

        set_ambient_color( cm->_ambient );
        set_diffuse_color( cm->_diffuse );
        set_diffuse_power( cm->_power );
        set_background_color( cm->_background );

        _brightness = cm->_global_brightness;
        if ( !_blank_display )
        {
            for ( int i = 0; i < num_cameras; i++ )
                set_global_brightness( i, cm->_global_brightness[i] );

            // Sets to 0-offset in call
            set_global_brightness_target( 1 );
        }

        // Can't directly create a QString from a std::string, so use the C string
        _ui->shaderComboBox->setCurrentText( cm->_shader_choice.c_str() );
        _ui->textureResComboBox->setCurrentText( cm->_texture_resolution.c_str() );

        // Calibration Parameters
        _ui->calibrateXValue->setValue( cm->_calib_x );
        _ui->calibrateXGain->setValue( cm->_gain_x );

        _ui->calibrateYValue->setValue( cm->_calib_y );
        _ui->calibrateYGain->setValue( cm->_gain_y );

        _ui->calibrateZValue->setValue( cm->_calib_z );
        _ui->calibrateZGain->setValue( cm->_gain_z );

        _start_index = cm->_start_location;
        // Reset the start index if the config was saved with a larger index
        if ( _model->start_locations().size() < _start_index )
            _start_index = 0;

        if ( _model->start_locations().size() > _start_index )
        {
            _start_location_widgets->select( _start_index );
            jump_to_location();
        }

        _ui->velocitySmoothingEnabled->setChecked( !cm->_enable_vel_smoothing );
        _ui->velocitySmoothingEnabled->setChecked( cm->_enable_vel_smoothing );
        _ui->inputSmoothingEnabled->setChecked( !cm->_enable_input_smoothing );
        _ui->inputSmoothingEnabled->setChecked( cm->_enable_input_smoothing );
        _ui->intermediarySmoothingEnabled->setChecked( !cm->_enable_intermediary_smoothing );
        _ui->intermediarySmoothingEnabled->setChecked( cm->_enable_intermediary_smoothing );
        _ui->outputSmoothingEnabled->setChecked( !cm->_enable_output_smoothing );
        _ui->outputSmoothingEnabled->setChecked( cm->_enable_output_smoothing );
        _ui->velocitySmoothingInterval->setValue( cm->_vel_smoothing_interval );
        _ui->inputSmoothingInterval->setValue( cm->_input_interval );
        _ui->intermediarySmoothingInterval->setValue( cm->_intermediary_interval );
        _ui->outputSmoothingInterval->setValue( cm->_output_interval );

        _ui->ballRadiusBox->setValue( cm->_ball_radius );
        _ui->xoffset->setValue( cm->_x_offset );
        _ui->yoffset->setValue( cm->_y_offset );
        _ui->zoffset->setValue( cm->_z_offset );

        _ui->crossbarEnabledCheckBox->setChecked( !cm->_crossbar_enabled );
        _ui->crossbarEnabledCheckBox->setChecked( cm->_crossbar_enabled );
        _ui->crossbarWidthSpinBox->setValue( cm->_crossbar_width );
        _ui->minimumVelocityThreshold->setValue( cm->_minimum_velocity_thresold );
        _ui->restrictVerticalMotion->setChecked( !cm->_restrict_vertical_motion );
        _ui->restrictVerticalMotion->setChecked( cm->_restrict_vertical_motion );

        int index = _ui->outputPortName->findText( cm->_output_port_name.c_str() );
        if ( index >= 0 )
            _ui->outputPortName->setCurrentIndex( index );
        else
            _ui->outputPortName->setCurrentIndex( 0 );
        set_output_serial_port_name( index );

        index = _ui->dataServerPortName->findText( cm->_data_server_port_name.c_str() );
        if ( index >= 0 )
            _ui->dataServerPortName->setCurrentIndex( index );
        else
            _ui->dataServerPortName->setCurrentIndex( 0 );

        _ui->outputFormatSelector->setCurrentIndex( cm->_output_format_id );
        _ui->outputRateSpinBox->setValue( cm->_output_rate );

        _ui->rotatedCamerasCheckBox->setChecked( cm->_rotatedCameras );
        _ui->fovSlider->setValue( cm->_fov );
        _ui->fovOffsetSpinBox->setValue( cm->_fov_offset );
        set_rotated_cameras( cm->_rotatedCameras );

        _ui->frameModeComboBox->setCurrentIndex( cm->_frame_mode );
        _ui->indicatorSizeSlider->setValue( cm->_indicator_size );
        _ui->borderSizeSlider->setValue( cm->_border_size );
        _ui->screenSelectSpinBox->setValue( cm->_screen_select );
        _ui->indicatorHorizontalPositionSlider->setValue( cm->_indicator_horizontal_pos );
        _ui->indicatorVerticalPositionSlider->setValue( cm->_indicator_vertical_pos );

        // Flip the sense of the widget so that the real callback occurs
        _ui->thresholdTurningEnabled->setChecked( !cm->_enable_threshold_turning );
        _ui->thresholdTurningEnabled->setChecked( cm->_enable_threshold_turning );
        _ui->minThresholdScale->setValue( cm->_min_threshold_scale );
        _ui->maxThresholdScale->setValue( cm->_max_threshold_scale );
        _ui->thresholdTurningSlider->setValue( cm->_current_threshold_scale );

        initialize_plot_widgets( cm->_plot_x, cm->_plot_y, cm->_plot_vel_x, cm->_plot_vel_y, false );
    }
}

void
Console::save_config( Config_Memento* cm )
{
    if ( _viewer )
        cm->_numCameras = _viewer->number_of_cameras();
    else
        cm->_numCameras = _ui->numCamerasSpinBox->value();

    cm->_num_displays = _ui->numDisplaysSpinBox->value();
    cm->_starting_display = _ui->startingDisplaySpinBox->value();
    cm->_gangCameras = _ui->gangedCheckBox->isChecked();
    cm->_xCenterDistorts = _ui->xCenterDistortsBox->isChecked();

    if ( _viewer )
        reset_sliders( _last_camera );

    cm->_frame = _frame;
    cm->_distort = _distort;
    cm->_center = _center;

    if ( _ui->TCP_radioBtn->isChecked() )
        cm->_connection_type = "TCP";
    else if ( _ui->UDP_radioBtn->isChecked() )
        cm->_connection_type = "UDP";
    else
        cm->_connection_type = "Shared Memory";

    cm->_retries = retries();
    from_string<int>( cm->_port, port(), std::dec );
    cm->_host = hostname();

    cm->_gangMotion = _ui->gangMotionCheckBox->isChecked();

    cm->_frame_packing = _ui->framePackingEnabled->isChecked();
    cm->_graph.resize( cm->_numCameras );
    for ( int i = 0; i < cm->_numCameras; i++ )
        _graphics_view[i]->save( cm->_graph[i] );

    cm->_ambient = QColor( _ui->ambientLabel->text() );
    cm->_diffuse = QColor( _ui->diffuseLabel->text() );
    cm->_power = _ui->diffusePowerLevel->value();
    cm->_background = QColor( _ui->backgroundLabel->text() );
    cm->_global_brightness = _brightness;
    cm->_shader_choice = _ui->shaderComboBox->currentText().toStdString();
    cm->_texture_resolution = _ui->textureResComboBox->currentText().toStdString();

    cm->_calib_x = _ui->calibrateXValue->value();
    cm->_gain_x = _ui->calibrateXGain->value();
    cm->_calib_y = _ui->calibrateYValue->value();
    cm->_gain_y = _ui->calibrateYGain->value();
    cm->_calib_z = _ui->calibrateZValue->value();
    cm->_gain_z = _ui->calibrateZGain->value();

    cm->_start_location = _start_index;

    cm->_enable_vel_smoothing = _ui->velocitySmoothingEnabled->isChecked();
    cm->_vel_smoothing_interval = _ui->velocitySmoothingInterval->value();
    cm->_enable_input_smoothing = _ui->inputSmoothingEnabled->isChecked();
    cm->_input_interval = _ui->inputSmoothingInterval->value();
    cm->_enable_output_smoothing = _ui->outputSmoothingEnabled->isChecked();
    cm->_output_interval = _ui->outputSmoothingInterval->value();
    cm->_enable_intermediary_smoothing = _ui->intermediarySmoothingEnabled->isChecked();
    cm->_intermediary_interval = _ui->intermediarySmoothingInterval->value();

    cm->_ball_radius = _ui->ballRadiusBox->value();
    cm->_x_offset = _ui->xoffset->value();
    cm->_y_offset = _ui->yoffset->value();
    cm->_z_offset = _ui->zoffset->value();

    cm->_crossbar_enabled = _ui->crossbarEnabledCheckBox->isChecked();
    cm->_crossbar_width = _ui->crossbarWidthSpinBox->value();
    cm->_minimum_velocity_thresold = _ui->minimumVelocityThreshold->value();
    cm->_restrict_vertical_motion = _ui->restrictVerticalMotion->isChecked();

    cm->_data_server_port_name = _ui->dataServerPortName->currentText().toStdString();
    cm->_output_port_name = _ui->outputPortName->currentText().toStdString();
    cm->_output_rate = _ui->outputRateSpinBox->value();

    cm->_output_format_id = _ui->outputFormatSelector->currentIndex();
    cm->_treadmill_output = _ui->treadmillOutputToggle->isChecked();
    cm->_reduced_output = _ui->reducedOutputToggle->isChecked();

    cm->_rotatedCameras = _ui->rotatedCamerasCheckBox->isChecked();
    cm->_fov = _ui->fovSlider->value();
    cm->_fov_offset = _ui->fovOffsetSpinBox->value();

    cm->_frame_mode = _ui->frameModeComboBox->currentIndex();
    cm->_indicator_size = _ui->indicatorSizeSlider->value();
    cm->_border_size = _ui->borderSizeSlider->value();
    cm->_screen_select = _ui->screenSelectSpinBox->value();
    cm->_indicator_horizontal_pos = _ui->indicatorHorizontalPositionSlider->value();
    cm->_indicator_vertical_pos = _ui->indicatorVerticalPositionSlider->value();

    cm->_enable_threshold_turning = _ui->thresholdTurningEnabled->isChecked();
    cm->_min_threshold_scale = _ui->minThresholdScale->value();
    cm->_max_threshold_scale = _ui->maxThresholdScale->value();
    cm->_current_threshold_scale = _ui->thresholdTurningSlider->value();

    int rows = _ui->plotTableWidget->rowCount();
    cm->_plot_x.resize( rows );
    cm->_plot_y.resize( rows );

    for ( int i = 0; i < rows; ++i )
    {
        cm->_plot_x[i] = _ui->plotTableWidget->item( i, 0 )->data( Qt::EditRole ).toDouble();
        cm->_plot_y[i] = _ui->plotTableWidget->item( i, 1 )->data( Qt::EditRole ).toDouble();
    }

    rows = _ui->plotVelocityTableWidget->rowCount();
    cm->_plot_vel_x.resize( rows );
    cm->_plot_vel_y.resize( rows );

    for ( int i = 0; i < rows; ++i )
    {
        cm->_plot_vel_x[i] = _ui->plotVelocityTableWidget->item( i, 0 )->data( Qt::EditRole ).toDouble();
        cm->_plot_vel_y[i] = _ui->plotVelocityTableWidget->item( i, 1 )->data( Qt::EditRole ).toDouble();
    }
}

void
Console::update_ratio_line()
{
    std::vector< float > const& vals = _model->camera_callback()->ratio();
    float ratio = std::abs( vals[0] );
    float vel = vals[1];
    float smoothed_ratio = std::abs( vals[2] );
    float smoothed_vel = vals[3];

    //std::cout << ratio << ", " << _plot_ratio_max_y << std::endl;

    _ratio_line->start->setCoords( ratio, 0.f );
    _ratio_line->end->setCoords( ratio, _plot_ratio_max_y );
    _velocity_ratio_line->start->setCoords( vel, 0.f );
    _velocity_ratio_line->end->setCoords( vel, _plot_velocity_max_y );

    if ( _ui->inputSmoothingEnabled->isChecked() )
    {
        _smoothed_ratio_line->start->setCoords( smoothed_ratio, 0.f );
        _smoothed_ratio_line->end->setCoords( smoothed_ratio, _plot_ratio_max_y );
    }

    if ( _ui->velocitySmoothingEnabled->isChecked() )
    {
        _smoothed_velocity_ratio_line->start->setCoords( smoothed_vel, 0.f );
        _smoothed_velocity_ratio_line->end->setCoords( smoothed_vel, _plot_velocity_max_y );
    }

    _plot_ratio_widget->replot();
    _plot_velocity_widget->replot();
}

void
Console::set_export_filename( QString const& fileName )
{
    QString temp( fileName );
    switch ( _ui->exportFileNameStyle->currentIndex() )
    {
        case 0:
            temp += "_out.txt";
            break;
        case 1:
        {
            char s[40];
            sprintf( s, "_out_%d.txt", _connection_counter );
            temp += s;
        }
        break;
        case 2:
        {
            using namespace boost::gregorian;
            using namespace boost::posix_time;
            using namespace boost::local_time;

            // This is a pretty convoluted way to set the date_time, but it's
            // the way boost does it. For some reason, you have grab the current time
            // and put it into the local_date_time, and then read it back, otherwise,
            // boost won't format it the way we would like
            //
            // The two dynamically allocated time facests below are memory leaks, but if
            // I delete them like a good citized, the program crashes deep in the bowels
            // of Qt. For now, I'll let them leak (they're rather small, and this is called
            // infrequently), until a time I can find the crash.
            std::stringstream ss;
            local_time_facet* output_facet = new local_time_facet();
            local_time_input_facet* input_facet = new local_time_input_facet();
            ss.imbue( std::locale( std::locale::classic(), output_facet ) );
            ss.imbue( std::locale( ss.getloc(), input_facet ) );
            ptime now = second_clock::local_time();
            local_date_time ldt( not_a_date_time );
            ss << now;
            ss >> ldt;

            output_facet->format( "%Y%m%dT%H%M%S%F" );
            ss.str( "" );
            ss << ldt;

            temp += "_out_";
            temp += ss.str().c_str();
            temp += ".txt";

        }
        break;
        default:
            temp += "_out.txt";
            break;
    }

    _model->set_export_filename( temp );
}

void
Console::update_timer()
{
    QTime t = _ui->timeTrialEditor->time();
    int s = t.second();
    int m = t.minute();
    int h = t.hour();
    s--;
    if ( s < 0 )
    {
        s = 59;
        m--;
        if ( m < 0 )
        {
            m = 59;
            h--;
        }
    }

    t.setHMS( h, m, s );
    _ui->timeTrialEditor->setTime( t );
}

float
Console::compute_turning_mixture( float val )
{
    float range = roundf( scaling_val * ( _ui->maxThresholdScale->value() - _ui->minThresholdScale->value() ) ) / scaling_val;
    float fac = val / 100.f;
    float scale_fac = range * fac + _ui->minThresholdScale->value();

    return scale_fac;
}

