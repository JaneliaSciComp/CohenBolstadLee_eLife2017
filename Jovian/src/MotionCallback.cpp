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

#include <algorithm>
using std::min;
using std::max;

#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <osg/Quat>
#include <osg/io_utils>

#include <CameraUpdateCallback.h>
#include <Console.h>
#include <MotionCallback.h>

double Pi = boost::math::constants::pi< double >();
double Two_Pi = boost::math::constants::two_pi< double >();

MovementCallback::MovementCallback( osg::MatrixTransform* trackNode ):
    _track_node( trackNode ), _axis( 0.f, -1.f, 0.f ) {}

void
MovementCallback::operator()( const btTransform& worldTrans )
{
    osg::Vec3f worldPt( worldTrans.getOrigin()[0],
                        worldTrans.getOrigin()[1],
                        worldTrans.getOrigin()[2] );

    osg::Matrixd matrix;
    matrix.makeTranslate( worldPt );

    _track_node->setMatrix( matrix );
}

ChannelMovementCallback::ChannelMovementCallback( osg::MatrixTransform* trackNode,
                                                  array_type* tangent_grid,
                                                  osg::Vec3f min_point,
                                                  float min_dist ): _track_node( trackNode ),
    _axis( 0.f, -1.f, 0.f ),
    _tangent_grid( tangent_grid ),
    _min_point( min_point ),
    _min_dist( min_dist ) {}

void
ChannelMovementCallback::operator()( const btTransform& worldTrans )
{
    osg::Vec3f worldPt( worldTrans.getOrigin()[0],
                        worldTrans.getOrigin()[1],
                        worldTrans.getOrigin()[2] );
    osg::Vec3f pt = ( worldPt - _min_point ) / _min_dist;
    int lx = int( floorf( pt.x() ) );
    int ux = int( ceilf( pt.x() ) );
    int ly = int( floorf( pt.y() ) );
    int uy = int( ceilf( pt.y() ) );
    osg::Vec3f interp( 0.f, 0.f, 0.f );
    osg::Vec3f* v = ( *_tangent_grid )[ lx ][ ly ];
    if ( v != 0 )
        interp = *v;
    v = ( *_tangent_grid )[ ux ][ ly ];
    if ( v != 0 )
        interp += *v;
    v = ( *_tangent_grid )[ lx ][ uy ];
    if ( v != 0 )
        interp += *v;
    v = ( *_tangent_grid )[ ux ][ uy ];
    if ( v != 0 )
        interp += *v;

    interp /= 4.f;
    osg::Matrixd matrix;
    double cos_theta = _axis * interp;
    //    std::cout << interp.x() << " " << interp.y() << " " << acos( cos_theta ) << std::endl;

    matrix.makeRotate( _axis, interp );
    matrix.postMultTranslate( worldPt );

    _track_node->setMatrix( matrix );
}

CameraMotionUpdateCallback::CameraMotionUpdateCallback( btRigidBody* body,
                                                        Collision_World* dynamics_world,
                                                        osg::ref_ptr< osgGA::NodeTrackerManipulator > tracker,
                                                        ContactNodes const& nodes ):
    _body( body ), _dynamics_world( dynamics_world ), _tracker( tracker ),
    _traversed( false ), _first( false ), _initializing( true ), _enable_turning( false ),
    _use_dynamics( false ), _use_vel_smoothing( false ),
    _use_input_heading_smoothing( false ), _use_output_heading_smoothing( false ),
    _vel_frame_index( 0 ), _interval_for_vel_smoothing( 32 ),
    _interval_for_input_smoothing( 32 ), _interval_for_intermediary_smoothing(32),
    _interval_for_output_smoothing( 32 ), _vel_frames( 2 ), _tt_vel_frames( 2 ),
    _input_ratio( 2 ), _output_angle( 2 ), _intermediary_smoothing_val( 0.f ),
    _input_smoothing_val( osg::Vec3d( 0., 0., 0. ) ), _smoothing_vel( osg::Vec3d( 0., 0., 0. ) ),
    _smoothing_tt_vel( osg::Vec3d( 0., 0., 0. ) ), _output_smoothing_val( 0 ), _turning_mixture( 0.f ),
    _restrict_vertical_motion( false ), _minimum_velocity_thresold( 0.f ), _y_axis( 0, 1, 0 ),
    _ratio_values( 4 ), _contact_nodes( nodes )
{
    //This set the framerate, but will also resize all of the smoothing buffers
    set_average_framerate( 0.0167f );
}

void
CameraMotionUpdateCallback::interval_for_velocity_smoothing( int interval )
{
    _interval_for_vel_smoothing = ( interval >= 0 ? interval : _interval_for_vel_smoothing );
    compute_frame_buffer_size( _frame_rate, _interval_for_vel_smoothing, &_vel_frames,
                               &_smoothing_vel, osg::Vec3d( 0., 0., 0. ) );

    compute_frame_buffer_size( _frame_rate, _interval_for_vel_smoothing,
                               &_tt_vel_frames, &_smoothing_tt_vel, osg::Vec3d( 0., 0., 0. ) );
}

void
CameraMotionUpdateCallback::interval_for_heading_smoothing( int input_interval,
                                                            int intermediary_interval,
                                                            int output_interval )
{
    _interval_for_input_smoothing = ( input_interval >= 0 ? input_interval : _interval_for_input_smoothing );
    _interval_for_intermediary_smoothing = ( intermediary_interval >= 0 ? intermediary_interval : _interval_for_intermediary_smoothing );
    _interval_for_output_smoothing = ( output_interval >= 0 ? output_interval : _interval_for_output_smoothing );

    compute_frame_buffer_size( _frame_rate, _interval_for_input_smoothing, &_input_ratio, &_input_smoothing_val, osg::Vec3d( 0., 0., 0. ) );

    compute_frame_buffer_size( _frame_rate, _interval_for_intermediary_smoothing, &_intermediary_angle, &_intermediary_smoothing_val, 0.f );

    compute_frame_buffer_size( _frame_rate, _interval_for_output_smoothing, &_output_angle, &_output_smoothing_val, 0.f );
}

