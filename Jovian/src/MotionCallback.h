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

#ifndef MOTIONCALLBACK_H
#define MOTIONCALLBACK_H

#include <set>
#include <utility>
#include <vector>

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/multi_array.hpp>
#endif

#include <osg/AnimationPath>
#include <osg/MatrixTransform>
#include <osgGA/NodeTrackerManipulator>

#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>

#include <osgbDynamics/MotionState.h>

#include <Globals.h>
#include <Graph_Evaluator.h>

extern double Pi;
extern double Two_Pi;

typedef boost::multi_array< osg::Vec3f*, 2 > array_type;

#define BIT(x) (1<<(x))
enum collision_types
{
    COL_NOTHING = 0, //<Collide with camera
    COL_CHANNEL = BIT( 0 ), //<Collide with channel
    COL_REPORTABLE = BIT( 1 ), //<Collide with reportable object
    COL_EE = BIT( 2 ), //<Collides with Everybody Else
    COL_CAMERA = BIT( 3 ) //<Collide with camera
};

// Floating-point modulo
// The result (the remainder) has same sign as the divisor.
// Similar to matlab's mod(); Not similar to fmod() -   Mod(-3,4)= 1   fmod(-3,4)= -3
template<typename T>
T Mod( T x, T y )
{
    if ( 0. == y )
        return x;

    double m = x - y * floor( x / y );

    // handle boundary cases resulted from floating-point cut off:

    if ( y > 0 )            // modulo range: [0..y)
    {
        if ( m >= y )       // Mod(-1e-16             , 360.    ): m= 360.
            return 0;

        if ( m < 0 )
        {
            if ( y + m == y )
                return 0  ; // just in case...
            else
                return y + m; // Mod(106.81415022205296 , Two_Pi ): m= -1.421e-14
        }
    }
    else                    // modulo range: (y..0]
    {
        if ( m <= y )       // Mod(1e-16              , -360.   ): m= -360.
            return 0;

        if ( m > 0 )
        {
            if ( y + m == y )
                return 0  ; // just in case...
            else
                return y + m; // Mod(-106.81415022205296, -Two_Pi): m= 1.421e-14
        }
    }

    return m;
}

template<typename T>
void
compute_frame_buffer_size( float frame_rate, int interval, boost::circular_buffer< T >* buffer, T* var, T const& zero_var )
{
    typename boost::circular_buffer< T >::iterator front, back;

    if ( frame_rate > 0.f )
    {
        int num_frames = interval / ( frame_rate * 1000 );
        num_frames = ( num_frames < 2 ) ? 2 : num_frames;
        front = buffer->begin();
        back = buffer->end();

        if ( num_frames != buffer->size() )
        {
            if ( num_frames < buffer->size() )
            {
                T t_var = zero_var;

                for ( int i = 0; i < buffer->size() - num_frames; i++ )
                {
                    t_var += *front;
                    front++;
                }
                ( *var ) -= t_var;

                boost::circular_buffer< T > new_buffer( num_frames, front, back );
                buffer->swap( new_buffer );
            }
            else
            {
                boost::circular_buffer< T > new_buffer( front, back );
                new_buffer.resize( num_frames, zero_var );
                buffer->swap( new_buffer );
            }
        }
    }
}


// wrap [rad] angle to [-PI..PI)
inline double WrapPosNegPI( double fAng )
{
    return Mod( fAng + Pi, Two_Pi ) - Pi;
}

// wrap [rad] angle to [0..Two_Pi)
inline double WrapTwoPi( double fAng )
{
    return Mod( fAng, Two_Pi );
}

// wrap [deg] angle to [-180..180)
inline double WrapPosNeg180( double fAng )
{
    return Mod( fAng + 180., 360. ) - 180.;
}

// wrap [deg] angle to [0..360)
inline double Wrap360( double fAng )
{
    return Mod( fAng , 360. );
}

inline double unwrap( double prev, double cur )
{
    double result = cur;
    while ( fabs( result - prev ) > Pi )
    {
        if ( result - prev > 0 )
            result -= Two_Pi;
        else
            result += Two_Pi;
    }

    return result;
}

inline std::ostream& operator<<( std::ostream& output, const btVector3& vec )
{
    output << "<" << vec.x() << ", "
           << vec.y() << ", "
           << vec.z() << ">";
    return output;     // to enable cascading
}


struct MovementCallback : public osgbDynamics::MotionStateCallback
{
    osg::MatrixTransform* _track_node;
    osg::Vec3f _axis;

    MovementCallback( osg::MatrixTransform* trackNode );

    virtual void operator()( const btTransform& worldTrans );
};

struct ChannelMovementCallback : public osgbDynamics::MotionStateCallback
{
    osg::MatrixTransform* _track_node;
    osg::Vec3f _axis;
    array_type* _tangent_grid;
    osg::Vec3f _min_point;
    float _min_dist;

    ChannelMovementCallback( osg::MatrixTransform* trackNode,
                             array_type* tangent_grid,
                             osg::Vec3f min_point,
                             float min_dist );

    virtual void operator()( const btTransform& worldTrans );
};

class CameraMotionUpdateCallback : public osg::NodeCallback
{
  public:
    CameraMotionUpdateCallback( btRigidBody* body,
                                Collision_World* dynamics_world,
                                osg::ref_ptr< osgGA::NodeTrackerManipulator > tracker,
                                ContactNodes const& nodes );

    virtual void clear() {}
    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv ) = 0;
    virtual void set_average_framerate( float rate ) { _frame_rate = rate; interval_for_velocity_smoothing( _interval_for_vel_smoothing ); interval_for_heading_smoothing( _interval_for_input_smoothing, _interval_for_intermediary_smoothing, _interval_for_output_smoothing ); }
    void enable_velocity_smoothing( bool on_or_off ) { _use_vel_smoothing = on_or_off; _vel_frames.clear(); _tt_vel_frames.clear(); _smoothing_vel = osg::Vec3d( 0.f, 0.f, 0.f ); _smoothing_tt_vel = osg::Vec3d( 0.f, 0.f, 0.f );}
    void enable_input_heading_smoothing( bool on_or_off ) { _use_input_heading_smoothing = on_or_off; _input_ratio.clear(); _input_smoothing_val = osg::Vec3d( 0.f, 0.f, 0.f );}
    void enable_intermediary_heading_smoothing( bool on_or_off ) { _use_intermediary_heading_smoothing = on_or_off; _intermediary_angle.clear(); _intermediary_smoothing_val = 0.f;}
    void enable_output_heading_smoothing( bool on_or_off ) { _use_output_heading_smoothing = on_or_off; _output_angle.clear(); _output_smoothing_val = 0.f;}
    void interval_for_velocity_smoothing( int interval );
    void interval_for_heading_smoothing( int input_interval,
                                         int intermediary_interval,
                                         int output_interval );
    void set_plot_data( Graph_Evaluator vals, Graph_Evaluator vels ) { _plot = vals; _v_plot = vels; }
    void enable_threshold_turning( bool yes_or_no ) { _enable_turning = yes_or_no; }
    void set_turning_mixture( float val ) { _turning_mixture = val; }
    void restrict_vertical_motion( bool yes_or_no )  { _restrict_vertical_motion = yes_or_no; }
    void set_minimum_velocity_thresold( float val ) { _minimum_velocity_thresold = val; }
    virtual void set_auto_heading_turn_rate( float value ) {}

    virtual void reset() { _traversed = false;}
    virtual void reset_accumulators()
    {
        _input_smoothing_val = osg::Vec3d( 0., 0., 0. );
        _intermediary_smoothing_val = 0.f;
        _output_smoothing_val = 0.f;
    }
    virtual void home( double start_time = 0.0 ) { _initializing = true; _traversed = false; _body->setWorldTransform( _startPos ); }
    void use_dynamics( bool on_or_off )  { _use_dynamics = on_or_off; }
    std::vector< float > const& ratio() const { return _ratio_values; }

  protected:

    btRigidBody* _body;
    Collision_World* _dynamics_world;
    osg::ref_ptr< osgGA::NodeTrackerManipulator > _tracker;
    btTransform   _startPos;
    int         _punchCount;
    double     _basetime;
    float       _frame_rate;
    bool       _traversed;
    bool       _first;
    bool _initializing;
    bool _enable_turning;
    float      _theta;
    bool      _use_dynamics, _use_vel_smoothing, _use_input_heading_smoothing;
    bool      _use_intermediary_heading_smoothing;
    bool      _use_output_heading_smoothing;
    int        _vel_frame_index, _head_frame_index, _interval_for_vel_smoothing, _interval_for_input_smoothing,
                _interval_for_intermediary_smoothing, _interval_for_output_smoothing;
    boost::circular_buffer< osg::Vec3d > _input_ratio, _vel_frames, _tt_vel_frames;
    boost::circular_buffer< float > _intermediary_angle, _output_angle;

    osg::Vec3d _input_smoothing_val, _smoothing_vel, _smoothing_tt_vel, _euler_vector;
    float _intermediary_smoothing_val, _output_smoothing_val;
    Graph_Evaluator _plot, _v_plot;
    float _turning_mixture;
    bool _restrict_vertical_motion;
    float _minimum_velocity_thresold;
    osg::Vec3d _y_axis;
    std::vector< float > _ratio_values;
    ContactNodes const& _contact_nodes;
};

#endif
