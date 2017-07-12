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
#include <iostream>

#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <osg/Quat>
#include <osg/io_utils>

#include "CameraPathUpdateCallback.h"

PathMovementCallback::PathMovementCallback( osg::MatrixTransform* trackNode ):
    _track_node( trackNode ) {}

void
PathMovementCallback::operator()( const btTransform& worldTrans )
{
    osg::Vec3f worldPt( worldTrans.getOrigin()[0],
                        worldTrans.getOrigin()[1],
                        worldTrans.getOrigin()[2] );

    osg::Matrixd matrix;
    matrix.makeTranslate( worldPt );

    _track_node->setMatrix( matrix );
}

CameraPathUpdateCallback::CameraPathUpdateCallback( btRigidBody* body, btRigidBody* sliding_body,
                                                    Collision_World* dynamics_world,
                                                    osg::ref_ptr< osgGA::NodeTrackerManipulator > tracker,
                                                    ContactNodes const& nodes, osg::Vec3d start_dir,
                                                    osg::AnimationPath* ap, double start_time ):
    CameraThresholdUpdateCallback( body, dynamics_world, tracker, nodes, start_dir ),
    _sliding_body( sliding_body ),
    _firstTime( 0.f ), _start_time( start_time ), _deltaTime( 0.f ),
    _minimum_distance( 99999999.f ),
    _averaging( true ), _flip_tangent( false ),
    _auto_heading_turn_rate( 1.57079 )
{
    _orig_ap = ap;

    if ( _dynamics_world->has_crossbar )
    {
        // now get constraint frame in local coordinate systems
        btTransform frameInA = _sliding_body->getCenterOfMassTransform().inverse();
        //    btTransform frameInB = _body->getCenterOfMassTransform().inverse() * _frameInW;
        btTransform frameInB = btTransform::getIdentity();

        //    frameInA.setIdentity();
        //    frameInB.setIdentity();

        frameInA.setOrigin( btVector3( 0, 0, 0 ) );

        _constraint = new btGeneric6DofConstraint( *_sliding_body, *_body, frameInA, frameInB, true );
        btVector3 lowerSliderLimit( -_dynamics_world->crossbar_width, -1., -1. );
        btVector3 hiSliderLimit( _dynamics_world->crossbar_width, 1., 1. ) ;
        _constraint->setLinearLowerLimit( lowerSliderLimit );
        _constraint->setLinearUpperLimit( hiSliderLimit );
        _constraint->setOverrideNumSolverIterations( 20 );
        _constraint->setDbgDrawSize( btScalar( 5.f ) );
    }

    // Copy the path for bashing later
    _ap = new osg::AnimationPath;
    _ap->setLoopMode( osg::AnimationPath::LOOP );
    osg::AnimationPath::TimeControlPointMap& tcpm = _orig_ap->getTimeControlPointMap();
    osg::AnimationPath::TimeControlPointMap::iterator it = tcpm.begin();
    double last_key = ( *it ).first;
    _ap->insert( ( *it ).first, ( *it ).second );
    it++;
    for ( ; it != tcpm.end(); it++ )
    {
        _ap->insert( ( *it ).first, ( *it ).second );
        _minimum_distance = std::min( ( *it ).first - last_key, _minimum_distance );
        last_key = ( *it ).first;
    }
}

void
CameraPathUpdateCallback::clear()
{
    if ( _dynamics_world->has_crossbar )
    {
        for ( int i = _dynamics_world->world->getNumConstraints() - 1; i >= 0; i-- )
            _dynamics_world->world->removeConstraint( _dynamics_world->world->getConstraint( i ) );
    }
}

void
CameraPathUpdateCallback::update_crossbar()
{
    if ( _dynamics_world->has_crossbar )
    {
        btVector3 lowerSliderLimit( -_dynamics_world->crossbar_width, -1., -1. );
        btVector3 hiSliderLimit( _dynamics_world->crossbar_width, 1., 1. );
        _constraint->setLinearLowerLimit( lowerSliderLimit );
        _constraint->setLinearUpperLimit( hiSliderLimit );
    }
}

float
CameraPathUpdateCallback::getClosetPoint( osg::Vec3d const& A, osg::Vec3d const& B,
                                          osg::Vec3f& P, bool segmentClamp ) const
{
    osg::Vec3f AP = P - A;
    osg::Vec3f AB = B - A;
    float ab2 = AB * AB;
    float ap_ab = AP * AB;
    float t = ap_ab / ab2;
    if ( segmentClamp )
    {
        if ( t < 0.0f ) t = 0.0f;
        else if ( t > 1.0f ) t = 1.0f;
    }
    return t;
}

void
CameraPathUpdateCallback::getInterpolatedControlPoint( double& time,
                                                       osg::Vec3f pos,
                                                       osg::AnimationPath::ControlPoint& cp ) const
{
    double modulated_time = ( time - _ap->getFirstTime() ) / _ap->getPeriod();
    double fraction_part = modulated_time - floor( modulated_time );
    time = _ap->getFirstTime() + fraction_part * _ap->getPeriod();
    //  std::cout << time << ", " << pos << std::endl;

    osg::AnimationPath::TimeControlPointMap::const_iterator first, mid_low, mid_high, third;
    mid_low = _ap->getTimeControlPointMap().lower_bound( time );
    mid_high = mid_low;

    if ( mid_low == _ap->getTimeControlPointMap().begin() )
    {
        mid_low = _ap->getTimeControlPointMap().end();
        mid_low--; // Same control point as begin
        first = mid_low;
        first--;
    }
    else
    {
        first = mid_low;
        --first;
    }

    third = mid_high;
    ++third;
    if ( third == _ap->getTimeControlPointMap().end() )
    {
        mid_high = _ap->getTimeControlPointMap().begin();
        third = mid_high;
        ++third;
    }

    float t = getClosetPoint( first->second.getPosition(), mid_low->second.getPosition(), pos );
    float last_t = getClosetPoint( mid_high->second.getPosition(), third->second.getPosition(), pos );

    // Loop while t or last_t is outside of [0, 1]. Exit if in range, or we're bouncing
    // around one of the endpoints, e.g. t = 1.007..., last_t = -0.02...
    while ( !( ( t >= 0.f && t <= 1.f ) || ( last_t >= 0.f && last_t <= 1.f ) ) &&
            !( last_t* t < 0 ) )
    {
        //    std::cout << t << ", " << last_t << std::endl;
        //    std::cout << first->first << ", " << mid_low->first << ", " << mid_high->first << ", " << third->first << std::endl;

        if ( t < 0 )
        {
            if ( first == _ap->getTimeControlPointMap().begin() )
            {
                mid_low = _ap->getTimeControlPointMap().end();
                mid_low--;
                first = mid_low;
                first--;
            }
            else
            {
                mid_low = first;
                first--;
            }

            if ( mid_high == _ap->getTimeControlPointMap().begin() )
            {
                third = _ap->getTimeControlPointMap().end();
                third--;
                mid_high = third;
                mid_high--;
            }
            else
            {
                third = mid_high;
                mid_high--;
            }

            last_t = t;
            t = getClosetPoint( first->second.getPosition(), mid_low->second.getPosition(), pos );
        }
        else
        {
            mid_high = third;
            third++;
            if ( third == _ap->getTimeControlPointMap().end() )
            {
                mid_high = _ap->getTimeControlPointMap().begin();
                third = mid_high;
                ++third;
            }

            mid_low++;
            if ( mid_low == _ap->getTimeControlPointMap().end() )
            {
                first = _ap->getTimeControlPointMap().begin();
                mid_low = first;
                ++mid_low;
            }
            else
            {
                first++;
            }

            t = last_t;
            last_t = getClosetPoint( mid_high->second.getPosition(), third->second.getPosition(), pos );
        }
    }

    osg::Vec3f seg, ep1, ep2, mp;
    double t1, t2, tt, d1, d2, d_mp;

    // Check first segment
    ep1 = first->second.getPosition();
    d1 = ( pos - ep1 ).length();
    t1 = 0.f;
    ep2 = mid_low->second.getPosition();
    d2 = ( pos - ep2 ).length();
    t2 = 1.f;
    seg = ep2 - ep1;
    tt = 0.5;
    mp = ep1 + seg * tt;
    d_mp = ( pos - mp ).length();

    //  std::cout << tt << ", " << d1 << ", " << d2 << ", " << d_mp << ", <" << ep1 << ">, <" << ep2 << ">, <" << mp << ">" << std::endl;
    if ( !boost::math::isfinite( d_mp ) )
        std::cout << "Degenerate midpoint found during interpolation" << std::endl;

    while ( fabs( d2 - d1 ) > 1.e-5 && fabs( d_mp - d1 ) > 1.e-5 )
    {
        if ( d2 < d1 )
        {
            t1 = tt;
            d1 = d_mp;
            tt = t1 + ( t2 - t1 ) / 2.f;
            mp = ep1 + seg * tt;
        }
        else
        {
            t2 = tt;
            d2 = d_mp;
            tt = t1 + ( t2 - t1 ) / 2.f;
            mp = ep1 + seg * tt;
        }

        d_mp = ( pos - mp ).length();
        //    std::cout << tt << ", " << d1 << ", " << d2 << ", " << d_mp << ", <" << ep1 << ">, <" << ep2 << ">, <" << mp << ">" << std::endl;
    }

    double d_min = d_mp;
    osg::Vec3f p_min = mp;
    double t_min = tt;
    osg::AnimationPath::TimeControlPointMap::const_iterator first_min = first, second_min = mid_low;

    if ( tt > 0.9 || tt < 0.1 )
    {
        // Check second segment
        ep1 = mid_high->second.getPosition();
        d1 = ( pos - ep1 ).length();
        t1 = 0.f;
        ep2 = third->second.getPosition();
        d2 = ( pos - ep2 ).length();
        t2 = 1.f;
        seg = ep2 - ep1;
        tt = 0.5;
        mp = ep1 + seg * tt;
        d_mp = ( pos - mp ).length();

        while ( fabs( d2 - d1 ) > 1.e-5 && fabs( d_mp - d1 ) > 1.e-5 )
        {
            if ( d2 < d1 )
            {
                t1 = tt;
                d1 = d_mp;
                tt = t1 + ( t2 - t1 ) / 2.f;
                mp = ep1 + seg * tt;
            }
            else
            {
                t2 = tt;
                d2 = d_mp;
                tt = t1 + ( t2 - t1 ) / 2.f;
                mp = ep1 + seg * tt;
            }

            d_mp = ( pos - mp ).length();
            //      std::cout << "- " << tt << ", " << d1 << ", " << d2 << ", " << d_mp << ", <" << ep1 << ">, <" << ep2 << ">, <" << mp << ">" << std::endl;
        }

        if ( d_mp < d_min )
        {
            t_min = tt;
            first_min = mid_high;
            second_min = third;
        }
    }

    double delta_time = second_min->first - first_min->first;
    time = first_min->first + t_min * delta_time;
    //  std::cout << time << ", " << first_min->first << ", " << second_min->first << ", " << ( time - first_min->first ) / delta_time << ", " << t_min << std:: endl;

    cp.interpolate( t_min,
                    first_min->second,
                    second_min->second );
}

void
CameraPathUpdateCallback::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
    // must traverse the Node's subgraph
    traverse( node, nv );

    if ( _initializing )
    {
        const osg::FrameStamp* frameStamp = nv->getFrameStamp();
        if ( frameStamp )
        {
            _initializing = false;
            // Need to extract a reasonable z value first
            _ap->getInterpolatedControlPoint( 0.1, _cp_now );
            btVector3 currentPosition = _body->getWorldTransform().getOrigin();
            osg::Vec3f pos( currentPosition[0],
                            currentPosition[1],
                            _cp_now.getPosition().z() );

            _firstTime = _start_time;
            getInterpolatedControlPoint( _firstTime, pos, _cp_now );
            _pt_now = _cp_now.getPosition();
            _firstTime = _start_time - 0.1;
            getInterpolatedControlPoint( _firstTime, pos, _cp_last );
            _pt_last = _cp_last.getPosition();

            osg::Vec3d tangent = _pt_now - _pt_last;
            if ( tangent * _start_dir < 0 )
            {
                // Swap now and last as we're pointing in the wrong direction
                tangent = _pt_now;
                _pt_now = _pt_last;
                _pt_last = tangent;
                tangent = _pt_now - _pt_last;
            }
            tangent.normalize();
            _tangent = tangent;

            osg::Vec3d axis;
            _cp_last.getRotation().getRotate( _last_unwrapped_angle, axis );
            _last_ah_angle = _last_unwrapped_angle;
            btVector3 a( axis.x(), axis.y(), axis.z() );
            btVector3 p( _pt_now.x(), _pt_now.y(), _pt_now.z() );
            btQuaternion q( a, _last_unwrapped_angle );
            _sliding_body->getWorldTransform().setOrigin( p );
            _sliding_body->getWorldTransform().setRotation( q );

            if ( _dynamics_world->has_crossbar )
                _dynamics_world->world->addConstraint( _constraint, true );
        }
    }
    else
    {
        Motion_Data* md = ( Motion_Data* )_tracker->getUserData();
        update( node );
        md->motion.set( 0.f, 0.f, 0.f );
    }
}

void
CameraPathUpdateCallback::update( osg::Node* node )
{
    if ( !_traversed )
    {
        _traversed = true;
        Motion_Data* md = ( Motion_Data* )_tracker->getUserData();
        if ( ! _first && md->motion.length() > _minimum_velocity_thresold )
        {
            _first = true;
            _startPos = _body->getWorldTransform();
        }

        osg::MatrixTransform* pat = dynamic_cast< osg::MatrixTransform* >( node );
        //    std::cout << "  " << "_tangent - <" << _tangent << ">" << std::endl;

        osg::Vec3d vec = md->motion;
        osg::Vec3d up, forward;
        osg::Vec3d x_axis( 1, 0, 0 );
        int sign;

        // Find angle to transform ball coordinate system to front
        osg::Matrixd ball_to_camera = compute_ball_to_camera_matrix( up, forward, sign );
        osg::Vec3d v = ball_to_camera * vec;
        osg::Vec3d x_axis_ws = ball_to_camera * x_axis; // X in world/camera space
        v.normalize();
        //    std::cout << "  " << "v - <" << v << ">" << std::endl;

        double cos_theta = _tangent * v;
        _timeIncrement = sign * cos_theta * vec.length();
        //std::cout << "  " << sign << " " << cos_theta << " " << _timeIncrement << " " << _firstTime << std::endl;

        // Use the puck to find the actual camera position
        btVector3 currentPosition = _body->getWorldTransform().getOrigin();
        osg::Vec3f pos( currentPosition[0],
                        currentPosition[1],
                        _cp_now.getPosition().z() );

        //    std::cout << "  <" << currentPosition << ">" << std::endl;

        getInterpolatedControlPoint( _firstTime, pos, _cp_now );
        _pt_now = _cp_now.getPosition();

        osg::Vec3d axis, axis_pred, v_now, v_next;
        double ah_angle, ah_angle_pred;
        _cp_now.getRotation().getRotate( ah_angle, axis );
        v_now = _pt_now - _pt_last;
        v_now.normalize();
        //    std::cout << "  " << ah_angle << ", <" << axis << ">" << std::endl;

        // Update the position of the sliding body.
        btVector3 a( axis.x(), axis.y(), axis.z() );
        btVector3 p( _pt_now.x(), _pt_now.y(), _pt_now.z() );
        btQuaternion q( a, ah_angle );
        _sliding_body->getWorldTransform().setOrigin( p );
        _sliding_body->getWorldTransform().setRotation( q );

        // Use the minimum distance to predict where we will wind up next
        osg::AnimationPath::ControlPoint cp_next;
        _ap->getInterpolatedControlPoint( _firstTime + _timeIncrement + sign * _minimum_distance, cp_next );
        osg::Vec3d pt_next = cp_next.getPosition();

        cp_next.getRotation().getRotate( ah_angle_pred, axis_pred );
        v_next = pt_next - _pt_now;
        v_next.normalize();
        double aa = acos( v_now * v_next );

        // if the dot product of the vectors between last, now, and next is greater than 90
        // _timeIncrement has the wrong sign
        if ( std::abs( aa ) > boost::math::constants::half_pi< float >() )
        {
            // flip sign
            sign *= -1;
            _timeIncrement *= -1;
            _ap->getInterpolatedControlPoint( _firstTime + _timeIncrement + sign * _minimum_distance, cp_next );
            pt_next = cp_next.getPosition();

            cp_next.getRotation().getRotate( ah_angle_pred, axis_pred );
            v_next = pt_next - _pt_now;
            v_next.normalize();
            aa = acos( v_now * v_next );
        }
        //std::cout << "  " << v_now << ", " << v_next << std::endl;
        //std::cout << "  " << aa << ", " << ( v_now ^ v_next ) << std::endl;
        _firstTime += _timeIncrement;

        double blended_angle = _last_unwrapped_angle;
        {
            //std::cout << "  " << "pt_next - <" << pt_next << ">" << std::endl;
            //std::cout << "  " << "_pt_now - <" << _pt_now << ">" << std::endl;
            //std::cout << "  " << "_pt_last - <" << _pt_last << ">" << std::endl;
            _tangent = _pt_now - _pt_last;
            _tangent.normalize();

            {
                osg::Vec3d vv, tv;
                double d_angle = 0.0;
                double ratio = _plot.x_vals().back();
                double blended_dangle = 0.0;
                osg::Vec3d ahv, ttv;
                //        std::cout << "  " << _turning_mixture << ", " << blended_angle  << ", " << ah_angle << std::endl;

                //std::cout << "  " << ah_angle  << ", " << ah_angle_pred << std::endl;
                ah_angle = unwrap( _last_ah_angle, ah_angle );
                ah_angle_pred = unwrap( _last_ah_angle, ah_angle_pred );
                double ah_angular_rate = ( ah_angle_pred - _last_unwrapped_angle ) * ( 1.f - _turning_mixture );
                _last_ah_angle = ah_angle;

                //std::cout << "  " << ah_angle  << ", " << ah_angle_pred << ", " << ah_angular_rate << std::endl;

                if ( md->motion.length() > _minimum_velocity_thresold )
                {
                    if ( _turning_mixture > 0.f )
                    {
                        if ( md->roll == 0.f )
                            ratio = _plot.x_vals().back();
                        else
                            // Use -pitch as forward is negative pitch
                            ratio = -md->pitch / md->roll;

                        _ratio_values[0] = ratio;
                    }

                    if ( _turning_mixture < 1.f )
                        ahv = computeTurningVector( vec, ball_to_camera, up, md->yaw );

                    //          std::cout << "  *  " << d_angle << ", <" << ttv << ">" << ", <" << ahv << ">" << std::endl;
                    _euler_vector = osg::Vec3d( md->pitch, md->roll, md->yaw );

                    if ( _use_input_heading_smoothing )
                    {
                        if ( _input_ratio.full() )
                        {
                            // Remove first element from the accumulated value
                            _input_smoothing_val -= _input_ratio.front();
                            _input_ratio.push_back( _euler_vector );
                            _input_smoothing_val += _input_ratio.back();
                        }
                        else
                        {
                            // Filling the buffer
                            _input_ratio.push_back( _euler_vector );
                            _input_smoothing_val += _input_ratio.back();
                        }

                        _euler_vector = _input_smoothing_val / _input_ratio.size();
                    }
                    else
                    {
                        _input_ratio.push_back( _euler_vector );
                    }

                    if ( _turning_mixture > 0.f )
                    {
                        if ( _euler_vector.y() == 0.f )
                            ratio = _plot.x_vals().back();
                        else
                            // Use -pitch as forward is negative pitch
                            ratio = -_euler_vector.x() / _euler_vector.y();

                        _ratio_values[2] = ratio;
                        ttv = computeThresholdVector( ratio, vec, ball_to_camera, up, md, &d_angle );
                        // Scale threshold vector by cos( theta ), so that translation is
                        // minimized with a stronger roll component.
                        osg::Vec3d v = vec;
                        v.normalize();
                        double cos_theta = v * x_axis;
                        ttv *= fabs( cos_theta );
                    }

                    smooth_velocities( ttv, ahv );

                    if ( _use_intermediary_heading_smoothing )
                    {
                        if ( _intermediary_angle.full() )
                        {
                            // Remove first element from the accumulated value
                            _intermediary_smoothing_val -= _intermediary_angle.front();
                            _intermediary_angle.push_back( d_angle );
                            _intermediary_smoothing_val += _intermediary_angle.back();
                        }
                        else
                        {
                            // Filling the buffer
                            _intermediary_angle.push_back( d_angle );
                            _intermediary_smoothing_val += _intermediary_angle.back();
                        }

                        d_angle = _intermediary_smoothing_val / _intermediary_angle.size();

                    }
                    else
                    {
                        _intermediary_angle.push_back( d_angle );
                    }

                    blended_dangle = d_angle * _turning_mixture + ah_angular_rate * ( 1.f - _turning_mixture );

                    //          std::cout << " **  " << blended_dangle  << ", " << blended_angle << std::endl;

                    if ( _use_output_heading_smoothing )
                    {
                        if ( _output_angle.full() )
                        {
                            // Remove first element from the accumulated value
                            _output_smoothing_val -= _output_angle.front();
                            _output_angle.push_back( blended_dangle );
                            _output_smoothing_val += _output_angle.back();
                        }
                        else
                        {
                            // Filling the buffer
                            _output_angle.push_back( blended_dangle );
                            _output_smoothing_val += _output_angle.back();
                        }

                        // Smooth the post-blend rate by using a weighted average of
                        // the calculated rate and the smoothed value. We do this
                        // because auto-heading is computing an absolute delta to pull
                        // us back to the path, and we don't really want to smooth it
                        // as it will lead to oscillations
                        blended_dangle = _output_smoothing_val / _output_angle.size() * _turning_mixture
                                         + blended_dangle * ( 1.f - _turning_mixture );
                    }
                    else
                    {
                        _output_angle.push_back( blended_dangle );
                    }

                    blended_angle += blended_dangle;
                }
                else
                    blended_angle = _last_unwrapped_angle;

                //        std::cout << " *** " << blended_dangle  << ", " << blended_angle << ", " << _output_smoothing_val << std::endl;

                tv = ttv * _turning_mixture + ahv * ( 1.f - _turning_mixture );
                //        std::cout << "  " << "<" << ahv << ">, <" << tv << ">" << std::endl;
                _ratio_values[3] = tv.length() / _frame_rate;

                if ( _restrict_vertical_motion )
                    tv.z() = -10.f;
                else
                    tv.z() = _body->getLinearVelocity().z();

                //        std::cout << "  " << v << std::endl;

                if ( !_use_dynamics )
                {
                    btTransform start, end;

                    btVector3 step( tv.x(), tv.y(), 0.f );
                    btVector3 targetPosition = currentPosition + step;

                    //          std::cout << "  " << currentPosition << ", " << targetPosition << std::endl;

                    start.setIdentity();
                    end.setIdentity();

                    start.setOrigin( currentPosition );
                    end.setOrigin( targetPosition );

                    btCollisionWorld::ClosestConvexResultCallback callback( currentPosition, targetPosition );
                    callback.m_collisionFilterGroup = COL_CAMERA;

                    _dynamics_world->world->convexSweepTest ( ( btConvexShape* )( _body->getCollisionShape() ), start, end, callback, _dynamics_world->world->getDispatchInfo().m_allowedCcdPenetration );

                    if ( callback.hasHit() )
                    {
                        btVector3 punch( tv.x() / _frame_rate, tv.y() / _frame_rate, tv.z() );
                        _body->setAngularVelocity( btVector3( 0.f, 0.f, 0.f ) );
                        _body->setLinearVelocity( punch );
                    }
                    else
                    {
                        _body->getWorldTransform().setOrigin( targetPosition );
                        btVector3 punch( 0.f, 0.f, -10 );
                        // Need these to dampen motion when transitioning from dynamics back to kinematics
                        _body->setAngularVelocity( btVector3( 0.f, 0.f, 0.f ) );
                        _body->setLinearVelocity( punch );
                    }
                }
                else
                {
                    btVector3 punch( tv.x() / _frame_rate, tv.y() / _frame_rate, tv.z() );
                    _body->setAngularVelocity( btVector3( 0.f, 0.f, 0.f ) );
                    _body->setLinearVelocity( punch );
                }
            }
        }

        {
            // Block for updating tracker
            const osg::Quat& rot = _tracker->getRotation();
            osg::Matrix mat;
            mat.makeRotate( -WrapTwoPi( blended_angle - _last_unwrapped_angle ), _y_axis );
            osg::Quat r;
            r.set( mat );
            _tracker->setRotation( r * rot );
            ( r * rot ).get( mat );
            _last_unwrapped_angle = blended_angle;
        }

        _pt_last = _pt_now;
        _cp_last = _cp_now;
    }
}

CameraPathUpdateCallback::~CameraPathUpdateCallback()
{
    _ap.release();
    //_tracker.release();
}

void
CameraPathUpdateCallback::useRotations( bool on_or_off )
{
    _ap.release();

    _ap = new osg::AnimationPath;
    _ap->setLoopMode( osg::AnimationPath::LOOP );
    osg::AnimationPath::TimeControlPointMap& tcpm = _orig_ap->getTimeControlPointMap();
    osg::AnimationPath::TimeControlPointMap::iterator it;
    for ( it = tcpm.begin(); it != tcpm.end(); it++ )
        if ( on_or_off )
            _ap->insert( ( *it ).first, ( *it ).second );
        else
            // Yank out the position, ignoring the rotation factor
            _ap->insert( ( *it ).first,
                         osg::AnimationPath::ControlPoint( ( *it ).second.getPosition() ) );
}

