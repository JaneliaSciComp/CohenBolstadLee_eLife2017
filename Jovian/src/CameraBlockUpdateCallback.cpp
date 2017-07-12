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

#include "CameraBlockUpdateCallback.h"

CameraBlockUpdateCallback::CameraBlockUpdateCallback( btRigidBody* body,
        Collision_World* dynamics_world,
        osg::ref_ptr< osgGA::NodeTrackerManipulator >tracker,
        ContactNodes const& nodes,
        osg::Vec3f& start_dir ):
    CameraThresholdUpdateCallback( body, dynamics_world, tracker, nodes, start_dir )
{}

void
CameraBlockUpdateCallback::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
    traverse( node, nv );
    if ( _initializing )
    {
        const osg::FrameStamp* frameStamp = nv->getFrameStamp();
        if ( frameStamp )
        {
            _initializing = false;
            _last_unwrapped_angle = _theta;
            _last_ah_angle = _last_unwrapped_angle;
        }
    }
    else
    {
        Motion_Data* md = ( Motion_Data* )_tracker->getUserData();
        if ( _enable_turning )
            update_threshold_turning( node );
        else
            update( node );
        md->motion.set( 0.f, 0.f, 0.f );
    }
}


void
CameraBlockUpdateCallback::update( osg::Node* node )
{
    if ( !_traversed )
    {
        _traversed = true;
        Motion_Data* md = ( Motion_Data* )_tracker->getUserData();
        if ( md->motion.length() > 1.e-6  && ! _first )
        {
            _first = true;
            _startPos = _body->getWorldTransform();
        }

        {
            osg::Vec3d m = md->motion;
            float angle = _theta + md->yaw;
            osg::Vec3d v;
            _vel_frames.push_back( osg::Vec3d( m.x() * cosf( angle ) - m.y() * sinf( angle ),
                                               m.y() * cosf( angle ) + m.x() * sinf( angle ),
                                               _body->getLinearVelocity().z() ) );

            if ( _use_vel_smoothing )
            {
                if ( _vel_frames.full() )
                {
                    // Remove first element from the accumulated value
                    _smoothing_vel -= _vel_frames.front();
                    _smoothing_vel += _vel_frames.back();
                }
                else
                {
                    // Filling the buffer
                    _smoothing_vel += _vel_frames.back();
                }

                v = _smoothing_vel / _vel_frames.size();
            }
            else
                v = _vel_frames.back();

            if ( _restrict_vertical_motion )
                v.z() = -10.f;

            //      std::cout << v << std::endl;

            if ( !_use_dynamics )
            {
                btTransform start, end;

                btVector3 currentPosition = _body->getWorldTransform().getOrigin();
                btVector3 step( v.x(), v.y(), 0.f );
                btVector3 targetPosition = currentPosition + step;

                //      std::cout << currentPosition << ", " << targetPosition << ", " << targetPosition - _startPos << std::endl;

                start.setIdentity();
                end.setIdentity();

                start.setOrigin( currentPosition );
                end.setOrigin( targetPosition );

                btCollisionWorld::ClosestConvexResultCallback callback( currentPosition, targetPosition );
                callback.m_collisionFilterGroup = COL_CAMERA;

                _dynamics_world->world->convexSweepTest ( ( btConvexShape* )( _body->getCollisionShape() ), start, end, callback, _dynamics_world->world->getDispatchInfo().m_allowedCcdPenetration );

                if ( callback.hasHit() )
                {
                    btVector3 punch( v.x() / _frame_rate, v.y() / _frame_rate, v.z() );
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
                btVector3 punch( v.x() / _frame_rate, v.y() / _frame_rate, v.z() );
                _body->setAngularVelocity( btVector3( 0.f, 0.f, 0.f ) );
                _body->setLinearVelocity( punch );
            }
        }
    }
}

void
CameraBlockUpdateCallback::update_threshold_turning( osg::Node* node )
{
    if ( !_traversed )
    {
        _traversed = true;
        Motion_Data* md = ( Motion_Data* )_tracker->getUserData();
        if ( md->motion.length() > 1.e-6  && ! _first )
        {
            _first = true;
            _startPos = _body->getWorldTransform();
        }

        {
            float c1, c2, c3, s1, s2, s3;
            c1 = cosf( md->roll / 2.f );
            s1 = sinf( md->roll / 2.f );
            c2 = cosf( md->pitch / 2.f );
            s2 = sinf( md->pitch / 2.f );
            c3 = cosf( md->yaw / 2.f );
            s3 = sinf( md->yaw / 2.f );

            osg::Quat q( s1 * c2 * c3 - c1 * s2 * s3,
                         c1 * s2 * c3 + s1 * c2 * s3,
                         c1 * c2 * s3 - s1 * s2 * c3,
                         c1 * c2 * c3 + s1 * s2 * s3 );
            osg::Vec3f z_axis = osg::Vec3f( 0, 0, 1 );
            osg::Vec3f qv, v1 = q * z_axis;
            v1.z() = 0.f;
            v1.normalize();
            double ang;
            osg::Vec3f axis;
            q.getRotate( ang, axis );
            qv = v1 * ang * md->radius;

            osg::Vec3d vec( -qv.x(), qv.y(), qv.z() );
            _theta += md->yaw;

            osg::Vec3d up, forward;
            osg::Vec3d x_axis( 1, 0, 0 );
            int sign;

            // Find angle to transform ball coordinate system to front
            osg::Matrixd ball_to_camera = compute_ball_to_camera_matrix( up, forward, sign );
            osg::Vec3d v = ball_to_camera * vec;
            osg::Vec3d x_axis_ws = ball_to_camera * x_axis; // X in world/camera space
            v.normalize();

            double ah_angle = _theta;
            double blended_angle = _last_unwrapped_angle;
            {
                {
                    osg::Vec3d vv, tv;
                    double d_angle = 0.0;
                    double ratio = _plot.x_vals().back();
                    double blended_dangle = 0.0;
                    osg::Vec3d ahv, ttv;
                    //        std::cout << "  " << _turning_mixture << ", " << blended_angle  << ", " << ah_angle << std::endl;

                    ah_angle = unwrap( _last_ah_angle, ah_angle );
                    double ah_angular_rate = ( ah_angle - _last_unwrapped_angle ) * ( 1.f - _turning_mixture );
                    _last_ah_angle = ah_angle;

                    //        std::cout << "  " << ah_angle  << ", " << ah_angle_pred << ", " << ah_angular_rate << std::endl;

                    if ( vec.length() > _minimum_velocity_thresold )
                    {
                        if ( _turning_mixture > 0.f )
                        {
                            if ( md->roll == 0.f )
                                ratio = _plot.x_vals().back();
                            else
                                // Forward is negative pitch
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
                        blended_angle += ( md->yaw * ( 1.f - _turning_mixture ) );

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
                        btVector3 currentPosition = _body->getWorldTransform().getOrigin();
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
                mat.makeRotate( WrapTwoPi( blended_angle - _last_unwrapped_angle ), _y_axis );
                osg::Quat r;
                r.set( mat );
                _tracker->setRotation( r * rot );
                ( r * rot ).get( mat );
                _last_unwrapped_angle = blended_angle;
            }
        }
    }
}

