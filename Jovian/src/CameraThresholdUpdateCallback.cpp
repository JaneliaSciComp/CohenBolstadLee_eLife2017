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
#include "CameraThresholdUpdateCallback.h"

CameraThresholdUpdateCallback::CameraThresholdUpdateCallback( btRigidBody* body,
                                                              Collision_World* dynamics_world,
                                                              osg::ref_ptr< osgGA::NodeTrackerManipulator > tracker,
                                                              ContactNodes const& nodes,
                                                              osg::Vec3f start_dir ):
    CameraMotionUpdateCallback( body, dynamics_world, tracker, nodes ), _start_dir( start_dir )
{
    osg::Vec3d translation, scale;
    osg::Quat rotation, so;

    _tracker->getMatrix().decompose( translation, rotation, scale, so );
    osg::Matrixd m, m_inv;
    rotation.get( m );
    m_inv.invert( m );

    osg::Vec3d eye, center, up, front;
    m_inv.getLookAt( eye, center, up );

    osg::Vec3d cross;
    osg::Vec3d x_axis( 1, 0, 0 );
    cross = start_dir ^ x_axis;
    int sign = ( up * cross ) < 0 ? -1 : 1;

    _theta = sign * acosf( start_dir * x_axis );
}

osg::Vec3d
CameraThresholdUpdateCallback::computeThresholdVector( float ratio, osg::Vec3d const& vec, osg::Matrixd const& ball_to_camera,
                                                       osg::Vec3d const& up, Motion_Data const* md, double* d_angle )
{
    float dps = 0.f;
    float velocity;

    // Velocity plot is given in cm/sec, so scale by frame rate to convert to the
    // appropriate units
    velocity = vec.length() / _frame_rate;
    float gain = _v_plot.evaluate( velocity );
    dps = gain * _plot.evaluate( ratio );

    dps *= ( ratio > 0 ) ? 1.f : -1.f;
    dps *= ( md->pitch > 0 ) ? -1.f : 1.f;

    osg::Matrixd mat, m;
    *d_angle = dps * _frame_rate;
    mat.makeRotate( *d_angle, up );
    m = ball_to_camera * mat;

    _ratio_values[1] = velocity;

    return m * vec;
}

osg::Vec3d
CameraThresholdUpdateCallback::computeTurningVector( osg::Vec3d const& vec, osg::Matrixd const& ball_to_camera, osg::Vec3d const& up, float yaw )
{
    osg::Matrixd mat, m;
    mat.makeRotate( yaw, up );
    m = ball_to_camera * mat;

    return m * vec;
}


osg::Matrixd
CameraThresholdUpdateCallback::compute_ball_to_camera_matrix( osg::Vec3d& up,
                                                              osg::Vec3d& forward,
                                                              int& sign )
{
    osg::Vec3d translation, scale;
    osg::Quat rotation, so;

    _tracker->getMatrix().decompose( translation, rotation, scale, so );
    osg::Matrixd m, m_inv;
    rotation.get( m );
    m_inv.invert( m );

    osg::Vec3d eye, center;
    m_inv.getLookAt( eye, center, up );
    up.x() = std::abs( up.x() ) < 0.00001 ? 0 : up.x();
    up.y() = std::abs( up.y() ) < 0.00001 ? 0 : up.y();
    up.z() = std::abs( up.z() ) < 0.00001 ? 0 : up.z();
    forward = center - eye;
    if ( std::abs( forward.z() ) < 0.00001 )
        forward.z() = 0.;

    forward.normalize();

    // Find angle to transform ball coordinate system to front
    osg::Vec3d x_axis( 1, 0, 0 );
    osg::Matrixd ball_to_camera;
    osg::Vec3d cross = x_axis ^ forward;
    cross.normalize();
    sign = ( up * cross ) > 0 ? -1 : 1;
    //std::cout << "  " << "forward - <" << forward << ">" << std::endl;
    //std::cout << "  " << "cross - <" << cross << ">" << std::endl;
    //std::cout << "  " << "up - <" << up << ">" << std::endl;
    double cos_theta = x_axis * forward;
    //std::cout << "  " << "cos_theta - <" << cos_theta << ">" << std::endl;
    ball_to_camera.makeRotate( sign * acosf( cos_theta ), up );

    return ball_to_camera;
}

void
CameraThresholdUpdateCallback::smooth_velocities( osg::Vec3d& ttv, osg::Vec3d& ahv )
{
    if ( _use_vel_smoothing )
    {
        if ( _tt_vel_frames.full() )
        {
            // Remove first element from the accumulated value
            _smoothing_tt_vel -= _tt_vel_frames.front();
            _tt_vel_frames.push_back( ttv );
            _smoothing_tt_vel += _tt_vel_frames.back();
        }
        else
        {
            // Filling the buffer
            _tt_vel_frames.push_back( ttv );
            _smoothing_tt_vel += _tt_vel_frames.back();
        }

        ttv = _smoothing_tt_vel / _tt_vel_frames.size();

        if ( _vel_frames.full() )
        {
            // Remove first element from the accumulated value
            _smoothing_vel -= _vel_frames.front();
            _vel_frames.push_back( ahv );
            _smoothing_vel += _vel_frames.back();
        }
        else
        {
            // Filling the buffer
            _vel_frames.push_back( ahv );
            _smoothing_vel += _vel_frames.back();
        }

        ahv = _smoothing_vel / _vel_frames.size();
    }
    else
    {
        _tt_vel_frames.push_back( ttv );
        _vel_frames.push_back( ahv );
    }

}

