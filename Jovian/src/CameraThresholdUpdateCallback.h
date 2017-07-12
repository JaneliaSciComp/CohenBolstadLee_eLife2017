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

#ifndef CAMERATHRESHOLDUPDATECALLBACK_H
#define CAMERATHRESHOLDUPDATECALLBACK_H

#include <set>
#include <utility>
#include <vector>

#include <osg/MatrixTransform>
#include <osgGA/NodeTrackerManipulator>

#include <btBulletDynamicsCommon.h>

#include "MotionCallback.h"

class CameraThresholdUpdateCallback : public CameraMotionUpdateCallback
{
  public:
    CameraThresholdUpdateCallback ( btRigidBody* body,
                                    Collision_World* dynamics_world,
                                    osg::ref_ptr< osgGA::NodeTrackerManipulator > tracker,
                                    ContactNodes const& nodes,
                                    osg::Vec3f start_dir );
    osg::Matrixd compute_ball_to_camera_matrix( osg::Vec3d& up, osg::Vec3d& forward, int& sign );
    void update_start_direction( osg::Vec3f start_dir ) { _start_dir = start_dir; }
  protected:
    osg::Vec3d computeThresholdVector( float ratio, osg::Vec3d const& vec, osg::Matrixd const& mr, osg::Vec3d const& up, Motion_Data const* md, double* d_angle );
    osg::Vec3d computeTurningVector( osg::Vec3d const& vec, osg::Matrixd const& mr, osg::Vec3d const& up, float yaw );
    void smooth_velocities( osg::Vec3d& ttv, osg::Vec3d& ahv );

    osg::Vec3f _start_dir;
    double _last_unwrapped_angle, _last_ah_angle;
};


#endif
