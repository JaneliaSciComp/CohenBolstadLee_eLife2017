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


#ifndef CAMERABLOCKUPDATECALLBACK_H
#define CAMERABLOCKUPDATECALLBACK_H

#include <btBulletDynamicsCommon.h>

#include "CameraThresholdUpdateCallback.h"

class CameraBlockUpdateCallback : public CameraThresholdUpdateCallback
{
  public:
    CameraBlockUpdateCallback( btRigidBody* body,
                               Collision_World* dynamics_world,
                               osg::ref_ptr< osgGA::NodeTrackerManipulator > tracker,
                               ContactNodes const& nodes,
                               osg::Vec3f& start_dir );

    void operator()( osg::Node* node, osg::NodeVisitor* nv );
    void update( osg::Node* node );
    void update_threshold_turning( osg::Node* node );
};

#endif
