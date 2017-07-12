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

#ifndef HUD_H
#define HUD_H

#include <osg/Camera>
#include <osg/Geometry>
#include <osg/Switch>

osg::Camera* createHUD( osg::Switch* switch_node, int numCameras,
                        int left, int right, int top, int bottom,
                        osg::Vec2 lower_left, int size, int offset );

#endif
