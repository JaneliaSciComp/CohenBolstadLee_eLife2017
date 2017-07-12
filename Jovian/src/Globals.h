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

/** @file Globals.h
  *
  * This file contains a few types, macros, routines, and one global variable that
  * didn't make sense to include anywhere else
*/


#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include <sstream>

#ifndef Q_MOC_RUN
#include <boost/multi_array.hpp>
#include <boost/shared_ptr.hpp>
#endif

#include <osg/Vec4f>

#ifdef _WINDOWS
// Needed to pickup M_PI and M_PI_2 on Windows because it's too stupid
// to actually let you use them when you include where they're defined.
#define _USE_MATH_DEFINES
#endif
#include <cmath>

#include <btBulletDynamicsCommon.h>

#include <osgbCollision/GLDebugDrawer.h>

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifdef _WINDOWS
inline float roundf(float x) { return floorf(x + 0.5f); }
#endif

extern int INVISIBLE_MASK;

typedef std::vector< std::vector< std::pair< float, float > > > GraphPositions;
class Console;
typedef boost::shared_ptr< Console > Console_Ptr;
class Scene_Model;
typedef boost::shared_ptr< Scene_Model > Scene_Model_Ptr;

/**
 * @brief Templated triple similar to the std::pair
 * @details A simple triple class designed in a similar vein to the standard pair class
 */
template <class T1, class T2, class T3> struct triple
{
	typedef T1 first_type;
	typedef T2 second_type;
	typedef T3 third_type;

	T1 first;
	T2 second;
	T3 third;
	triple() : first( T1() ), second( T2() ), third( T3() ) {}
	triple( const T1& x, const T2& y, const T3& z ): first( x ), second( y ), third( z ) {}
	template <class U, class V, class W>
	triple( const triple< U, V, W > &p ): first( p.first ), second( p.second ), third( p.third ) {}
};

/**
 * @brief Utility structure for physics related variables
 * @details This structure groups a number of physics related variables that are needed
 * for each scene. It's main function is that it contains two pointers to Bullet World objects.
 * We need two because the normal collision detection does not work if we are inside a
 * non-convex body.
 */
struct Collision_World
{
	btDiscreteDynamicsWorld* world;
	btDiscreteDynamicsWorld* concave;
	osgbCollision::GLDebugDrawer* debug_drawer;
	bool has_crossbar;
	float crossbar_width;

	Collision_World(): world( 0 ), concave( 0 ), debug_drawer( 0 ),
	                   has_crossbar( false ), crossbar_width( 0.f ) {}
};

/**
 * @brief Converts from a std::string to a type T
 * @details Converts from a std::string to a templated type T. Can convert anything an
 * istringstream knows about.
 *
 * @param t The value extracted from the string
 * @param s The string to be converted
 * @param f A descendant of ios_base, most typically std::dec
 * @return bool if the conversion succeeded.
 */
template <class T>
bool from_string(T& t,
                 const std::string& s,
                 std::ios_base& (*f)(std::ios_base&))
{
  std::istringstream iss(s);
  return !(iss >> f >> t).fail();
}

/**
 * @brief Holds information about a segment
 * @details Each segment of a segment object is stored separately
 */
typedef boost::multi_array< float, 2 > Segment_Grid;


/**
 * @brief Holds information about last collision event
 * @details During each frame anything that collides with the camera is added to this
 * structure. If the node is specially tagged as a gain node, we store it separately.
 */
struct ContactNodes
{
	int count, g_count;
	std::set< osg::Node* > nodes;
	std::set< osg::Node* > g_nodes;
	osg::Vec4f gain;

	void addNode( osg::Node* node )
	{
		if ( nodes.find( node ) == nodes.end() )
		{
			nodes.insert( node );

			count++;
		}
	}

	void addGainNode( osg::Node* node )
	{
		g_count++;
	}

	void reset()
	{
		count = 0;
		g_count = 0;
		gain = osg::Vec4f( 1.f, 1.f, 1.f, 1.f );
		nodes.clear();
		g_nodes.clear();
	}

	ContactNodes(): count( 0 ), g_count( 0 ), gain( 1.f, 1.f, 1.f, 1.f ) {}

};

/**
 * @brief Utility class for passing information about motion
 * @details Used as an interface between the RemoteDataServer (via CameraUpdateCallback)
 * and the CameraMotionUpdateCallback derived classes. Stores the computed motion vector
 * or the raw euler angles along with the raw values from the data server.
 */
class Motion_Data: public osg::Referenced
{
public:
	osg::Vec3d motion;
	osg::Vec4f raw;
	float velocity;
	float angle, roll, pitch, yaw, radius;
	float interval;

	Motion_Data(): Referenced(), motion( 0.f, 0.f, 0.f ), raw( 0.f, 0.f, 0.f, 0.f ),
								 velocity( 0.f ), angle( 0.f ),roll( 0.f ),
								 pitch( 0.f ), yaw( 0.f ), radius( 1.f ), interval( 1./60.f ) {}
};

#endif
