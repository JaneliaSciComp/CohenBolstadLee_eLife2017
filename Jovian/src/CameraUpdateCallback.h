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

#ifndef CAMERA_UPDATE_CALLBACK_H
#define CAMERA_UPDATE_CALLBACK_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#include <boost/lexical_cast.hpp>
#endif

#include <osg/Matrix>
#include <osg/NodeCallback>
#include <osgGA/CameraManipulator>
#include <osgGA/NodeTrackerManipulator>

#include <osg/io_utils>

#include <Globals.h>
#include <Communicator.h>
#include <MotionCallback.h>

/**
 * @brief [brief description]
 * @details Camera_Update_Callback is an abstract base class for all classes that
 * interface to a data server. The intent is for this classes descendants to
 * convert output from a data server into a form consumable by the rest of Jovian.
*/
class Camera_Update_Callback : public osg::NodeCallback
{
  public:
    /**
     * @brief Constructor
     * @details Constructor
     *
     * @param ma a generic osg CameraManipulator
     * @param nodes a reference to a
     * container that will hold the reportable objects that made contact with the
     * camera in a previous frame.
     */
    Camera_Update_Callback( osgGA::CameraManipulator* ma, ContactNodes const& nodes ):
        osg::NodeCallback(), _displacement( 0.f, 0.f, 0.f ), _offset( 0.f, 0.f, 0.f ),
        ntm( 0 ), _disable_motion( false ), _contact_nodes( nodes )
    {
        tm = ma;
    }

    /**
     * @brief Constructor
     * @details Constructor
     *
     * @param ma a generic osg NodeTrackerManipulator
     * @param nodes a reference to a
     * container that will hold the reportable objects that made contact with the
     * camera in a previous frame.
     */
    Camera_Update_Callback( osgGA::NodeTrackerManipulator* ma, ContactNodes const& nodes ):
        osg::NodeCallback(), _displacement( 0.f, 0.f, 0.f ), _offset( 0.f, 0.f, 0.f ),
        tm( 0 ), _disable_motion( false ), _contact_nodes( nodes )
    {
        ntm = ma;
    }

    /**
     * @brief Destructor
     * @details
     */
    virtual ~Camera_Update_Callback() {}

    /**
     * @brief Adjust scaling factors
     * @details Adjust the scale factors on each of three orthogonal axes.
     * Makes no assumption about what is being scaled (linear values
     * or euler angles).
     *
     * @param xs [description]
     * @param ys [description]
     * @param zs [description]
     */
    virtual void update_scale_factors( float xs, float ys, float zs ) = 0;

    /**
     * @brief The operation to be applied during scene graph traversal
     * @details The operation to be applied during scene graph traversal. Typically
     * should run during OSG's update traversal
     *
     * @param node Supplied through OSG. This should be the node the operator
     * was attached to.
     * @param nv Supplied by OSG
     */
    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv ) = 0;

    /**
     * @brief What is the status of motion? Are we enabled or disabled?
     *
     * @return bool
     */
    bool motion_state( ) const { return _disable_motion; }

    /**
     * @brief What is the current displacement?
     * @details `displacement'is the distance the animal has traveled on the
     * input device while motion is stopped.
     *
     * @return Vec3D - The distance traveled on the input device
     */
    osg::Vec3d const& displacement() const { return _displacement; }

    /**
     * @brief Clear the displacement vector
     */
    void clear() { _displacement.set( 0.f, 0.f, 0.f ); }

    /**
     * @brief Sets the vector for manual speed
     * @details When motion is disabled, this method can be used to provide
     * a manual velocity to the system. The use case is that the experimenter
     * can disable the input from the animal, and manually drive them around the
     * world.
     *
     * @param v - Vector - Speed to drive system when the animal is disconnected
     */
    void set_manual_speed_vector( osg::Vec3d const& v ) { _offset = v; }

    /**
     * @brief Turn motion on or off
     * @details If motion is off, then displacement is the distance the animal
     * has run on the ball, and _offset is the manual speed.
     *
     */
    void toggleMotion( )
    {
        _disable_motion = !_disable_motion;
        clear();
        if ( _disable_motion )
            _offset.set( 0.f, 0.f, 0.f );
        if ( ntm )
        {
            osg::Vec3d* vec = ( osg::Vec3d* )ntm->getUserData();
            vec->set( 0.f, 0.f, 0.f );
        }
    }

  protected:
    osgGA::CameraManipulator* tm;
    osgGA::NodeTrackerManipulator* ntm;
    /// displacement is the length of the motion vector since motion was disabled
    osg::Vec3d _displacement;
    /// offset is used in the case of researcher controlled movement
    osg::Vec3d _offset;
    bool _disable_motion;
    ContactNodes const& _contact_nodes;
};



/**
 * @brief Interface to the RemoteDataServer
 * @details Remote_Camera_Update_Callback is the Jovian interface to the RemoteDataServer
 * application. Its purpose is to translate the data provided by the
 * RemoteDataServer into a vector that is usable by Bullet.
 */
class Remote_Camera_Update_Callback : public Camera_Update_Callback
{
  public:
    /**
      * @brief Constructor
      * @details
      *
      * @param ma - Generic osg::CameraManipulator.
      * @param c - A Communicator. Reads the current values from the RemoteDataServer.
      * @param xs, ys, zs - Scale factors that convert the units from the RemoteDataServer into radians.
      * @param numSamples - The number of samples to use for smoothing the raw values from the RemoteDataServer.
      * @param nodes - The physics objects that contacted the camera in the last frame.
      * @param for_path - Is this being used for a world that contains a path object? We need to know as yaw is accumulated differently when we have a path object.
    */
    Remote_Camera_Update_Callback( osgGA::CameraManipulator* ma, Communicator* c,
                                   float xs, float ys, float zs, float rad,
                                   int numSamples, ContactNodes const& nodes,
                                   bool for_path_or_segment = false ):
        Camera_Update_Callback( ma, nodes ), comm( c ), _direction_buffer( numSamples ),
        _y_axis( 0, 1, 0 ), _z_axis( 0, 0, 1 ),
        _x_scale( xs ), _y_scale( ys ), _z_scale( zs ), _radius( rad ),
        _accumulated_vector( 0.f, 0.f, 0.f ), _Omega( 0.f ), _for_path_or_segment( for_path_or_segment )
    {
    }

    /** Constructor
      * @brief Constructor
      * @details
      *
      * @param ma - A NodeTrackerManipulator, a specific type of an osg::CameraManipulator. An NTM attaches the camera to a geometry object and follows it around the world.
      * @param c - A Communicator. Reads the current values from the RemoteDataServer.
      * @param xs, ys, zs - Scale factors that convert the units from the RemoteDataServer into radians.
      * @param numSamples - The number of samples to use for smoothing the raw values from the RemoteDataServer.
      * @param nodes - The physics objects that contacted the camera in the last frame.
      * @param for_path_or_segment - Is this being used for a world that contains a path object? We need to know as yaw is accumulated differently when we have a path object.
    */
    Remote_Camera_Update_Callback( osgGA::NodeTrackerManipulator* ma, Communicator* c,
                                   float xs, float ys, float zs, float rad,
                                   int numSamples, ContactNodes const& nodes,
                                   bool for_path_or_segment = false ):
        Camera_Update_Callback( ma, nodes ), comm( c ), _direction_buffer( numSamples ),
        _y_axis( 0, 1, 0 ), _z_axis( 0, 0, 1 ),
        _x_scale( xs ), _y_scale( ys ), _z_scale( zs ), _radius( rad ),
        _accumulated_vector( 0.f, 0.f, 0.f ), _Omega( 0.f ), _for_path_or_segment( for_path_or_segment )
    {
        ntm->setDistance( 0.04f );
    }

    /**
     * @brief Destructor
     * @details
     */
    virtual ~Remote_Camera_Update_Callback()
    {
        comm->close();
        //    std::cout << _Vfwd << ", " << _Vss << ", " << _Omega << std::endl;
    }

    /**
     * @brief Update the scale factors for RemoteDataServer data
     * @details The scale factors are used to scale the RemoteDataServer values
     * from camera units into ball (the input device)/world units. As it currently
     * operates, the RemoteDataServer is reading data from a ball, these scale
     * factors scale the raw input into degrees/radians
     *
     * @param xs number of camera units in pitch / 180 degrees
     * @param ys number of camera units in yaw / 180 degrees
     * @param zs number of camera units in roll / 180 degrees
     */
    virtual void update_scale_factors( float xs, float ys, float zs )
    {
        _x_scale = xs;
        _y_scale = ys;
        _z_scale = zs;
    }

    /**
     * @brief Add a vector for averaging
     * @details Adds a vector into a circular buffer to keep a running average
     * of the directions
     *
     * @param dir Vec3d - current heading
     */
    void average_direction( osg::Vec3d dir )
    {
        if ( _direction_buffer.full() )
        {
            // Remove first element from the accumulated value
            _accumulated_vector -= _direction_buffer.front();
            _direction_buffer.push_back( dir );
            _accumulated_vector += dir;
        }
        else
        {
            // Filling the buffer
            _direction_buffer.push_back( dir );
            _accumulated_vector += dir;
        }
    }

    /**
     * @brief The operation that's applied during scene graph traversal
     * @details This routine reads the data from the RemoteDataServer, scales the values
     * by the scale factors, then converts the euler angles into a quaternion and
     * extracts the magnitude (arc length) and direction of rotation
     * and prepares it for manipulation by the physics system.
     *
     * @param node Node* - unused, set by OSG
     * @param nv NodeVisitor - unused, set by OSG
     */
    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        comm->read();

        if ( !_disable_motion )
        {
            if ( comm->message_length() > 0 )
            {
                try
                {
                    float Vfwd, Vss, Omega, interval, r1, r2, r3, r4;
                    sscanf( comm->data(), "%f,%f,%f,%f,%f,%f,%f,%f",
                            &Vfwd, &Vss, &Omega, &r1, &r2, &r3, &r4, &interval );
                    //          Vfwd = -1.41421; Vss = 0.f;
                    // Average the gain colors
                    osg::Vec4f color;
                    if ( _contact_nodes.g_count > 0 )
                        color = _contact_nodes.gain / _contact_nodes.g_count;
                    else
                        color = _contact_nodes.gain;

                    // Convert to scaled angles
                    float Vfwdf = ( Vfwd / _x_scale ) * M_PI * color.g();
                    float Omegaf = ( Omega / _y_scale ) * M_PI * color.b();
                    float Vssf = ( Vss / _z_scale ) * M_PI * color.r();

                    _Omega = Omegaf;

                    float c1, c2, c3, s1, s2, s3;
                    c1 = cosf( Vssf / 2.f );
                    s1 = sinf( Vssf / 2.f );
                    c2 = cosf( Vfwdf / 2.f );
                    s2 = sinf( Vfwdf / 2.f );
                    c3 = cosf( Omegaf / 2.f );
                    s3 = sinf( Omegaf / 2.f );

                    osg::Quat q( s1 * c2 * c3 - c1 * s2 * s3,
                                 c1 * s2 * c3 + s1 * c2 * s3,
                                 c1 * c2 * s3 - s1 * s2 * c3,
                                 c1 * c2 * c3 + s1 * s2 * s3 );
                    osg::Vec3f v, v1 = q * _z_axis;
                    v1.z() = 0.f;
                    v1.normalize();
                    double ang;
                    osg::Vec3f axis;
                    q.getRotate( ang, axis );
                    v = v1 * ang * _radius;

                    //            std::cout << std::endl;
                    //            std::cout << Vfwd << ", " << Vss << ", " << Omega << ", " << interval << std::endl;
                    //            std::cout << Vfwdf << ", " << Vssf << ", " << Omegaf <<  std::endl;
                    //            std::cout << ang << ", " << axis << ", " << v << std::endl;

                    Motion_Data* md = ( Motion_Data* )ntm->getUserData();
                    md->motion.set( -v.x(), v.y(), v.z() );
                    md->raw.set( r1, r2, r3, r4 );

                    md->velocity = md->motion.length();
                    md->angle = ang;
                    md->roll = Vssf;
                    md->pitch = Vfwdf;

                    if ( _for_path_or_segment )
                        md->yaw = _Omega;
                    else
                    {
                        md->yaw += _Omega;

                        osg::Matrix mat;
                        mat.makeRotate( _Omega, _y_axis );
                        const osg::Quat& rot = ntm->getRotation();
                        osg::Quat r;
                        r.set( mat );
                        ntm->setRotation( r * rot );
                    }
                    md->interval = interval;
                    _displacement.set( -v.x(), v.y(), v.z() );
                }

                catch ( std::exception& e )
                {
                    std::cout << "Caught exception\n";
                    // Reset the connection
                    comm->close();
                    comm->reset();
                }
            }
        }
        else
        {
            float Vfwd, Vss, Omega;
            sscanf( comm->data(), "%f,%f,%f", &Vfwd, &Vss, &Omega );
            float Vfwdf = ( ( Vfwd / _x_scale ) * M_PI ) * _radius;
            float Vssf = ( ( Vss / _z_scale ) * M_PI ) * _radius;

            osg::Vec3d v = osg::Vec3d( -Vfwdf, Vssf, 0.f ) ;
            _displacement += v;
            Motion_Data* md = ( Motion_Data* )ntm->getUserData();
            md->motion = _offset;
        }

        // traverse at the end so then the camera update can run
        traverse( node, nv );
    }

  protected:
    Communicator* comm;
    boost::circular_buffer< osg::Vec3d > _direction_buffer;
    osg::Vec3d _y_axis, _z_axis;
    float _x_scale, _y_scale, _z_scale;

    float _radius;
    osg::Vec3d _accumulated_vector;
    float _Omega;
    bool _for_path_or_segment;
};

/**
 * @brief Constructor
 * @details Threshold_Camera_Update_Callback is a specialization of the Remote_Camera_Update_Callback.
 * This class is used under the special case where we are running in open field mode
 * (no path), but want to use thresold-based turning (where turn rate is
 * determined by a lookup table from the ratio of pitch to roll, and not yaw).
 */
class Threshold_Camera_Update_Callback : public Remote_Camera_Update_Callback
{
  public:
    /**
      * @brief Constructor
      * @details
      *
      * @param ma - Generic osg::CameraManipulator.
      * @param c - A Communicator. Reads the current values from the RemoteDataServer.
      * @param xs, ys, zs - Scale factors that convert the units from the RemoteDataServer into radians.
      * @param numSamples - The number of samples to use for smoothing the raw values from the RemoteDataServer.
      * @param nodes - The physics objects that contacted the camera in the last frame.
    */
    Threshold_Camera_Update_Callback( osgGA::CameraManipulator* ma, Communicator* c, float xs, float ys,
                                      float zs, float rad, int numSamples,
                                      ContactNodes const& nodes ):
        Remote_Camera_Update_Callback( ma, c, xs, ys, zs, rad, numSamples, nodes )
    {}

    /**
      * @brief Constructor
      * @details
      *
      * @param ma - A NodeTrackerManipulator, a specific type of an osg::CameraManipulator. An NTM attaches the camera to a geometry object and follows it around the world.
      * @param c - A Communicator. Reads the current values from the RemoteDataServer.
      * @param xs, ys, zs - Scale factors that convert the units from the RemoteDataServer into radians.
      * @param numSamples - The number of samples to use for smoothing the raw values from the RemoteDataServer.
      * @param nodes - The physics objects that contacted the camera in the last frame.
    */
    Threshold_Camera_Update_Callback( osgGA::NodeTrackerManipulator* ma, Communicator* c, float xs,
                                      float ys, float zs, float rad, int numSamples,
                                      ContactNodes const& nodes ):
        Remote_Camera_Update_Callback( ma, c, xs, ys, zs, rad, numSamples, nodes )
    {}

    /**
     * @brief Destructor
     * @details
     */
    ~Threshold_Camera_Update_Callback() {}

    /**
     * @brief The operation applied during scene graph traversal.
     * @details This routine reads the data from the RemoteDataServer and scales the values
     * by the scale factors. But, unlike it's parent class, it packs the scaled
     * values into a Motion_Data structure and defers the quaternion computation
     * to be closer to the point of usage.
     *
     * @param node Node* - unused, set by OSG
     * @param nv NodeVisitor - unused, set by OSG
     */
    void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        comm->read();

        if ( !_disable_motion )
        {
            if ( comm->message_length() > 0 )
            {
                try
                {
                    float Vfwd, Vss, Omega, interval, r1, r2, r3, r4;
                    sscanf( comm->data(), "%f,%f,%f,%f,%f,%f,%f,%f",
                            &Vfwd, &Vss, &Omega, &r1, &r2, &r3, &r4, &interval );
                    //          Vfwd = -1.41421; Vss = 0.f;
                    // Average the gain colors
                    osg::Vec4f color;
                    if ( _contact_nodes.g_count > 0 )
                        color = _contact_nodes.gain / _contact_nodes.g_count;
                    else
                        color = _contact_nodes.gain;

                    // Convert to scaled angles
                    float Vfwdf = ( Vfwd / _x_scale ) * M_PI * color.g();
                    float Omegaf = ( Omega / _y_scale ) * M_PI * color.b();
                    float Vssf = ( Vss / _z_scale ) * M_PI * color.r();

                    _Omega = Omegaf;

                    Motion_Data* md = ( Motion_Data* )ntm->getUserData();
                    md->raw.set( r1, r2, r3, r4 );

                    md->roll = Vssf;
                    md->pitch = Vfwdf;
                    md->yaw = Omegaf;
                    md->radius = _radius;
                    md->interval = interval;
                }

                catch ( std::exception& e )
                {
                    std::cout << "Caught exception\n";
                    // Reset the connection
                    comm->close();
                    comm->reset();
                }
            }
        }
        else
        {
            float Vfwd, Vss, Omega;
            sscanf( comm->data(), "%f,%f,%f", &Vfwd, &Vss, &Omega );
            float Vfwdf = ( ( Vfwd / _x_scale ) * M_PI ) * _radius;
            float Vssf = ( ( Vss / _z_scale ) * M_PI ) * _radius;

            osg::Vec3d v = osg::Vec3d( -Vfwdf, Vssf, 0.f ) ;
            _displacement += v;
            Motion_Data* md = ( Motion_Data* )ntm->getUserData();
            md->motion = _offset;
        }

        // traverse at the end so then the camera update can run
        traverse( node, nv );
    }
};

#endif
