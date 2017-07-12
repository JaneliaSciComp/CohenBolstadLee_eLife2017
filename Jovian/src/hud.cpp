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

#include <osg/Geode>

#include <hud.h>

osg::Camera* createHUD( osg::Switch* switch_node, int numCameras,
                        int left, int right, int top, int bottom,
                        osg::Vec2 lower_left, int size, int offset )
{
	// create a camera to set up the projection and model view matrices, and the subgraph to drawn in the HUD
	osg::Camera* camera = new osg::Camera;

	// set the projection matrix
	camera->setProjectionMatrix(osg::Matrix::ortho2D( 0, (right - left),
	                                                  0, bottom - top ) );

	// set the view matrix
	camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
	camera->setViewMatrix( osg::Matrix::identity() );

	// only clear the depth buffer
	camera->setClearMask( GL_DEPTH_BUFFER_BIT );

	// draw subgraph after main camera view.
	camera->setRenderOrder( osg::Camera::POST_RENDER );

	// we don't want the camera to grab event focus from the viewers main camera(s).
	camera->setAllowEventFocus( false );

	// add to this camera a subgraph to render
	{
		osg::Geode* geode = new osg::Geode();
		osg::Group* group = new osg::Group();
		osg::Geometry* bk_geom = new osg::Geometry;

		// turn lighting off for the text and disable depth test to ensure its always ontop.
		osg::StateSet* stateset = geode->getOrCreateStateSet();
		stateset->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
		{
			osg::Geometry* geom_white = new osg::Geometry;
			osg::Geometry* geom_black = new osg::Geometry;
			osg::Geometry* geom_yellow = new osg::Geometry;
			osg::Geometry* geom_purple = new osg::Geometry;
			osg::Geometry* geom_green = new osg::Geometry;

			osg::Vec3Array* vertices = new osg::Vec3Array;
			osg::Vec3Array* bk_vertices = new osg::Vec3Array;

			vertices->push_back( osg::Vec3( lower_left.x(),                   lower_left.y() + size, 0 ) );
			vertices->push_back( osg::Vec3( lower_left.x(),                   lower_left.y(), 0 ) );
			vertices->push_back( osg::Vec3( lower_left.x() + numCameras*size, lower_left.y(), 0 ) );
			vertices->push_back( osg::Vec3( lower_left.x() + numCameras*size, lower_left.y() + size, 0 ) );
			bk_vertices->push_back( osg::Vec3( lower_left.x() - (numCameras*offset), lower_left.y() + size + offset, -0.1f ) );
			bk_vertices->push_back( osg::Vec3( lower_left.x() - (numCameras*offset), lower_left.y() - offset, -0.1f ) );
			bk_vertices->push_back( osg::Vec3( lower_left.x() + numCameras*(size + offset), lower_left.y() - offset, -0.1f ) );
			bk_vertices->push_back( osg::Vec3( lower_left.x() + numCameras*(size + offset), lower_left.y() + size + offset, -0.1f ) );
			geom_white->setVertexArray( vertices );
			geom_black->setVertexArray( vertices );
			geom_yellow->setVertexArray( vertices );
			geom_purple->setVertexArray( vertices );
			geom_green->setVertexArray( vertices );
			bk_geom->setVertexArray( bk_vertices );

			osg::Vec3Array* normals = new osg::Vec3Array;
			normals->push_back( osg::Vec3( 0.0f, 0.0f, 1.0f ) );
			geom_white->setNormalArray( normals );
			geom_white->setNormalBinding( osg::Geometry::BIND_OVERALL );
			geom_black->setNormalArray( normals );
			geom_black->setNormalBinding( osg::Geometry::BIND_OVERALL );
			geom_yellow->setNormalArray( normals );
			geom_yellow->setNormalBinding( osg::Geometry::BIND_OVERALL );
			geom_purple->setNormalArray( normals );
			geom_purple->setNormalBinding( osg::Geometry::BIND_OVERALL );
			geom_green->setNormalArray( normals );
			geom_green->setNormalBinding( osg::Geometry::BIND_OVERALL );

			osg::Vec4Array* colors = new osg::Vec4Array;
			colors->push_back( osg::Vec4( 1.0f, 1.0, 1.0f, 1.0f ) );
			geom_white->setColorArray( colors );
			geom_white->setColorBinding( osg::Geometry::BIND_OVERALL );

			colors = new osg::Vec4Array;
			colors->push_back( osg::Vec4( 0.0f, 0.0, 0.0f, 1.0f ) );
			geom_black->setColorArray( colors );
			geom_black->setColorBinding( osg::Geometry::BIND_OVERALL );
			bk_geom->setColorArray( colors );
			bk_geom->setColorBinding( osg::Geometry::BIND_OVERALL );

			colors = new osg::Vec4Array;
			colors->push_back( osg::Vec4( 1.0f, 1.0, 0.0f, 1.0f ) );
			geom_yellow->setColorArray( colors );
			geom_yellow->setColorBinding( osg::Geometry::BIND_OVERALL );

			colors = new osg::Vec4Array;
			colors->push_back( osg::Vec4( 1.0f, 0.0, 1.0f, 1.0f ) );
			geom_purple->setColorArray( colors );
			geom_purple->setColorBinding( osg::Geometry::BIND_OVERALL );

			colors = new osg::Vec4Array;
			colors->push_back( osg::Vec4( 0.0f, 1.0, 0.0f, 1.0f ) );
			geom_green->setColorArray( colors );
			geom_green->setColorBinding( osg::Geometry::BIND_OVERALL );

			geom_white->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4 ) );
			geom_black->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4 ) );
			geom_yellow->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4 ) );
			geom_purple->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4 ) );
			geom_green->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4 ) );
			bk_geom->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4 ) );

			osg::StateSet* stateset = geom_white->getOrCreateStateSet();
			stateset->setMode( GL_BLEND,osg::StateAttribute::ON );
			//stateset->setAttribute(new osg::PolygonOffset(1.0f,1.0f),osg::StateAttribute::ON);
			stateset->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );

			geode->addDrawable( geom_yellow );
			switch_node->addChild( geode );

			geode = new osg::Geode();
			geode->addDrawable( geom_white );
			switch_node->addChild( geode );

			geode = new osg::Geode();
			geode->addDrawable( geom_black );
			switch_node->addChild( geode );

			geode = new osg::Geode();
			geode->addDrawable( geom_purple );
			switch_node->addChild( geode );

			geode = new osg::Geode();
			geode->addDrawable( geom_green );
			switch_node->addChild( geode );
		}

		switch_node->setSingleChildOn( 0 );
		geode = new osg::Geode();
		geode->addDrawable( bk_geom );
		group->addChild( switch_node );
		group->addChild( geode );
		camera->addChild( group );
	}

	return camera;
}
