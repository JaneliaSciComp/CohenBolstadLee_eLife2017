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

/* -*-c++-*- OpenSceneGraph - Copyright (C) 2009 Wang Rui
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/

#ifndef OSGVIEWER_MYGRAPHICSWINDOWQT
#define OSGVIEWER_MYGRAPHICSWINDOWQT

#include <QWidget>
#include <QInputEvent>
#include <QGLWidget>
#include <osgViewer/GraphicsWindow>

/**
 * @brief Create a custom OpenGL widget for rendering
 * @details Primarily for OS X development. The built-in OSG OpenGL and
 * GraphicsWindow widgets didn't work properly on either Windows or OS X. At some
 * point, the Windows version was fixed, but the OS X version lags behind. When
 * the OS X version is functional, these class will be removed.
 */
class OpenGLWidget : public QGLWidget
{
  public:
    OpenGLWidget( const QGLFormat& format, QWidget* parent = 0, const QGLWidget* shareWidget = 0, Qt::WindowFlags f = 0 );

    inline void setGraphicsWindow( osgViewer::GraphicsWindow* gw ) { _gw = gw; }

    void setKeyboardModifiers( QInputEvent* event );

    virtual void resizeEvent( QResizeEvent* event );
    virtual void keyPressEvent( QKeyEvent* event );
    virtual void keyReleaseEvent( QKeyEvent* event );
    virtual void mousePressEvent( QMouseEvent* event );
    virtual void mouseReleaseEvent( QMouseEvent* event );
    virtual void mouseDoubleClickEvent( QMouseEvent* event );
    virtual void mouseMoveEvent( QMouseEvent* event );
    virtual void wheelEvent( QWheelEvent* event );

  protected:
    osgViewer::GraphicsWindow* _gw;
};

/**
 * @brief Create a custom QtOpenGL widget for rendering
 * @details Primarily for OS X development. The built-in OSG OpenGL and
 * GraphicsWindow widgets didn't work properly on either Windows or OS X. At some
 * point, the Windows version was fixed, but the OS X version lags behind. When
 * the OS X version is functional, these class will be removed.
 */
class MyGraphicsWindowQt : public osgViewer::GraphicsWindow
{
  public:
    MyGraphicsWindowQt( osg::GraphicsContext::Traits* traits );
    virtual ~MyGraphicsWindowQt();

    inline OpenGLWidget* getOpenGLWidget() { return _widget; }
    inline const OpenGLWidget* getOpenGLWidget() const { return _widget; }

    struct WindowData : public osg::Referenced
    {
        WindowData( OpenGLWidget* widget ): _widget( widget ) {}
        OpenGLWidget* _widget;
    };

    bool init();

    virtual bool setWindowRectangleImplementation( int x, int y, int width, int height );
    virtual void getWindowRectangle( int& x, int& y, int& width, int& height );
    virtual bool setWindowDecorationImplementation( bool windowDecoration );
    virtual bool getWindowDecoration() const;
    virtual void grabFocus();
    virtual void grabFocusIfPointerInWindow();
    virtual void raiseWindow();
    virtual void setWindowName( const std::string& name );
    virtual std::string getWindowName();
    virtual void useCursor( bool cursorOn );
    virtual void setCursor( MouseCursor cursor );

    virtual bool valid() const;
    virtual bool realizeImplementation();
    virtual bool isRealizedImplementation() const;
    virtual void closeImplementation();
    virtual bool makeCurrentImplementation();
    virtual bool releaseContextImplementation();
    virtual void swapBuffersImplementation();

    virtual void requestWarpPointer( float x, float y );

  protected:
    OpenGLWidget* _widget;
    QCursor _currentCursor;
    unsigned int _width, _height;
    bool _initialized;
    bool _realized;
};

#endif
