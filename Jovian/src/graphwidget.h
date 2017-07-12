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

/****************************************************************************
**
** Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL$
** Commercial Usage
** Licensees holding valid Qt Commercial licenses may use this file in
** accordance with the Qt Commercial License Agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and Nokia.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU Lesser General Public License version 2.1 requirements
** will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** In addition, as a special exception, Nokia gives you certain additional
** rights.  These rights are described in the Nokia Qt LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3.0 as published by the Free Software
** Foundation and appearing in the file LICENSE.GPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU General Public License version 3.0 requirements will be
** met: http://www.gnu.org/copyleft/gpl.html.
**
** If you have questions regarding the use of this file, please contact
** Nokia at qt-info@nokia.com.
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <fstream>

#include <QtCore/QVector>

#include <QGraphicsView>

#include <Globals.h>

class Node;
class Viewing_Window_Qt;

/** @brief Graph_Widget.
 * @details This class was lifted almost in it's entirety from a Qt demo. It
 * encapsulates the editing and drawing of a Bezier Surface.
 */

class Graph_Widget: public QGraphicsView
{

    Q_OBJECT

  public:

    /// @name Initialization
    ///@{
    Graph_Widget( QWidget* parent = 0 );
    ///@}

    /// @name Duplication
    ///@{
    ///@}

    /// @name Move
    ///@{
    ///@}

    /// @name Destruction
    ///@{
    ///@}

    /// @name Access
    ///@{
    /**
     * @brief the locations of the nodes in the widget
     * @details The nodes are the control points for a bezier surface. As such, nodes is a
     * 4x4 array of Node objects
     */
    QVector< QVector< Node* > > nodes;
    float push_weight, pull_weight;

    ///@}
    /// @name Measurement
    ///@{
    ///@}
    /// @name Comparison
    ///@{
    ///@}
    /// @name Status report
    ///@{
    ///@}
    /// @name Status setting
    ///@{
    ///@}
    /// @name Cursor movement
    ///@{
    ///@}
    /// @name Element change
    ///@{
    void set_viewer( Viewing_Window_Qt* viewer ) { _viewer = viewer; }
    void save( GraphPositions& gp );
    void load( GraphPositions const& gp );

    ///@}
    /// @name Removal
    ///@{
    ///@}
    /// @name Resizing
    ///@{
    ///@}
    /// @name Transformation
    ///@{
    ///@}
    /// @name Conversion
    ///@{
    ///@}
    /// @name Basic operations
    ///@{
    /**
     * @brief Evaluate the graph at `u', `v'
     * @details Evaluates the bezier surface representation at the poin `u,, `v'
     *
     * @param u The horizontal parameter in the range [0, 1]
     * @param v The vertical parameter in the range [0, 1]
     *
     * @return QPointF - A floating-point 2D Point
     */
    QPointF evaluate( float u, float v );

    void compute_basis_matrix();
    /**
     * @brief distribute nodes along horizontal axis
     * @details For each row, evenly divide the distance between the min node and
     * max node and move interior nodes to computed positions
     */
    void distribute_horizontally();

    /**
     * @brief distribute nodes along vertical axis
     * @details For each row, evenly divide the distance between the min node and
     * max node and move interior nodes to computed positions
     */
    void distribute_vertically();

    /**
     * @brief Called when a node was moved
     * @details When a node is moved, the Node class calls back to this routine to
     * recompute the basis matrix and notify the viewer of an update.
     */
    void node_moved();

    /**
     * @brief Reset the nodes to default position
     * @details Resets the nodes to the default positions, 4 rows of 4 distributed
     * horizontally and vertically.
     */
    void reset();

    /**
     * @brief Runs an N-Body simulation to distribute nodes
     * @details Run a couple of iterations of an N-Body simulation to distribute `forces'
     * equally between the nodes
     */
    void smooth();

    /**
     * @brief Linearize each of the outside edges
     * @details For each edge of the perimeter, move the two interior points to the
     * closest position on the line connecting the first and last points. Interior
     * points are not moved.
     */
    void linearize_edges();

    ///@}
    /// @name Miscellaneous
    ///@{
    ///@}
    /// @name Obsolete
    ///@{
    ///@}
    /// @name Inapplicable
    ///@{
    ///@}

  protected:
    ///@{
    void keyPressEvent( QKeyEvent* event );
    void timerEvent( QTimerEvent* event );
    void wheelEvent( QWheelEvent* event );
    void drawBackground( QPainter* painter, const QRectF& rect );

    void scale_view( qreal scaleFactor );
    void linearize_edge_in_x( Node* n1, Node* n2, Node* n3, Node* n4 );
    void linearize_edge_in_y( Node* n1, Node* n2, Node* n3, Node* n4 );
    ///@}

  private:
    int _timerId;
    QPointF _origin;
    QVector< QVector< qreal > > _basis;
    QVector< QVector< QPointF > > _mesh;
    Viewing_Window_Qt* _viewer;

};  // end of class Graphwidget
#endif
