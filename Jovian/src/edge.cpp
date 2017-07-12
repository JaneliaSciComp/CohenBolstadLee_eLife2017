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

#include <iostream>

#include <QtCore/QVector>

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>

#include <edge.h>
#include <graphwidget.h>
#include <node.h>

#include <math.h>

static const double Pi = 3.14159265358979323846264338327950288419717;
static double TwoPi = 2.0 * Pi;
static QVector< Edge* > edges_to_delete;

Edge::Edge(Node *sourceNode, Node *destNode)
    : arrowSize(10)
{
	//setAcceptedMouseButtons(0);
	source = sourceNode;
	dest = destNode;
	source->addEdge(this);
	dest->addEdge(this);
	adjust();
}

Edge::~Edge()
{
}

Node *Edge::sourceNode() const
{
	return source;
}

void Edge::setSourceNode(Node *node)
{
	source = node;
	adjust();
}

Node *Edge::destNode() const
{
	return dest;
}

void Edge::setDestNode(Node *node)
{
	dest = node;
	adjust();
}

void Edge::adjust()
{
	if (!source || !dest)
		return;

	QLineF line(mapFromItem(source, 0, 0), mapFromItem(dest, 0, 0));
	qreal length = line.length();

	prepareGeometryChange();

	if (length > qreal(20.)) {
		QPointF edgeOffset((line.dx() * 10) / length, (line.dy() * 10) / length);
		sourcePoint = line.p1() + edgeOffset;
		destPoint = line.p2() - edgeOffset;
	} else {
		sourcePoint = destPoint = line.p1();
	}
}

QRectF Edge::boundingRect() const
{
	if (!source || !dest)
		return QRectF();

	qreal penWidth = 1;
	qreal extra = (penWidth + arrowSize) / 2.0;

	return QRectF(sourcePoint, QSizeF(destPoint.x() - sourcePoint.x(),
	                                  destPoint.y() - sourcePoint.y()))
		.normalized()
		.adjusted(-extra, -extra, extra, extra);
}

void Edge::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
	if (!source || !dest)
		return;

	QLineF line(sourcePoint, destPoint);
	if (qFuzzyCompare(line.length(), qreal(0.)))
		return;

	// Draw the line itself
	painter->setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
	painter->drawLine(line);

	// Draw the arrows
	double angle = ::acos(line.dx() / line.length());
	if (line.dy() >= 0)
		angle = TwoPi - angle;

	QPointF sourceArrowP1 = sourcePoint + QPointF(sin(angle + Pi / 3) * arrowSize,
	                                              cos(angle + Pi / 3) * arrowSize);
	QPointF sourceArrowP2 = sourcePoint + QPointF(sin(angle + Pi - Pi / 3) * arrowSize,
	                                              cos(angle + Pi - Pi / 3) * arrowSize);
	QPointF destArrowP1 = destPoint + QPointF(sin(angle - Pi / 3) * arrowSize,
	                                          cos(angle - Pi / 3) * arrowSize);
	QPointF destArrowP2 = destPoint + QPointF(sin(angle - Pi + Pi / 3) * arrowSize,
	                                          cos(angle - Pi + Pi / 3) * arrowSize);

	painter->setBrush(Qt::black);
	painter->drawPolygon(QPolygonF() << line.p1() << sourceArrowP1 << sourceArrowP2);
	painter->drawPolygon(QPolygonF() << line.p2() << destArrowP1 << destArrowP2);
}

void
Edge::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
{
	Graph_Widget* graph = source->graph_widget();
	qreal frac = (event->pos() - sourcePoint).manhattanLength() /
		(destPoint - sourcePoint).manhattanLength();
	int s_row = source->row;
	int s_col = source->col;
	int d_row = dest->row;
	int d_col = dest->col;
	QVector< Node* > new_nodes;

	// Delete edges marked for deletion from previous update
	if ( edges_to_delete.size() > 0 )
	{
		Q_FOREACH( Edge* e, edges_to_delete )
			scene()->removeItem( e );

		edges_to_delete.clear();
	}

	if ( s_row == d_row )
	{
		for ( int i = 0; i < graph->nodes.size(); i++ )
		{
			QVector< Node* >& v = graph->nodes[i];
			// update the indices for each row
			for ( int j = 0; j < v.size(); j++ )
				if ( j > s_col )
					(v[j]->col)++;

			Node* node = new Node( graph, i, d_col );
			new_nodes.push_back( node );
			scene()->addItem( node );
			scene()->addItem( new Edge( v[s_col], node ) );
			scene()->addItem( new Edge( node, v[d_col] ) );

			Edge* edge;
			Q_FOREACH( Edge* e, v[s_col]->edges() )
			{
				if ( e->source == v[s_col] && e->dest == v[d_col] )
					edge = e;
			}

			QPointF vec = (edge->destPoint - edge->sourcePoint) * frac;

			node->setPos( edge->sourcePoint + vec );
			edge->hide();
			edges_to_delete.push_back( edge );

			v.insert( d_col, node );
		}
	}
	else 	if ( s_col == d_col )
	{
		// update the indices for each row after insertion point
		for ( int i = d_row; i < graph->nodes.size(); i++ )
		{
			QVector< Node* >& v = graph->nodes[i];
			for ( int j = 0; j < v.size(); j++ )
					(v[j]->row)++;
		}

		for ( int i = 0; i < graph->nodes[0].size(); i++ )
		{
			Node* node = new Node( graph, d_row, i );
			new_nodes.push_back( node );
			scene()->addItem( node );
		}

		QVector< Node* >& e_row = graph->nodes[s_row];
		QVector< Node* >& e_col = graph->nodes[d_row];
		for ( int i = 0; i < e_row.size(); i++ )
		{
			Edge* edge;
			Q_FOREACH( Edge* e, e_row[i]->edges() )
			{
				if ( e->source == e_row[i] && e->dest == e_col[i] )
					edge = e;
			}

			QPointF vec = (edge->destPoint - edge->sourcePoint) * frac;

			scene()->addItem( new Edge( e_row[i], new_nodes[i] ) );
			scene()->addItem( new Edge( new_nodes[i], e_col[i] ) );
			new_nodes[i]->setPos( edge->sourcePoint + vec );
			edge->hide();
			edges_to_delete.push_back( edge );
		}

		graph->nodes.insert( d_row, new_nodes );
	}
	else
		std::cout << "No idea how in the hell I got to this point. Should be impossible.\n";

	// Add an edge for each new node
	for ( int i = 0; i < new_nodes.size() - 1; i++ )
		scene()->addItem( new Edge( new_nodes[i], new_nodes[i+1] ) );
}


