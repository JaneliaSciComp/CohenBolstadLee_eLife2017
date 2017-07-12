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

#include <fstream>
using std::ofstream;
using std::ifstream;
#include <iostream>
using std::cout;
using std::endl;

#include <math.h>

#include <QtCore/QDebug>

#include <QGraphicsScene>
#include <QWheelEvent>

#include <graphwidget.h>
#include <edge.h>
#include <node.h>
#include <viewingWindowQt.h>

Graph_Widget::Graph_Widget(QWidget* parent ): QGraphicsView( parent ), push_weight( 50. ), pull_weight( 15. ), _timerId(0), _basis( 4 ), _mesh( 4 ), _viewer( 0 )
{
	QGraphicsScene *scene = new QGraphicsScene(this);
	scene->setItemIndexMethod(QGraphicsScene::NoIndex);
	scene->setSceneRect(-200, -200, 400, 400);
	setScene(scene);
	setCacheMode(CacheBackground);
	setViewportUpdateMode(BoundingRectViewportUpdate);
	setRenderHint(QPainter::Antialiasing);
	setTransformationAnchor(AnchorUnderMouse);
	setResizeAnchor(AnchorViewCenter);

	QVector< Node* > n;

	for ( int i = 0; i < 4; i++ )
	{
		for ( int j = 0; j < 4; j++ )
		{
			Node* node = new Node( this, i, j );
			n.push_back( node );
			scene->addItem( node );
		}
		nodes.push_back( n );
		n.clear();
	}


	for ( int i = 0; i < 4; i++ )
	{
		for ( int j = 0; j < 4; j++ )
		{
			if ( j < 3 )
			{
				Edge* edge = new Edge( nodes[i][j], nodes[i][j+1] );
				scene->addItem( edge );
			}
			if ( i < 3 )
			{
				Edge* edge = new Edge( nodes[i][j], nodes[i+1][j] );
				scene->addItem( edge );
			}

			nodes[i][j]->setPos( -200 + (j/3.) * 400., -200 + (i/3.) * 400. );
		}
	}

	setMinimumSize(400, 400);

	// Build Bernstein matrix for Bezier surface
	QVector< qreal > b( 4 );
	b[0] = -1.; b[1] =  3.; b[2] = -3.; b[3] = 1.;
	_basis[0] = b;
	b[0] =  3.; b[1] = -6.; b[2] =  3.; b[3] = 0.;
	_basis[1] = b;
	b[0] = -3.; b[1] =  3.; b[2] =  0.; b[3] = 0.;
	_basis[2] = b;
	b[0] =  1.; b[1] =  0.; b[2] =  0.; b[3] = 0.;
	_basis[3] = b;

	_origin = nodes[0][0]->pos();
}

void Graph_Widget::node_moved()
{
	if ( _viewer )
	{
		compute_basis_matrix();
		_viewer->setup_slaves();
	}
}

void
Graph_Widget::compute_basis_matrix( )
{
	QVector< QVector< QPointF > > c( 4 );

	for ( int i = 0; i < 4; ++i )
	{
		QVector< QPointF > b( 4 );

		for (int j = 0; j < 4; ++j)
			b[j] = _basis[i][0] * nodes[0][j]->pos() +
				_basis[i][1] * nodes[1][j]->pos() +
				_basis[i][2] * nodes[2][j]->pos() +
				_basis[i][3] * nodes[3][j]->pos();

		c[i] = b;
	}

	for ( int i = 0; i < 4; ++i )
	{
		QVector< QPointF > b( 4 );

		for (int j = 0; j < 4; ++j)
			b[j] = c[i][0] * _basis[0][j] +
				c[i][1] * _basis[1][j] +
				c[i][2] * _basis[2][j] +
				c[i][3] * _basis[3][j];

		_mesh[i] = b;
	}
}

QPointF
Graph_Widget::evaluate( float u, float v )
{
	qreal s[4] = { u*u*u, u*u, u, 1. };
	qreal t[4] = { v*v*v, v*v, v, 1. };
	QVector< QPointF > temp( 4 );

	for ( int i = 0; i < 4; i++ )
	{
		QPointF pt;

		for ( int j = 0; j < 4; j++ )
			pt += s[j] * _mesh[i][j];

		temp[i] = pt;
	}

	QPointF result;
	for ( int i = 0; i < 4; i++ )
	{
		result += temp[i] * t[i];
	}

	result -= _origin;

	return QPointF( (qreal)result.x() / minimumWidth(), (qreal)result.y() / minimumHeight() );
}

void
Graph_Widget::distribute_horizontally()
{
	Q_FOREACH ( QVector< Node* > v, nodes )
	{
		QPointF startPos = v[0]->pos();
		QPointF endPos = v[v.size() - 1]->pos();
		qreal length = endPos.x() - startPos.x();

		for ( int i = 1; i < v.size() - 1; i++ )
		{
			QPointF p = v[0]->pos() + QPointF( i * ( length / ( v.size() - 1 )), 0 );
			v[i]->setPos( QPointF( p.x(), v[i]->pos().y() ) );
		}
	}
}

void
Graph_Widget::distribute_vertically()
{
	for ( int j = 0; j < nodes[0].size(); j++ )
	{
		QPointF startPos = nodes[0][j]->pos();
		QPointF endPos = nodes[nodes.size() - 1][j]->pos();
		qreal length = endPos.y() - startPos.y();

		for ( int i = 1; i < nodes.size() - 1; i++ )
		{
			QPointF p = nodes[0][j]->pos() + QPointF( 0, i * ( length / ( nodes.size() - 1 )));
			nodes[i][j]->setPos( QPointF( nodes[i][j]->x(), p.y() ) );
		}
	}
}

void Graph_Widget::reset()
{
	for ( int i = 0; i < 4; i++ )
		for ( int j = 0; j < 4; j++ )
			nodes[i][j]->setPos( -200 + (j/3.) * 400., -200 + (i/3.) * 400. );
}

void Graph_Widget::smooth()
{
	if (!_timerId)
		_timerId = startTimer(1000 / 25);
	else
	{
		killTimer(_timerId);
		_timerId = 0;
	}
}

void
Graph_Widget::linearize_edges()
{
	linearize_edge_in_y( nodes[0][0], nodes[1][0], nodes[2][0], nodes[3][0] );
	linearize_edge_in_y( nodes[0][3], nodes[1][3], nodes[2][3], nodes[3][3] );
	linearize_edge_in_x( nodes[0][0], nodes[0][1], nodes[0][2], nodes[0][3] );
	linearize_edge_in_x( nodes[3][0], nodes[3][1], nodes[3][2], nodes[3][3] );
}

void
Graph_Widget::save( GraphPositions& gp )
{
	gp.resize( nodes.size() );

	for ( int i = 0; i < nodes.size(); i++ )
	{
		gp[i].resize( nodes[0].size() );

		for ( int j = 0; j < nodes[0].size(); j++ )
			gp[i][j] = std::make_pair( nodes[i][j]->pos().x(), nodes[i][j]->pos().y() );
	}
}

void
Graph_Widget::load( GraphPositions const& gp )
{
	if ( gp.size() == nodes.size() && gp[0].size() == nodes[0].size() )
	{
		for ( int i = 0; i < gp.size(); i++ )
		{
			for ( int j = 0; j < gp[0].size(); j++ )
			{
				nodes[i][j]->setPos( gp[i][j].first, gp[i][j].second );
			}
		}
	}
}

void Graph_Widget::keyPressEvent(QKeyEvent *event)
{
	switch (event->key()) {
		case Qt::Key_Up:
			break;
		case Qt::Key_Down:
			distribute_vertically();
			break;
		case Qt::Key_Left:
			break;
		case Qt::Key_Right:
			distribute_horizontally();
			break;
		case Qt::Key_Plus:
			scale_view(qreal(1.2));
			break;
		case Qt::Key_Minus:
			scale_view(1 / qreal(1.2));
			break;
		case Qt::Key_Space:
			smooth();
			break;
		default:
			QGraphicsView::keyPressEvent(event);
	}
}

void Graph_Widget::timerEvent(QTimerEvent *event)
{
	Q_UNUSED(event);

	QList<Node *> node_list;
	Q_FOREACH (QGraphicsItem *item, scene()->items()) {
		if (Node *node = qgraphicsitem_cast<Node *>(item))
			node_list << node;
	}

	Q_FOREACH (Node *node, node_list)
		if ( node->row > 0 && node->col > 0 &&
		     node->row < nodes.size()-1 && node->col < nodes[0].size()-1 )
			node->calculateForces();

	bool itemsMoved = false;
	Q_FOREACH (Node *node, node_list) {
		if ( node->row > 0 && node->col > 0 &&
		     node->row < nodes.size()-1 && node->col < nodes[0].size()-1 )
			if (node->advance())
				itemsMoved = true;
	}

	if (!itemsMoved) {
		killTimer(_timerId);
		_timerId = 0;
	}
}

void Graph_Widget::wheelEvent(QWheelEvent *event)
{
    scale_view(pow((double)2, -event->delta() / 240.0));
}

void Graph_Widget::drawBackground(QPainter *painter, const QRectF &rect)
{
	Q_UNUSED(rect);

	// Shadow
	QRectF sceneRect = this->sceneRect();
	QRectF rightShadow(sceneRect.right(), sceneRect.top() + 5, 5, sceneRect.height());
	QRectF bottomShadow(sceneRect.left() + 5, sceneRect.bottom(), sceneRect.width(), 5);
	if (rightShadow.intersects(rect) || rightShadow.contains(rect))
		painter->fillRect(rightShadow, Qt::darkGray);
	if (bottomShadow.intersects(rect) || bottomShadow.contains(rect))
		painter->fillRect(bottomShadow, Qt::darkGray);

	// Fill
	QLinearGradient gradient(sceneRect.topLeft(), sceneRect.bottomRight());
	gradient.setColorAt(0, Qt::white);
	gradient.setColorAt(1, Qt::lightGray);
	painter->fillRect(rect.intersected(sceneRect), gradient);
	painter->setBrush(Qt::NoBrush);
	painter->drawRect(sceneRect);
}

void Graph_Widget::scale_view(qreal scaleFactor)
{
    qreal factor = matrix().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
    if (factor < 0.07 || factor > 100)
        return;

    scale(scaleFactor, scaleFactor);
}

void
Graph_Widget::linearize_edge_in_x( Node* n1, Node* n2, Node* n3, Node* n4 )
{
	QPointF p1 = n1->pos();
	QPointF p2 = n2->pos();
	QPointF p3 = n3->pos();
	QPointF p4 = n4->pos();
	QPointF v = p4 - p1;

	qreal length = sqrt( v.x() * v.x() + v.y() * v.y() );
	qreal d = ( p2.x() - p1.x() ) / length;
	n2->setPos( p2.x(), p1.y() + d * v.y() );
	d = ( p3.x() - p1.x() ) / length;
	n3->setPos( p3.x(), p1.y() + d * v.y() );
}

void
Graph_Widget::linearize_edge_in_y( Node* n1, Node* n2, Node* n3, Node* n4 )
{
	QPointF p1 = n1->pos();
	QPointF p2 = n2->pos();
	QPointF p3 = n3->pos();
	QPointF p4 = n4->pos();
	QPointF v = p4 - p1;

	qreal length = sqrt( v.x() * v.x() + v.y() * v.y() );
	qreal d = ( p2.y() - p1.y() ) / length;
	n2->setPos( p1.x() + d * v.x(), p2.y() );
	d = ( p3.y() - p1.y() ) / length;
	n3->setPos( p1.x() + d * v.x(), p3.y() );
}

