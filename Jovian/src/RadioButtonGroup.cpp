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

#include <iostream>

#include <QtCore/QFileInfo>
#include <QApplication>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

#include <RadioButtonGroup.h>

Radio_Button_Group::Radio_Button_Group( QWidget* parent, QVBoxLayout* layout, QSpacerItem* spacer ): _parent( parent ), _group_box_layout( layout ), _spacer( spacer )
{
	_grouper = new QButtonGroup( _parent );
	connect( _grouper, SIGNAL( buttonClicked(int) ), this, SLOT( selected(int) ) );
}

Radio_Button_Group::~Radio_Button_Group()
{
	clear();
}

void
Radio_Button_Group::load( QString name )
{
	_names.push_back( name.toStdString() );
	QFrame* frame_3 = new QFrame( _parent );
	frame_3->setFrameShape(QFrame::StyledPanel);
	frame_3->setFrameShadow(QFrame::Raised);

	QHBoxLayout* horizontalLayout_47 = new QHBoxLayout(frame_3);
#ifndef Q_OS_MAC
	horizontalLayout_47->setSpacing(-1);
#endif
	horizontalLayout_47->setContentsMargins(10, 0, 0, 0);

	QRadioButton* radioButton_4 = new QRadioButton( frame_3 );
	radioButton_4->setText( name );
	radioButton_4->setChecked( true );

	horizontalLayout_47->addWidget(radioButton_4);

	_widgets.push_back( frame_3 );
	_grouper->addButton( radioButton_4, _widgets.size() );

	horizontalLayout_47->setStretch(1, 1);
	horizontalLayout_47->setStretch(2, 1);

	_group_box_layout->insertWidget( _group_box_layout->count() - 1, frame_3 );
}

void
Radio_Button_Group::clear()
{
	QList< QAbstractButton* > buttons = _grouper->buttons();
	QList< QAbstractButton* >::iterator iter;
	for (iter = buttons.begin(); iter != buttons.end(); iter++)
		_grouper->removeButton(*iter);

	std::vector< QFrame* >::reverse_iterator rit;
	for ( rit = _widgets.rbegin(); rit != _widgets.rend(); rit++ )
	{
		(*rit)->hide();
		_group_box_layout->removeWidget( *rit );
	}

	_names.clear();
	_widgets.clear();
}

void
Radio_Button_Group::select( int index )
{
	_grouper->button( index + 1 )->setChecked( true );
}

void
Radio_Button_Group::selected( int index )
{
	std::cout << "Selected id " << index << ", " << _names[index-1] << std::endl;
	Q_EMIT activated( index - 1 );
}

