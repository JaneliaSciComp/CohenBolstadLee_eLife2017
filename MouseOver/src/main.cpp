/*
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

#include <QApplication>
#include <fstream>
using std::ifstream;
#include <iostream>
#include "Config_Memento.h"
#include "Console.h"

int main( int argc, char** argv )
{
    QApplication app( argc, argv );
    Q_INIT_RESOURCE( jovian );

    osg::ArgumentParser arguments( &argc, argv );

    Console_Ptr console = Console_Ptr( new Console( arguments ) );
    console->setGeometry( 100, 100, 800, 600 );
    console->show();

    app.setQuitOnLastWindowClosed( true );

    char* fileName = getenv( "JOVIAN_CONFIG_FILE" );
    if ( fileName != NULL )
    {
        ifstream config( fileName );
        Config_Memento* cm = console->model()->current_configuration();
        cm->initialize( console.get() );
        cm->load( config );

        console->load_config( cm );
        config.close();
    }

    return app.exec();
}
