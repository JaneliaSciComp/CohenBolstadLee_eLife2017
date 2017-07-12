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
#include <string>

#include <Communicator.h>

using boost::asio::ip::tcp;
using boost::asio::ip::udp;

TCPCommunicator::TCPCommunicator( std::string host, std::string port, int retries ):
    Communicator( host, port, retries ), socket( io_service )
{
    for ( int i = 0; i <= retries; i++ )
    {
        try
        {
            tcp::resolver resolver( io_service );
            tcp::resolver::query query( tcp::v4(), host, port );
            error = boost::asio::error::host_not_found;

            endpoint_iterator = resolver.resolve( query );

            socket.close();
            socket.connect( *endpoint_iterator, error );

            if ( error )
            {
                if ( i < retries )
                {
#ifndef _WINDOWS
                    std::cout << "Retrying in 3 secs" << std::endl;
                    sleep( 3 );
#endif
                }
            }
            else
            {
                success = true;
                // We close the socket so it can be reconnected in the camera callback
                //socket.close();
                break;
            }
        }
        catch ( std::exception& e )
        {
            if ( i < retries )
            {
#ifndef _WINDOWS
                std::cout << "Retrying in 3 secs" << std::endl;
                sleep( 3 );
#endif
            }
        }
    }
}

void
TCPCommunicator::read( void )
{
    boost::asio::write( socket, boost::asio::buffer( "", 10 ) );
    buf.assign( '\0' );

    reply_length = socket.read_some( boost::asio::buffer( buf ), error );

    if ( error == boost::asio::error::eof )
    {}
    else if ( error )
        throw boost::system::system_error( error ); // Some other error.
}

void
TCPCommunicator::write( std::string msg )
{
    boost::asio::write( socket, boost::asio::buffer( msg, msg.size() + 1 ) );
    reply_length = socket.read_some( boost::asio::buffer( buf ), error );
}

UDPCommunicator::UDPCommunicator( std::string host, std::string port, int retries ):
    Communicator( host, port, retries ), socket( io_service, udp::endpoint( udp::v4(), 0 ) )
{
    for ( int i = 0; i <= retries; i++ )
    {
        try
        {
            udp::resolver resolver( io_service );
            udp::resolver::query query( udp::v4(), host, port );

            endpoint_iterator = resolver.resolve( query );

            success = true;
            break;
        }
        catch ( std::exception& e )
        {
            if ( i < retries )
            {
#ifndef _WINDOWS
                std::cout << "Retrying in 3 secs" << std::endl;
                sleep( 3 );
#endif
            }
        }
    }
}

void
UDPCommunicator::read( void )
{
    socket.send_to( boost::asio::buffer( "", 1 ), *endpoint_iterator );
    udp::endpoint sender_endpoint;
    buf.assign( '\0' );
    reply_length = socket.receive_from( boost::asio::buffer( buf ), sender_endpoint );
}

