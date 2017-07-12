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

#ifndef __COMMUNICATOR__
#define __COMMUNICATOR__

#include <iostream>
#include <string>

#ifndef _WINDOWS
#include <unistd.h>
#endif

#ifndef Q_MOC_RUN
#include <boost/array.hpp>
#include <boost/asio.hpp>
#endif

/**
 * @briefBase class for communicating with a data server
 * @details Abstract base class for communicating to a data server.
 */
class Communicator
{
  public:

    Communicator( std::string host = "localhost", std::string port = "22222", int retries = 2 ): success( false ), reply_length( 0 ) {}
    virtual ~Communicator() {}

    /**
     * @brief Resets the communicator
     * @details Force a reset of the communicator. The effect is the same as closing the communicator
     * and reopening it (note, there is no open method. That occurs on constuction).
     */
    virtual void reset( void ) = 0;

    /**
     * @brief read from the data server
     * @details Read a stream of data from the server. The length is in message_length and the
     * message can be retrieved from data.
     */
    virtual void read( void ) = 0;

    /**
     * @brief write some data to the server
     * @details Writes `msg' to the server.
     */
    virtual void write( std::string msg ) = 0;

    /**
     * @brief Close the link to the data server
     */
    virtual void close( void ) = 0;

    /**
     * @brief The data from the last message
     *
     * @return string - The data from the last read
     */
    char* data( void ) { return buf.data(); }

    /**
     * @brief The size of the last message
     *
     * @return size_t - The size of the data from the last read
     */
    size_t message_length( void ) { return reply_length; }

    bool success;
    boost::asio::io_service io_service;

  protected:
    size_t reply_length;
    boost::array<char, 128> buf;
    boost::system::error_code error;
};

/**
 * @brief A Communicator used for testing
 * @details This communicator is used only for testing. As the name implies, it does nothing other
 * than implemt the pure virtual base members as empty functions.
 */
class VoidCommunicator: public Communicator
{
  public:

    VoidCommunicator( std::string host = "localhost", std::string port = "22222", int retries = 2 ):
        Communicator( host, port, retries ) {}
    virtual ~VoidCommunicator() {}

    void reset( void ) {}
    void read( void ) {}
    void write( std::string msg ) {}
    void close( void ) {}

};

/**
 * @brief Handles communication over a TCP socket.
 * @details This class creates a communicator that listens for data on a user-specifed
 * TCP port from a user-specifed host.
 */
class TCPCommunicator: public Communicator
{
  public:

    /**
     * @brief Constructor
     * @details Creates a TCP socket listening for data from host\@port
     *
     * @param host name of the host where the server socket resides (default is `localhost')
     * @param port TCP port of the server socket (default is `22222')
     * @param retries try `retries' times to connect to the server (default is `2')
     */
    TCPCommunicator( std::string host = "localhost", std::string port = "22222", int retries = 2 );

    virtual ~TCPCommunicator() { socket.close(); }

    void reset( void ) { socket.connect( *endpoint_iterator ); }
    void read( void );
    void write( std::string msg );

    void close( void ) { socket.close(); }

  private:
    boost::asio::ip::tcp::socket socket;
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator;

};

/**
 * @brief Handles communication over a UDP socket.
 * @details This class creates a communicator that listens for data on a user-specifed
 * UDP port from a user-specifed host.
 */
class UDPCommunicator: public Communicator
{
  public:

    /**
     * @brief Constructor
     * @details Creates a UDP socket listening for data from host\@port
     *
     * @param host name of the host where the server socket resides (default is `localhost')
     * @param port UDP port of the server socket (default is `22222')
     * @param retries try `retries' times to connect to the server (default is `2')
     */
    UDPCommunicator( std::string host = "localhost", std::string port = "22222", int retries = 2 );
    virtual ~UDPCommunicator() {}

    /**
     * @brief reset the socket
     * @details Empty implementation as there is no such thing as reseting a UDPsocket
     */
    void reset( void ) {}

    void read( void );
    void write( std::string msg ) {}

    void close( void ) {}

  private:
    boost::asio::ip::udp::socket socket;
    boost::asio::ip::udp::resolver::iterator endpoint_iterator;

};


#endif
