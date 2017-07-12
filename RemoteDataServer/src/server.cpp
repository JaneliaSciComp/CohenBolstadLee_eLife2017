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
#include <iostream>

#include <cmath>
#include <cstdlib>
#include <ctime>

#ifdef _WINDOWS
#include <conio.h>
//#include <windows.h>
#endif

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

boost::random::mt19937 gen;

#define ACCUMULATE_FIRST

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/chrono.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/xtime.hpp>

using boost::asio::ip::tcp;
using boost::asio::ip::udp;

const int bytes_per_read = 12;

class data_interface
{
  public:
    data_interface( std::string file_name, int chunk_size, bool noisy ):
        _Vfwd( 0.0 ), _Vss( 0.0 ), _Omega( 0.0 ), _Vother( 0 ),
#ifdef _WINDOWS
        _comport( NULL ),
#endif
        _valid_port( false ), _add_turning( false ), _use_noise( noisy ),
        _packet_count( 0 ), _malformed( 0 ), _driving_from_file( false ), _read_next_line( true ),
        _driver_file ( file_name.c_str(), std::ifstream::in ), _chunk_size( chunk_size ),
        _bytes_per_chunk( chunk_size * bytes_per_read )
    {
        char* portName = getenv( "JOVIAN_INPUT_PORT" );
        if ( portName != NULL )
            set_port( std::string( portName ) );
    }

    bool set_port( std::string port_name )
    {
#ifdef _WINDOWS
        DWORD        bytes_read    = 0;    // Number of bytes read from port
        DWORD        bytes_written = 0;    // Number of bytes written to the port
        DCB          comSettings;          // Contains various port settings
        COMMTIMEOUTS CommTimeouts;

        if ( _comport == NULL || _comport == INVALID_HANDLE_VALUE )
        {
            _comport = CreateFile( TEXT( port_name.c_str() ),            // open portName:
                                   GENERIC_READ | GENERIC_WRITE, // for reading and writing
                                   0,                            // exclusive access
                                   NULL,                         // no security attributes
                                   OPEN_EXISTING,
                                   FILE_ATTRIBUTE_NORMAL,
                                   NULL );
            if ( !( _comport == NULL || _comport == INVALID_HANDLE_VALUE ) )
            {
                std::cout << "Found open com port " << port_name << std::endl;
            }
            else
                std::cout << "Failed opening com port " << port_name << std::endl;
        }

        // Open COM port
        if ( _comport != INVALID_HANDLE_VALUE )
        {
            // Set timeouts in milliseconds
            CommTimeouts.ReadIntervalTimeout         = 2000;
            CommTimeouts.ReadTotalTimeoutMultiplier  = 0;
            CommTimeouts.ReadTotalTimeoutConstant    = 100;
            CommTimeouts.WriteTotalTimeoutMultiplier = 2000;
            CommTimeouts.WriteTotalTimeoutConstant   = 100;
            _bStatus = SetCommTimeouts( _comport, &CommTimeouts );
            if ( _bStatus == 0 )
            {
                // error processing code goes here
            }
            // Set Port parameters.
            // Make a call to GetCommState() first in order to fill
            // the comSettings structure with all the necessary values.
            // Then change the ones you want and call SetCommState().
            GetCommState( _comport, &comSettings );
            comSettings.BaudRate = 1250000;
            comSettings.StopBits = ONESTOPBIT;
            comSettings.ByteSize = 8;
            comSettings.Parity   = NOPARITY;
            comSettings.fParity  = FALSE;
            _bStatus = SetCommState( _comport, &comSettings );
            if ( _bStatus == 0 )
            {
                // error processing code goes here
            }
            _outbuffer[0] = 254;
            _outbuffer[1] = 0;
            _bStatus = WriteFile( _comport,             // Handle
                                  &_outbuffer,      // Outgoing data
                                  2,              // Number of bytes to write
                                  &bytes_written,  // Number of bytes written
                                  NULL );
            if ( _bStatus == 0 )
            {
                // error processing code here
            }
            Sleep( 200 );

            _outbuffer[0] = 246;
            _outbuffer[1] = 1;
            _bStatus = WriteFile( _comport,             // Handle
                                  &_outbuffer,      // Outgoing data
                                  2,              // Number of bytes to write
                                  &bytes_written,  // Number of bytes written
                                  NULL );
            if ( _bStatus == 0 )
            {
                // error processing code here
            }
            Sleep( 200 );

            _outbuffer[0] = 255;
            _outbuffer[1] = 0;
            _bStatus = WriteFile( _comport,             // Handle
                                  &_outbuffer,      // Outgoing data
                                  2,              // Number of bytes to write
                                  &bytes_written,  // Number of bytes written
                                  NULL );
            if ( _bStatus == 0 )
            {
                // error processing code here
            }
            Sleep( 200 );
            _valid_port = true;
        }
#else
        _valid_port = true;
#endif

        return _valid_port;
    }

    ~data_interface()
    {
#ifdef _WINDOWS
        DWORD bytes_written = 0;    // Number of bytes written to the port

        _outbuffer[0] = 254;
        _outbuffer[1] = 0;
        _bStatus = WriteFile( _comport,             // Handle
                              &_outbuffer,      // Outgoing data
                              2,              // Number of bytes to write
                              &bytes_written,  // Number of bytes written
                              NULL );
        if ( _bStatus == 0 )
        {
            // error processing code here
        }
        Sleep( 20 );


        CloseHandle( _comport );
        printf( "%d out of %d packets malformed\n", _malformed, _packet_count );
#endif
    }

    void get( float* Vfwd, float* Vss, float* Omega, float* raw_vals )
    {
        boost::mutex::scoped_lock lock( m_mutex );
#ifdef ACCUMULATE_FIRST
        *Vfwd = ( _Vss + _Vother ) * 0.707107f;
        *Vss = ( _Vss - _Vother ) * 0.707107f;
        *Omega = ( _Vfwd + _Omega ) / 2.f ;
#else
        *Vfwd = _Vfwd;
        *Vss = _Vss;
        *Omega = _Omega;
#endif
        raw_vals[0] = _Vfwd;
        raw_vals[1] = _Vss;
        raw_vals[2] = _Omega;
        raw_vals[3] = _Vother;

        _Vfwd = 0;
        _Vss = 0;
        _Omega = 0;
        _Vother = 0;

        if ( _driving_from_file )
            _read_next_line = true;
    }

    // Rather than lock the mutex internally, the caller is responsible for
    // locking and unlocking. Needed so we can do the update via file
    void put( int const* dx, int const* dy )
    {
#ifdef ACCUMULATE_FIRST
        _Vfwd += dx[0];
        _Vss += dy[0];
        _Omega += dx[1];
        _Vother += dy[1];
#else
        _Vfwd += ( dy[0] + dy[1] ) * 0.707107f;
        _Vss += ( dy[0] - dy[1] ) * 0.707107f;
        _Omega += ( dx[0] + dx[1] ) / 2.f ;
        _Vother = 0.f;
#endif
    }

    void run()
    {
        while ( !_valid_port ) boost::this_thread::sleep( boost::posix_time::milliseconds( 10 ) );

        if ( _driver_file.good() )
        {
            _driving_from_file = true;

            std::cout << "Using chunk size of " << _chunk_size << " for file aggregation" << std::endl;

            int dx[2], dy[2];
            while ( !_driver_file.eof() )
            {
                // If compiled with optimization this gets removed and the
                // result is no motion
                if ( _read_next_line )
                {
                    boost::mutex::scoped_lock lock( m_mutex );
                    float x1, y1, x2, y2;
                    for ( size_t j = 0; j < _chunk_size && !_driver_file.eof(); j++ )
                    {
                        _driver_file >> x1 >> y1 >> x2 >> y2;
                        dx[0] = ( int )x1;
                        dy[0] = ( int )y1;
                        dx[1] = ( int )x2;
                        dy[1] = ( int )y2;

                        put( dx, dy );
                    }

                    float Vfwd, Vss, Omega;
#ifdef ACCUMULATE_FIRST
                    Vfwd = ( _Vss + _Vother ) * 0.707107f;
                    Vss = ( _Vss - _Vother ) * 0.707107f;
                    Omega = ( _Vfwd + _Omega ) / 2.f ;
#else
                    Vfwd = _Vfwd;
                    Vss = _Vss;
                    Omega = _Omega;
#endif
                    std::cout << Vfwd << "," << Vss << "," << Omega << std::endl;
                    std::cout.flush();

                    _read_next_line = false;
                }
            }
            std::cout << "Driving from file complete." << std::endl;
            _driver_file.close();
            boost::this_thread::sleep( boost::posix_time::milliseconds( 1000 ) );

            // After the one second sleep, we throw the system into a spin to
            // indicate that we've completed the read
            dx[0] = 1;
            dx[1] = 1;
            dy[0] = 0;
            dy[1] = 0;

            boost::random::uniform_int_distribution<> dist( -3, 3 );

            for ( ;; )
            {
                // If compiled with optimization this gets removed and the
                // result is no motion
                if ( ( fabs( _Vfwd ) < abs( dx[0] ) ) || ( fabs( _Omega ) < abs( dx[1] ) ) )
                {
                    if ( _use_noise )
                    {
                        do
                        {
                            dx[0] = dist( gen );
                            dx[1] = dist( gen );
                        }
                        while ( dx[0] == 0 && dx[1] == 0 ) ;
                    }
                    {
                        boost::mutex::scoped_lock lock( m_mutex );
                        put( dx, dy );
                    }

                    boost::this_thread::sleep( boost::posix_time::milliseconds( 1 ) );
                }
            }
        }

#ifdef _WINDOWS
        DWORD  bytes_read = 0;    // Number of bytes read from port
        int dx[2], dy[2], squal[2];
        float shutter[2];

        for ( int count = 0;; count++ )
        {
            bool valid = true;
            int index;

            // Read _chunk_size packets of data (bytes_per_read bytes per packet)
            _bStatus = ReadFile( _comport,  // Handle
                                 &_inbuffer,            // Incoming data
                                 _bytes_per_chunk,                  // Number of bytes to read
                                 &bytes_read,          // Number of bytes read
                                 NULL );
            if ( _bStatus == 0 || bytes_read != _bytes_per_chunk )
            {
                printf( "Bad Read. %d bytes read\n", bytes_read );
                Sleep( 1 );
                continue;
            }

            _packet_count++;
            for ( index = 0; valid && index < _bytes_per_chunk; index += bytes_per_read )
            {
                valid &= ( ( int )_inbuffer[index] == 0 );
            }

            if ( valid )
            {
                //Accumulate Motion Data for this 20Hz chunk
                dx[0] = 0; dx[1] = 0; dy[0] = 0; dy[1] = 0;
                for ( int i = 0; i < _bytes_per_chunk; i += bytes_per_read )
                {
                    dx[0] += ( ( int )_inbuffer[i + 2] ) - 128;
                    dy[0] += ( ( int )_inbuffer[i + 3] ) - 128;
                    dx[1] += ( ( int )_inbuffer[i + 4] ) - 128;
                    dy[1] += ( ( int )_inbuffer[i + 5] ) - 128;
                }

                {
                    boost::mutex::scoped_lock lock( m_mutex );
                    put( dx, dy );
                }

                squal[0] = ( int )_inbuffer[6];
                squal[1] = ( int )_inbuffer[7];
                shutter[0] = ( ( ( float )( _inbuffer[8] - 1 ) ) * 256.0 + ( float )_inbuffer[9] ) / 24.0;
                shutter[1] = ( ( ( float )( _inbuffer[10] - 1 ) ) * 256.0 + ( float )_inbuffer[11] ) / 24.0;

                if ( count % 100 == 0 )
                {
                    printf( "\tsqual0 = %d", squal[0] );
                    printf( "\tsqual1 = %d\n", squal[1] );
                    printf( "\tShutter0 = %.2fus", shutter[0] );
                    printf( "\tShutter1 = %.2fus\n", shutter[1] );
                }
            }
            else
            {
                DWORD bytes_written = 0;    // Number of bytes written to the port

                printf( "Malformed packet #%d\n", ++_malformed );
                _outbuffer[0] = 254;
                _outbuffer[1] = 0;
                _bStatus = WriteFile( _comport,             // Handle
                                      &_outbuffer,      // Outgoing data
                                      2,              // Number of bytes to write
                                      &bytes_written,  // Number of bytes written
                                      NULL );
                if ( _bStatus = 0 )
                {
                    // error processing code here
                }
                Sleep( 20 );

                _bStatus = ReadFile( _comport,  // Handle
                                     &_inbuffer,            // Incoming data
                                     bytes_per_read * 200,                // Number of bytes to read
                                     &bytes_read,          // Number of bytes read
                                     NULL );

                _outbuffer[0] = 246;
                _outbuffer[1] = 1;
                _bStatus = WriteFile( _comport,             // Handle
                                      &_outbuffer,      // Outgoing data
                                      2,              // Number of bytes to write
                                      &bytes_written,  // Number of bytes written
                                      NULL );
                if ( _bStatus == 0 )
                {
                    // error processing code here
                }
                Sleep( 20 );

                _outbuffer[0] = 255;
                _outbuffer[1] = 0;
                _bStatus = WriteFile( _comport,             // Handle
                                      &_outbuffer,      // Outgoing data
                                      2,              // Number of bytes to write
                                      &bytes_written,  // Number of bytes written
                                      NULL );
                if ( _bStatus = 0 )
                {
                    // error processing code here
                }
                Sleep( 20 );
            }
        }
#else
        int dx[2], dy[2];
        if ( _driving_from_file )
        {
            dx[0] = 1;
            dx[1] = 1;
            dy[0] = 0;
            dy[1] = 0;
        }
        else
        {
            if ( _add_turning )
            {
                dx[0] = 0;
                dx[1] = 0;
                dy[0] = -3;
                dy[1] = -1;
            }
            else
            {
                dx[0] = 0;
                dx[1] = 0;
                dy[0] = -1;
                dy[1] = -1;
            }
        }

        boost::random::uniform_int_distribution<> dist( -3, 1 );

        for ( ;; )
        {
            // If compiled with optimization this gets removed and the
            // result is no motion
            if ( ( _driving_from_file &&
                    ( ( fabs( _Vfwd ) < fabs( dx[0] ) ) ||
                      ( fabs( _Omega ) < fabs( dx[1] ) ) ) ) ||
                    ( !_driving_from_file &&
                      ( ( fabs( _Vss ) < fabs( dy[0] ) ) ||
                        ( fabs( _Vother ) < fabs( dy[1] ) ) ) ) )
            {
                if ( _use_noise )
                {
                    do
                    {
                        dy[0] = dist( gen );
                        dy[1] = dist( gen );
                    }
                    while ( dy[0] == 0 && dy[1] == 0 ) ;
                }

                {
                    boost::mutex::scoped_lock lock( m_mutex );
                    put( dx, dy );
                }

                boost::this_thread::sleep( boost::posix_time::milliseconds( 1 ) );
            }
        }
#endif
    }

    static void do_thread( void* param )
    {
        static_cast<data_interface*>( param )->run();
    }

  private:
    boost::mutex m_mutex;
    float _Vfwd, _Vss, _Omega, _Vother;
    unsigned char _inbuffer[2410];
    unsigned char _outbuffer[50];
#ifdef _WINDOWS
    HANDLE  _comport;
#endif
    bool _valid_port, _add_turning;
    bool _use_noise;
    int   _bStatus;
    long _packet_count;
    long _malformed;
    bool _driving_from_file, _read_next_line;
    std::ifstream _driver_file;
    int _chunk_size, _bytes_per_chunk;
};

class session
{
  public:
    session( boost::asio::io_service& io_service, data_interface* data )
        : socket_( io_service ), _data( data )
    {
    }

    tcp::socket& socket()
    {
        return socket_;
    }

    void start()
    {
        _time = boost::chrono::system_clock::now();
        socket_.async_read_some( boost::asio::buffer( _buffer, max_length ),
                                 boost::bind( &session::handle_read, this,
                                              boost::asio::placeholders::error,
                                              boost::asio::placeholders::bytes_transferred ) );
    }

    void handle_read( const boost::system::error_code& error,
                      size_t bytes_transferred )
    {
        if ( !error )
        {
            if ( bytes_transferred <= 1 )
            {
                float Vfwd, Vss, Omega;
                float raw_vals[4];

                _data->get( &Vfwd, &Vss, &Omega, raw_vals );
                std::stringstream sstr;
                // Compute time delta since last read
                boost::chrono::system_clock::time_point start = _time;
                _time = boost::chrono::system_clock::now();
                boost::chrono::duration<float> sec = _time - start;
                // Pack arc length and duration
                sstr << Vfwd << "," << Vss << "," << Omega << ",";
                for ( int i = 0; i < 4; i++ )
                    sstr << raw_vals[i] << ",";

                sstr << sec.count();
                std::string message_( sstr.str() );

                boost::asio::async_write( socket_,
                                          boost::asio::buffer( message_.c_str(), message_.size() + 1 ),
                                          boost::bind( &session::handle_write, this,
                                                       boost::asio::placeholders::error ) );
            }
            else
            {
                // If message has a length, assume it's a port name
                _data->set_port( _buffer );
                std::string message_( "Success" );

                boost::asio::async_write( socket_,
                                          boost::asio::buffer( message_.c_str(), message_.size() + 1 ),
                                          boost::bind( &session::handle_write, this,
                                                       boost::asio::placeholders::error ) );
            }
        }
        else
        {
            std::cout << "Error on read" << std::endl;
            std::cout.flush();
            delete this;
        }
    }

    void handle_write( const boost::system::error_code& error )
    {
        if ( !error )
        {
            socket_.async_read_some( boost::asio::buffer( _buffer, max_length ),
                                     boost::bind( &session::handle_read, this,
                                                  boost::asio::placeholders::error,
                                                  boost::asio::placeholders::bytes_transferred ) );
        }
        else
        {
            std::cout << "Error on write\n";
            delete this;
        }
    }

  private:
    tcp::socket socket_;
    enum { max_length = 1024 };
    char _buffer[max_length];
    data_interface* _data;
    boost::chrono::system_clock::time_point _time;
};

class tcp_server
{
  public:
    tcp_server( boost::asio::io_service& io_service, short port, data_interface* data )
        : io_service_( io_service ),
          acceptor_( io_service, tcp::endpoint( tcp::v4(), port ) ), _data( data )
    {
        session* new_session = new session( io_service_, data );
        acceptor_.async_accept( new_session->socket(),
                                boost::bind( &tcp_server::handle_accept, this, new_session,
                                             boost::asio::placeholders::error ) );
    }

    void handle_accept( session* new_session,
                        const boost::system::error_code& error )
    {
        if ( !error )
        {
            new_session->start();
            new_session = new session( io_service_, _data );
            acceptor_.async_accept( new_session->socket(),
                                    boost::bind( &tcp_server::handle_accept, this, new_session,
                                                 boost::asio::placeholders::error ) );
        }
        else
        {
            delete new_session;
        }
    }

  private:
    boost::asio::io_service& io_service_;
    tcp::acceptor acceptor_;
    data_interface* _data;
};

class udp_server
{
  public:
    udp_server( boost::asio::io_service& io_service, short port, data_interface* data )
        : io_service_( io_service ),
          socket_( io_service, udp::endpoint( udp::v4(), port ) ),
          _data( data )
    {
        socket_.async_receive_from(
            boost::asio::buffer( _buffer, max_length ), sender_endpoint_,
            boost::bind( &udp_server::handle_receive_from, this,
                         boost::asio::placeholders::error,
                         boost::asio::placeholders::bytes_transferred ) );
    }

    void handle_receive_from( const boost::system::error_code& error,
                              size_t bytes_recvd )
    {
        if ( !error && bytes_recvd > 0 )
        {
            float Vfwd, Vss, Omega;
            float raw_vals[4];

            _data->get( &Vfwd, &Vss, &Omega, raw_vals );
            std::stringstream sstr;
            sstr << Vfwd << "," << Vss << "," << Omega << ",";
            for ( int i = 0; i < 4; i++ )
                sstr << raw_vals[i] << ",";
            std::string message_ = sstr.str();

            socket_.async_send_to(
                boost::asio::buffer( message_ ), sender_endpoint_,
                boost::bind( &udp_server::handle_send_to, this,
                             boost::asio::placeholders::error,
                             boost::asio::placeholders::bytes_transferred ) );

        }
        else
        {
            socket_.async_receive_from(
                boost::asio::buffer( _buffer, max_length ), sender_endpoint_,
                boost::bind( &udp_server::handle_receive_from, this,
                             boost::asio::placeholders::error,
                             boost::asio::placeholders::bytes_transferred ) );
        }
    }

    void handle_send_to( const boost::system::error_code& error, size_t bytes_sent )
    {
        socket_.async_receive_from(
            boost::asio::buffer( _buffer, max_length ), sender_endpoint_,
            boost::bind( &udp_server::handle_receive_from, this,
                         boost::asio::placeholders::error,
                         boost::asio::placeholders::bytes_transferred ) );
    }

  private:
    boost::asio::io_service& io_service_;
    udp::socket socket_;
    udp::endpoint sender_endpoint_;
    data_interface* _data;
    enum { max_length = 1024 };
    char _buffer[max_length];
};

class thread_adapter
{
  public:
    thread_adapter( void ( *func )( void* ), void* param )
        : _func( func ), _param( param )
    {
    }
    void operator()() const { _func( _param ); }
  private:
    void ( *_func )( void* );
    void* _param;
};

int main( int argc, char* argv[] )
{
    int port = 22222;
    int chunk_size = 10;
    std::string file_name( "" );
    bool use_noise = false;

    po::options_description generic( "Generic options" );
    generic.add_options()
    ( "help", "produce help message" )
    ( "file,f", po::value<std::string>()->implicit_value( "" ),
      "replay a prerecorded trace from the treadmill" )
    ( "chunk,c", po::value<int>()->implicit_value( chunk_size ),
      "chunk size to use [default = 10]" )
    ( "port,p", po::value<int>()->implicit_value( port ),
      "which port to use [default = 22222]" )
    ( "noise,n", "add noise to autmatic driving" )
    ;

    po::options_description cmdline_options;
    cmdline_options.add( generic );

    po::options_description visible( "Allowed options" );
    visible.add( generic );

    po::variables_map vm;
    po::store( po::command_line_parser( argc, argv ).
               options( cmdline_options ).run(), vm );
    po::notify( vm );

    if ( vm.count( "help" ) )
    {
        std::cout << "Usage: " << argv[0] << " [options] <input_file>\n";
        std::cout << generic;
        return 0;
    }

    if ( vm.count( "file" ) )
    {
        file_name = vm[ "file" ].as<std::string>();
        std::cout << "Using file " << file_name << " for replay" << std::endl;
    }

    if ( vm.count( "chunk" ) )
    {
        chunk_size = vm[ "chunk" ].as<int>();
    }

    if ( vm.count( "port" ) )
    {
        port = vm[ "port" ].as<int>();
    }

    std::cout << "Using port " << port << " for communication" << std::endl;

    if ( vm.count( "noise" ) )
    {
        use_noise = true;
        std::cout << "Noise added to driving function\n";
    }

    try
    {
        boost::asio::io_service io_service;
        data_interface* data = new data_interface( file_name, chunk_size, use_noise );
        boost::thread thrd( thread_adapter( &data_interface::do_thread, data ) );
        tcp_server s1( io_service, port, data );
        udp_server s2( io_service, port, data );

        io_service.run();
        thrd.join();
        delete data;
    }
    catch ( std::exception& e )
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}
