/* -*-c++-*- */

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

#ifndef CONFIG_MEMENTO_H
#define CONFIG_MEMENTO_H

#include <fstream>
#include <vector>

#include <QtGui/QColor>

#include <Globals.h> // for from_string, triple, GraphPositions

/** @brief Holds parameters for GUI configuration.
 * @details A Memento pattern for saving and restoring the state of the GUI (Console).
 * There is no direct access to the variables that are stored. Only Console is allowed
 * access by being declared a friend.
 */

class Config_Memento
{
    friend std::ostream& operator<<( std::ostream& os, Config_Memento const& f );

  public:

    /// @name Initialization
    ///@{
    Config_Memento();
    Config_Memento( Console_Ptr console );
    ///@}

    /// @name Destruction
    ///@{
    ~Config_Memento() {}
    ///@}

    /// @name Access
    ///@{
    bool initialized() const { return _initialized; }
    int major_version() const { return _major_version; }
    int minor_version() const { return _minor_version; }

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
    void initialize( Console* console );

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
    void load( std::ifstream& config );
    void save( std::ofstream& config );

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
    void read( std::ifstream& config );
    ///@}

  private:
    friend class Console;

    int const _major_version;
    int const _minor_version;
    bool _initialized;
    int _minor, _major;
    int _numCameras, _num_displays, _starting_display;
    bool _gangCameras;
    bool _xCenterDistorts;
    std::vector< std::pair< int, int > > _frame;
    std::vector< triple< bool, int, int > > _distort;
    std::vector< std::pair< int, int > > _center;
    std::string _connection_type;
    int _retries, _port;
    std::string _host;
    bool _gangMotion;
    bool _frame_packing;
    std::vector< GraphPositions > _graph;
    QColor _ambient, _diffuse, _background;
    int _power;
    std::vector< QColor > _global_brightness;
    std::string _shader_choice;
    std::string _texture_resolution;

    bool _average_runs;
    float _calib_x, _calib_y, _calib_z;
    float _gain_x, _gain_y, _gain_z;
	int _start_location;
	bool _enable_vel_smoothing, _enable_input_smoothing;
    bool _enable_intermediary_smoothing, _enable_output_smoothing;
    int _vel_smoothing_interval, _input_interval, _intermediary_interval, _output_interval;
    float _ball_radius, _minimum_velocity_thresold, _crossbar_width;
    float _x_offset, _y_offset, _z_offset;
    bool _crossbar_enabled, _restrict_vertical_motion;
    std::string _output_port_name, _command_port_name, _data_server_port_name;
    int _output_rate;
    int _output_format_id;
    bool _treadmill_output, _reduced_output;
    bool _rotatedCameras;
    int _fov;
    float _fov_offset;
    int _frame_mode, _indicator_size, _border_size, _screen_select;
    int _indicator_horizontal_pos, _indicator_vertical_pos;
    bool _enable_threshold_turning;
    float _min_threshold_scale, _max_threshold_scale, _current_threshold_scale;
    float _auto_heading_turning_rate;
    std::vector< double > _plot_x, _plot_y;
    std::vector< double > _plot_vel_x, _plot_vel_y;

};  // end of class Config_Memento


#endif // CONFIG_MEMENTO_H

