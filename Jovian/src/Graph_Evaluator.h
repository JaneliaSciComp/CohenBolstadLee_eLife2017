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

#ifndef GRAPH_EVALUATOR
#define GRAPH_EVALUATOR

#include <vector>

/**
 * @brief Graph_Evaluator.
 * @details Given a set of
 */

class Graph_Evaluator
{
	friend std::ostream& operator<<( std::ostream& os, Graph_Evaluator const& f );

public:

	/// @name Initialization
	///@{
	Graph_Evaluator() {}
	Graph_Evaluator( std::vector< double > x_vals, std::vector< double > y_vals ):
		_plot_x( x_vals ), _plot_y( y_vals ) {}
	///@}

	/// @name Duplication
	///@{
	///@}

	/// @name Destruction
	///@{
	~Graph_Evaluator() {}
	///@}

	/// @name Access
	///@{
	std::vector< double > const& x_vals() const { return _plot_x; }
	std::vector< double > const& y_vals() const { return _plot_y; }

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
	double evaluate( double x )
	{
		double value = 0.;
		std::vector< double >::iterator lb, ub;
		ub = std::upper_bound( _plot_x.begin(), _plot_x.end(), std::abs( x ) );

		if ( ub == _plot_x.end() )
		{
			value = _plot_y.back();
		}
		else if ( ub == _plot_x.begin() )
		{
			value = _plot_y.front();
		}
		else
		{
			// x is in range [_plot_x.begin(), _plot_x.end()]
			lb = ub - 1;
			double weight = ( std::abs( x ) - *lb ) / ( *ub - *lb );
			value = weight * _plot_y[ lb - _plot_x.begin() ] + ( 1.f - weight ) * _plot_y[ ub - _plot_x.begin() ];
		}

		return value;
	}

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
	///@}

private:
	std::vector< double > _plot_x, _plot_y;

};	// end of class Graph_Evaluator

#endif // GRAPH_EVALUATOR

