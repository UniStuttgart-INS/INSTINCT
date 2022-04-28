#pragma once

#include <cstdio>
#include <cassert>
#include <cmath>
#include <vector>
#include <algorithm>
#include <sstream>
#include <string>
#include <iostream>



// unnamed namespace only because the implementation is in this
// header file and we don't want to export symbols to the obj files


class CubicSpline
{
public:
    // boundary condition type for the spline end-points
    enum boundary_derivative_type {
        first_deriv = 1,
        second_deriv = 2,
        not_a_knot = 3
    };

protected:
    std::vector<double> vals_x,vals_y;            // x,y coordinates of points
    std::vector<double> coef_b,coef_c,coef_d;        // spline coefficients
    double coef_c0;                            // for left extrapolation
    boundary_derivative_type boundary_type_left, boundary_type_right;
    double  boundary_value_left, boundary_value_right;
    bool made_monotonic;
    void set_coeffs_frocoef_b();               // calculate c_i, d_i from b_i
    size_t find_closest(double x) const;    // closest idx so that vals_x[idx]<=x

public:
    CubicSpline(): boundary_type_left(second_deriv), boundary_type_right(second_deriv),
        boundary_value_left(0.0), boundary_value_right(0.0)
    {
        ;
    }
    CubicSpline(const std::vector<double>& X, const std::vector<double>& Y,
           boundary_derivative_type left  = second_deriv, double left_value  = 0.0,
           boundary_derivative_type right = second_deriv, double right_value = 0.0
          ):
        boundary_type_left(left), boundary_type_right(right),
        boundary_value_left(left_value), boundary_value_right(right_value)
    {
        this->set_points(X,Y);

    }


    void set_boundary(boundary_derivative_type left, double left_value,
                      boundary_derivative_type right, double right_value);

    void set_points(const std::vector<double>& x,
                    const std::vector<double>& y);

    double operator() (double x) const;
    double deriv(int order, double x) const;
	
private:
	class BandMatrix
	{
	private:
	    std::vector< std::vector<double> > upper_band;  // upper band
	    std::vector< std::vector<double> > lower_band;  // lower band
	public:
	    BandMatrix() {};                             // constructor
	    BandMatrix(int dim, int n_u, int n_l);       // constructor
	    ~BandMatrix() {};                            // destructor
	    void resize(int dim, int n_u, int n_l);      // init with dim,n_u,n_l
	    int dim() const;                             // matrix dimension
	    int dim_upper_band() const
	    {
	        return (int)upper_band.size()-1;
	    }
	    int dim_lower_band() const
	    {
	        return (int)lower_band.size()-1;
	    }
	    // access operator
	    double & operator () (int i, int j);            // write
	    double   operator () (int i, int j) const;      // read
	    // we can store an additional diagonal (in lower_band)
	    double& saved_diag(int i);
	    double  saved_diag(int i) const;
	    void lu_decompose();
	    std::vector<double> r_solve(const std::vector<double>& b) const;
	    std::vector<double> l_solve(const std::vector<double>& b) const;
	    std::vector<double> lu_solve(const std::vector<double>& b,
	                                 bool is_lu_decomposed=false);					 
	};
};
