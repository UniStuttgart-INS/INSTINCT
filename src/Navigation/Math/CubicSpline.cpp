#include "CubicSpline.hpp"

void CubicSpline::set_boundary(CubicSpline::boundary_derivative_type left, double left_value,
                               CubicSpline::boundary_derivative_type right, double right_value)
{
    boundary_type_left = left;
    boundary_type_right = right;
    boundary_value_left = left_value;
    boundary_value_right = right_value;
}

void CubicSpline::set_coeffs_frocoef_b()
{
    size_t n = coef_b.size();
    if (coef_c.size() != n)
        coef_c.resize(n);
    if (coef_d.size() != n)
        coef_d.resize(n);

    for (size_t i = 0; i < n - 1; i++)
    {
        const double h = vals_x[i + 1] - vals_x[i];
        // from continuity and differentiability condition
        coef_c[i] = (3.0 * (vals_y[i + 1] - vals_y[i]) / h - (2.0 * coef_b[i] + coef_b[i + 1])) / h;
        // from differentiability condition
        coef_d[i] = ((coef_b[i + 1] - coef_b[i]) / (3.0 * h) - 2.0 / 3.0 * coef_c[i]) / h;
    }

    // for left extrapolation coefficients
    coef_c0 = (boundary_type_left == first_deriv) ? 0.0 : coef_c[0];
}

void CubicSpline::set_points(const std::vector<double>& x,
                             const std::vector<double>& y)
{
    made_monotonic = false;
    vals_x = x;
    vals_y = y;
    int n = (int)x.size();

    // classical cubic splines which are C^2 (twice cont differentiable)
    // this requires solving an equation system

    // setting up the matrix and right hand side of the equation system
    // for the parameters b[]
    int n_upper = (boundary_type_left == CubicSpline::not_a_knot) ? 2 : 1;
    int n_lower = (boundary_type_right == CubicSpline::not_a_knot) ? 2 : 1;
    BandMatrix A(n, n_upper, n_lower);
    std::vector<double> rhs(n);
    for (int i = 1; i < n - 1; i++)
    {
        A(i, i - 1) = 1.0 / 3.0 * (x[i] - x[i - 1]);
        A(i, i) = 2.0 / 3.0 * (x[i + 1] - x[i - 1]);
        A(i, i + 1) = 1.0 / 3.0 * (x[i + 1] - x[i]);
        rhs[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]) - (y[i] - y[i - 1]) / (x[i] - x[i - 1]);
    }
    // boundary conditions
    if (boundary_type_left == CubicSpline::second_deriv)
    {
        // 2*c[0] = f''
        A(0, 0) = 2.0;
        A(0, 1) = 0.0;
        rhs[0] = boundary_value_left;
    }
    else if (boundary_type_left == CubicSpline::first_deriv)
    {
        // b[0] = f', needs to be re-expressed in terms of c:
        // (2c[0]+c[1])(x[1]-x[0]) = 3 ((y[1]-y[0])/(x[1]-x[0]) - f')
        A(0, 0) = 2.0 * (x[1] - x[0]);
        A(0, 1) = 1.0 * (x[1] - x[0]);
        rhs[0] = 3.0 * ((y[1] - y[0]) / (x[1] - x[0]) - boundary_value_left);
    }
    else if (boundary_type_left == CubicSpline::not_a_knot)
    {
        // f'''(x[1]) exists, i.e. d[0]=d[1], or re-expressed in c:
        // -h1*c[0] + (h0+h1)*c[1] - h0*c[2] = 0
        A(0, 0) = -(x[2] - x[1]);
        A(0, 1) = x[2] - x[0];
        A(0, 2) = -(x[1] - x[0]);
        rhs[0] = 0.0;
    }
    if (boundary_type_right == CubicSpline::second_deriv)
    {
        // 2*c[n-1] = f''
        A(n - 1, n - 1) = 2.0;
        A(n - 1, n - 2) = 0.0;
        rhs[n - 1] = boundary_value_right;
    }
    else if (boundary_type_right == CubicSpline::first_deriv)
    {
        // b[n-1] = f', needs to be re-expressed in terms of c:
        // (c[n-2]+2c[n-1])(x[n-1]-x[n-2])
        // = 3 (f' - (y[n-1]-y[n-2])/(x[n-1]-x[n-2]))
        A(n - 1, n - 1) = 2.0 * (x[n - 1] - x[n - 2]);
        A(n - 1, n - 2) = 1.0 * (x[n - 1] - x[n - 2]);
        rhs[n - 1] = 3.0 * (boundary_value_right - (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2]));
    }
    else if (boundary_type_right == CubicSpline::not_a_knot)
    {
        // f'''(x[n-2]) exists, i.e. d[n-3]=d[n-2], or re-expressed in c:
        // -h_{n-2}*c[n-3] + (h_{n-3}+h_{n-2})*c[n-2] - h_{n-3}*c[n-1] = 0
        A(n - 1, n - 3) = -(x[n - 1] - x[n - 2]);
        A(n - 1, n - 2) = x[n - 1] - x[n - 3];
        A(n - 1, n - 1) = -(x[n - 2] - x[n - 3]);
        rhs[0] = 0.0;
    }

    // solve the equation system to obtain the parameters c[]
    coef_c = A.lu_solve(rhs);

    // calculate parameters b[] and d[] based on c[]
    coef_d.resize(n);
    coef_b.resize(n);
    for (int i = 0; i < n - 1; i++)
    {
        coef_d[i] = 1.0 / 3.0 * (coef_c[i + 1] - coef_c[i]) / (x[i + 1] - x[i]);
        coef_b[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i])
                    - 1.0 / 3.0 * (2.0 * coef_c[i] + coef_c[i + 1]) * (x[i + 1] - x[i]);
    }
    // for the right extrapolation coefficients (zero cubic term)
    // f_{n-1}(x) = y_{n-1} + b*(x-x_{n-1}) + c*(x-x_{n-1})^2
    double h = x[n - 1] - x[n - 2];
    // coef_c[n-1] is determined by the boundary condition
    coef_d[n - 1] = 0.0;
    coef_b[n - 1] = 3.0 * coef_d[n - 2] * h * h + 2.0 * coef_c[n - 2] * h + coef_b[n - 2]; // = f'_{n-2}(x_{n-1})
    if (boundary_type_right == first_deriv)
        coef_c[n - 1] = 0.0; // force linear extrapolation

    // for left extrapolation coefficients
    coef_c0 = (boundary_type_left == first_deriv) ? 0.0 : coef_c[0];
}

// return the closest idx so that vals_x[idx] <= x (return 0 if x<vals_x[0])
size_t CubicSpline::find_closest(double x) const
{
    std::vector<double>::const_iterator it;
    it = std::upper_bound(vals_x.begin(), vals_x.end(), x); // *it > x
    size_t idx = std::max(int(it - vals_x.begin()) - 1, 0); // vals_x[idx] <= x
    return idx;
}

double CubicSpline::operator()(double x) const
{
    size_t n = vals_x.size();
    size_t idx = find_closest(x);

    double h = x - vals_x[idx];
    double interpol;
    if (x < vals_x[0])
    {
        // extrapolation to the left
        interpol = (coef_c0 * h + coef_b[0]) * h + vals_y[0];
    }
    else if (x > vals_x[n - 1])
    {
        // extrapolation to the right
        interpol = (coef_c[n - 1] * h + coef_b[n - 1]) * h + vals_y[n - 1];
    }
    else
    {
        // interpolation
        interpol = ((coef_d[idx] * h + coef_c[idx]) * h + coef_b[idx]) * h + vals_y[idx];
    }
    return interpol;
}

double CubicSpline::deriv(int order, double x) const
{
    size_t n = vals_x.size();
    size_t idx = find_closest(x);

    double h = x - vals_x[idx];
    double interpol;
    if (x < vals_x[0])
    {
        // extrapolation to the left
        switch (order)
        {
        case 1:
            interpol = 2.0 * coef_c0 * h + coef_b[0];
            break;
        case 2:
            interpol = 2.0 * coef_c0;
            break;
        default:
            interpol = 0.0;
            break;
        }
    }
    else if (x > vals_x[n - 1])
    {
        // extrapolation to the right
        switch (order)
        {
        case 1:
            interpol = 2.0 * coef_c[n - 1] * h + coef_b[n - 1];
            break;
        case 2:
            interpol = 2.0 * coef_c[n - 1];
            break;
        default:
            interpol = 0.0;
            break;
        }
    }
    else
    {
        // interpolation
        switch (order)
        {
        case 1:
            interpol = (3.0 * coef_d[idx] * h + 2.0 * coef_c[idx]) * h + coef_b[idx];
            break;
        case 2:
            interpol = 6.0 * coef_d[idx] * h + 2.0 * coef_c[idx];
            break;
        case 3:
            interpol = 6.0 * coef_d[idx];
            break;
        default:
            interpol = 0.0;
            break;
        }
    }
    return interpol;
}

CubicSpline::BandMatrix::BandMatrix(int dim, int n_u, int n_l)
{
    resize(dim, n_u, n_l);
}
void CubicSpline::BandMatrix::resize(int dim, int n_u, int n_l)
{
    upper_band.resize(n_u + 1);
    lower_band.resize(n_l + 1);
    for (size_t i = 0; i < upper_band.size(); i++)
    {
        upper_band[i].resize(dim);
    }
    for (size_t i = 0; i < lower_band.size(); i++)
    {
        lower_band[i].resize(dim);
    }
}
int CubicSpline::BandMatrix::dim() const
{
    if (upper_band.size() > 0)
    {
        return upper_band[0].size();
    }
    else
    {
        return 0;
    }
}

// defines the new operator (), so that we can access the elements
// by A(i,j), index going from i=0,...,dim()-1
double& CubicSpline::BandMatrix::operator()(int i, int j)
{
    int k = j - i; // what band is the entry
    // k=0 -> diagonal, k<0 lower left part, k>0 upper right part
    if (k >= 0)
        return upper_band[k][i];
    else
        return lower_band[-k][i];
}
double CubicSpline::BandMatrix::operator()(int i, int j) const
{
    int k = j - i; // what band is the entry
    // k=0 -> diagonal, k<0 lower left part, k>0 upper right part
    if (k >= 0)
        return upper_band[k][i];
    else
        return lower_band[-k][i];
}
// second diag (used in LU decomposition), saved in lower_band
double CubicSpline::BandMatrix::saved_diag(int i) const
{
    return lower_band[0][i];
}
double& CubicSpline::BandMatrix::saved_diag(int i)
{
    return lower_band[0][i];
}

// LR-Decomposition of a band matrix
void CubicSpline::BandMatrix::lu_decompose()
{
    int i_max, j_max;
    int j_min;
    double x;

    // preconditioning
    // normalize column i so that a_ii=1
    for (int i = 0; i < dim(); i++)
    {
        assert(this->operator()(i, i) != 0.0);
        this->saved_diag(i) = 1.0 / this->operator()(i, i);
        j_min = std::max(0, i - dim_lower_band());
        j_max = std::min(dim() - 1, i + dim_upper_band());
        for (int j = j_min; j <= j_max; j++)
        {
            this->operator()(i, j) *= saved_diag(i);
        }
        this->operator()(i, i) = 1.0; // prevents rounding errors
    }

    // Gauss LR-Decomposition
    for (int k = 0; k < dim(); k++)
    {
        i_max = std::min(dim() - 1, k + dim_lower_band()); // nulower_band not a mistake!
        for (int i = k + 1; i <= i_max; i++)
        {
            assert(this->operator()(k, k) != 0.0);
            x = -this->operator()(i, k) / this->operator()(k, k);
            this->operator()(i, k) = -x; // assembly part of L
            j_max = std::min(dim() - 1, k + dim_upper_band());
            for (int j = k + 1; j <= j_max; j++)
            {
                // assembly part of R
                this->operator()(i, j) = this->operator()(i, j) + x * this->operator()(k, j);
            }
        }
    }
}
// solves Ly=b
std::vector<double> CubicSpline::BandMatrix::l_solve(const std::vector<double>& b) const
{
    assert(dim() == (int)b.size());
    std::vector<double> x(dim());
    int j_start;
    double sum;
    for (int i = 0; i < dim(); i++)
    {
        sum = 0;
        j_start = std::max(0, i - dim_lower_band());
        for (int j = j_start; j < i; j++) sum += this->operator()(i, j) * x[j];
        x[i] = (b[i] * this->saved_diag(i)) - sum;
    }
    return x;
}

std::vector<double> CubicSpline::BandMatrix::r_solve(const std::vector<double>& b) const
{
    assert(dim() == (int)b.size());
    std::vector<double> x(dim());
    int j_stop;
    double sum;
    for (int i = dim() - 1; i >= 0; i--)
    {
        sum = 0;
        j_stop = std::min(dim() - 1, i + dim_upper_band());
        for (int j = i + 1; j <= j_stop; j++) sum += this->operator()(i, j) * x[j];
        x[i] = (b[i] - sum) / this->operator()(i, i);
    }
    return x;
}

std::vector<double> CubicSpline::BandMatrix::lu_solve(const std::vector<double>& b,
                                                      bool is_lu_decomposed)
{
    assert(dim() == (int)b.size());
    std::vector<double> x, y;
    if (is_lu_decomposed == false)
    {
        lu_decompose();
    }
    y = l_solve(b);
    x = r_solve(y);
    return x;
}
