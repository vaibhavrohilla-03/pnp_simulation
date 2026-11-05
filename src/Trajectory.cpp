#include "Trajectory.hpp"
#include <iostream>
#include <algorithm>


Trajectory3D::Trajectory3D(const std::vector<double> timestamps, 
                               const std::vector<double>& waypoints, 
                               int sample) 
        : _waypoints(waypoints), _timestamps(timestamps), _sample(sample) 
    {
        solve(waypoints);
    }

void Trajectory3D::solve(const std::vector<double>& waypoints) {

        size_t num_points = _timestamps.size();

        // waypoints is a flat vector: {x0,y0,z0, x1,y1,z1, ...}
        // so its size must be exactly 3 * num_points
        if (waypoints.size() != num_points * 3) {
            std::cerr << "Trajectory3D :: waypoints size mismatch. "
                      << "Expected " << num_points * 3
                      << ", got " << waypoints.size() << "\n";
            return;
        }

        // Split the flat waypoints into 3 separate coordinate vectors
        std::vector<double> x_coords(num_points);
        std::vector<double> y_coords(num_points);
        std::vector<double> z_coords(num_points);

        for (size_t i = 0; i < num_points; ++i) {
            x_coords[i] = waypoints[i * 3 + 0];
            y_coords[i] = waypoints[i * 3 + 1];
            z_coords[i] = waypoints[i * 3 + 2];
        }

        // Build three independent 1D splines, all parameterized by _timestamps
        spline_x = Spline(_timestamps, x_coords);
        spline_y = Spline(_timestamps, y_coords);
        spline_z = Spline(_timestamps, z_coords);

        // Build dense path for visualization
        std::vector<double> dense_t;
        dense_t.reserve(_sample);

        double t_start = _timestamps.front();
        double t_end   = _timestamps.back();
        double step    = (t_end - t_start) / static_cast<double>(_sample - 1);

        for (int i = 0; i < _sample; ++i) {
            dense_t.push_back(t_start + i * step);
        }

        std::vector<double> dense_x = spline_x.dense_evaluate(dense_t);
        std::vector<double> dense_y = spline_y.dense_evaluate(dense_t);
        std::vector<double> dense_z = spline_z.dense_evaluate(dense_t);

        // Interleave into flat dense_path: {x0,y0,z0, x1,y1,z1, ...}
        dense_path.clear();
        dense_path.reserve(_sample * 3);
        for (int i = 0; i < _sample; ++i) {
            dense_path.push_back(dense_x[i]);
            dense_path.push_back(dense_y[i]);
            dense_path.push_back(dense_z[i]);
        }
    }

    void Trajectory3D::evaluate_position(double t, double out[3]) const {
        out[0] = spline_x.evaluate(t);
        out[1] = spline_y.evaluate(t);
        out[2] = spline_z.evaluate(t);
    }

    void Trajectory3D::evaluate_velocity(double t, double out[3]) const {
        out[0] = spline_x.evaluate_velocity(t);
        out[1] = spline_y.evaluate_velocity(t);
        out[2] = spline_z.evaluate_velocity(t);
    }

    void Trajectory3D::evaluate_acceleration(double t, double out[3]) const {
        out[0] = spline_x.evaluate_acceleration(t);
        out[1] = spline_y.evaluate_acceleration(t);
        out[2] = spline_z.evaluate_acceleration(t);
    }

Trajectory3D::Spline::Spline(const std::vector<double>& points_x, const std::vector<double>& points_y) : 
    _points_x(points_x), _points_y(points_y)  {

    const size_t size_x = _points_x.size();
    if(size_x < 3 || size_x != points_y.size()) {
        std::cerr << "Spline :: invalid points\n"; 
        return;
    }

    std::vector<double> intervals(size_x - 1);
    for(size_t i = 0; i < size_x - 1; ++i) {
        intervals[i] = _points_x[i+1] - _points_x[i];
    }

    std::vector<double> u;
    std::vector<double> lambda;
    std::vector<double> d;
    
    for (size_t i = 1; i < size_x - 1; ++i) {
        double h_left  = intervals[i - 1]; 
        double h_right = intervals[i];
        double h_sum   = h_left + h_right;
        
        
        double mu_i = h_left / h_sum;
        u.push_back(mu_i);
        lambda.push_back(1.0 - mu_i);
        
        double slope_right = (_points_y[i + 1] - _points_y[i]) / h_right;
        double slope_left  = (_points_y[i] - _points_y[i - 1]) / h_left;
        
        d.push_back((6.0 / h_sum) * (slope_right - slope_left));   
    }

    _M = solve_tridiagonal(u,lambda, d);

        if(_M.empty()) {
            std::cerr << "Spline :: unsolved\n"; 
            return;
        }
    
    // std::vector<double> dense_x;
    // dense_x.reserve(samples);

    // double start = _points_x.front();
    // double end   = _points_x.back();
    // double step  = (end - start) / static_cast<double>(samples - 1);

    // for (size_t i = 0; i < samples; ++i) {
    //     dense_x.push_back(start + i * step);
    // }

    // spline = solve(dense_x);
}


std::vector<double> Trajectory3D::Spline::solve_tridiagonal(std::vector<double>& u, std::vector<double>& lambda, std::vector<double> d) {
    
    const size_t m = d.size();
    if (m == 0 || u.size() != m || lambda.size() != m) {
        std::cerr << "Spline :: invalid dimensions\n";
        return {};
    }

    std::vector<double> c_prime(m);
    std::vector<double> d_prime(m);

    double b_0 = 2.0;
    c_prime[0] = lambda[0] / 2.0;
    d_prime[0] = d[0] / 2.0;

    for (size_t i = 1; i < m; ++i) {
        double a_i = u[i];         
        double b_i = 2.0;          
        double denom = b_i - a_i * c_prime[i - 1];

        if (std::abs(denom) < 1e-12) {
            std::cerr << "Spline :: zero_div\n";
            return {};
        }

        c_prime[i] = lambda[i] / denom;
        d_prime[i] = (d[i] - a_i * d_prime[i - 1]) / denom;
    }

    std::vector<double> M(m + 2, 0.0);

    M[m] = d_prime[m - 1];
    for (int i = static_cast<int>(m) - 2; i >= 0; --i) {
        M[i + 1] = d_prime[i] - c_prime[i] * M[i + 2];
    }

    return M;
}

const double Trajectory3D::Spline::evaluate(double sample) const {
    
    const size_t n = _points_x.size() - 1; 

    if ( sample <= _points_x.front()) return _points_y.front();
        if (sample >= _points_x.back()) return _points_y.back();
        
        // Locate interval [x[i], x[i+1]] via binary search
        auto it = std::upper_bound(_points_x.begin(), _points_x.end(), sample);
        size_t i = std::distance(_points_x.begin(), it) - 1;
        if (i >= n) i = n - 1;
        
        double h_i = _points_x[i + 1] - _points_x[i];
        
        // Constants A and B derived from boundary conditions
        double A = (_points_y[i + 1] / h_i) - (_M[i + 1] * h_i / 6.0);
        double B = (_points_y[i] / h_i)     - (_M[i] * h_i / 6.0);
        
        double dx_left  = sample - _points_x[i];       
        double dx_right = _points_x[i + 1] - sample;   
        

        double term1 = (_M[i + 1] * dx_left * dx_left * dx_left) / (6.0 * h_i);
        double term2 = (_M[i] * dx_right * dx_right * dx_right) / (6.0 * h_i);
        double term3 = A * dx_left;
        double term4 = B * dx_right;
        
        return term1 + term2 + term3 + term4;
}


const double Trajectory3D::Spline::evaluate_velocity(double sample) const {
    
    const size_t n = _points_x.size() - 1;
    if (sample <= _points_x.front()) return 0.0;
    if (sample >= _points_x.back())  return 0.0;

    auto it = std::upper_bound(_points_x.begin(), _points_x.end(), sample);
    size_t i = std::distance(_points_x.begin(), it) - 1;
    if (i >= n) i = n - 1;
    
    double h_i = _points_x[i + 1] - _points_x[i];
    
    double A = (_points_y[i + 1] / h_i) - (_M[i + 1] * h_i / 6.0);
    double B = (_points_y[i] / h_i)     - (_M[i] * h_i / 6.0);
    
    double dx_left  = sample - _points_x[i];
    double dx_right = _points_x[i + 1] - sample;
    
    double term1 =  (_M[i + 1] * dx_left  * dx_left)  / (2.0 * h_i);
    double term2 = -(_M[i]     * dx_right * dx_right)  / (2.0 * h_i);
    
    return term1 + term2 + A - B;
}
    
    
const double Trajectory3D::Spline::evaluate_acceleration(double sample) const {
    const size_t n = _points_x.size() - 1;
    if (sample <= _points_x.front()) return 0.0;
    if (sample >= _points_x.back())  return 0.0;
    
    auto it = std::upper_bound(_points_x.begin(), _points_x.end(), sample);
    size_t i = std::distance(_points_x.begin(), it) - 1;
    if (i >= n) i = n - 1;
    
    double h_i = _points_x[i + 1] - _points_x[i];
    
    double dx_left  = sample - _points_x[i];
    double dx_right = _points_x[i + 1] - sample;
    
    double term1 = (_M[i + 1] * dx_left)  / h_i;
    double term2 = (_M[i]     * dx_right) / h_i;
    
    return term1 + term2;
    
}

const std::vector<double> Trajectory3D::Spline::dense_evaluate(const std::vector<double>& query_x) {

        std::vector<double> results;
        results.reserve(query_x.size());

        for (double qx : query_x) {
            results.push_back(evaluate(qx));
        }

        return results;
    }