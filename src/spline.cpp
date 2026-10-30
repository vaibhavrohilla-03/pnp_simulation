#include "spline.hpp"
#include <iostream>
#include <algorithm>
Spline::Spline(std::vector<double>& points_x, std::vector<double>& points_y, int samples) : 
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
    
    std::vector<double> dense_x;
    dense_x.reserve(samples);

    double start = _points_x.front();
    double end   = _points_x.back();
    double step  = (end - start) / static_cast<double>(samples - 1);

    for (size_t i = 0; i < samples; ++i) {
        dense_x.push_back(start + i * step);
    }

    spline = solve(dense_x);

}


std::vector<double> Spline::solve_tridiagonal(std::vector<double>& u, std::vector<double>& lambda, std::vector<double> d) {
    
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

const double Spline::evaluate(double sample) {
    
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

const std::vector<double> Spline::solve(const std::vector<double>& query_x) {

        std::vector<double> results;
        results.reserve(query_x.size());

        for (double qx : query_x) {
            results.push_back(evaluate(qx));
        }

        return results;
    }