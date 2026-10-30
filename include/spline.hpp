#pragma once 

#include <vector>

class Spline {

    public:
        Spline(std::vector<double>& points_x, std::vector<double>& points_y, int samples = 200);
        
        std::vector<double> spline;
        
        const double evaluate(double sample);
    private:
        std::vector<double> _points_x;
        std::vector<double> _points_y;
        std::vector<double> _M;

        std::vector<double> solve_tridiagonal(std::vector<double>& u, std::vector<double>& lambda, std::vector<double> d);


        const std::vector<double> solve(const std::vector<double>& query_x);
};

struct Trajectory3D {

    std::vector<double> timestamps;
    Spline spline_x;
    Spline spline_y;
    Spline spline_z;

};