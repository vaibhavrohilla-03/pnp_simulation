#pragma once 

#include <vector>

class Trajectory3D {

    public: 

        Trajectory3D(const std::vector<double> timestamps, const std::vector<double>& waypoints, int sample = 100);
        void solve(const std::vector<double>& waypoints);
        
        void evaluate_position(double t, double out[3]) const;
        void evaluate_velocity(double t, double out[3]) const;
        void evaluate_acceleration(double t, double out[3]) const;

        double start_time() const { return _timestamps.front(); }
        double end_time()   const { return _timestamps.back(); }
        bool is_finished(double t) const { return t >= _timestamps.back(); }
        
        std::vector<double> dense_path;

    private:
        class Spline {
            
            public:
            Spline() = default;
            Spline(const std::vector<double>& points_x, const std::vector<double>& points_y);
            
            const double evaluate(double sample) const;
            const double evaluate_velocity(double sample) const;
            const double evaluate_acceleration(double sample) const;
            
            const std::vector<double> dense_evaluate(const std::vector<double>& query_x);
            
            private:
            std::vector<double> _points_x;
            std::vector<double> _points_y;
            std::vector<double> _M;
            
            std::vector<double> solve_tridiagonal(std::vector<double>& u, std::vector<double>& lambda, std::vector<double> d);
    
        };
        
        std::vector<double> _waypoints;
        const std::vector<double> _timestamps;
        int _sample;
        Spline spline_x;
        Spline spline_y;
        Spline spline_z;
};