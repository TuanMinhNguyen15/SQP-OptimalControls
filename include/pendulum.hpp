#pragma once

#include "osqp-interface/qp.hpp"
#include <vector>
#include <cmath>
#include <random>

class Pendulum {
    public:
        struct Params{
            c_float m; // mass
            c_float l; // length
            c_float torque_lb,torque_ub;

            c_float torque_cost = 1;
            c_float slack_cost = 100;
            c_int horizon = 20; 
            c_float Ts = 0.01; // Sampling time
            c_int max_iter = 100;
            c_float threshold = 1e-3;
            c_float trust_region = 5;
        };

        struct Solution {
            std::vector<c_float> theta;
            std::vector<c_float> theta_dot;
            std::vector<c_float> torque;
            std::vector<c_float> slack_theta,slack_theta_dot;
            c_float cost;
            std::vector<c_float> slack_trials;
        };

        Pendulum(Params params);

        // Solution optimize(c_float theta0,c_float theta_dot0);
        Solution solve(c_float theta0_val,c_float theta0_dot_val);

    private:
        Params params_;

        std::vector<c_float> slack_theta_cost_val;
        std::vector<c_float> slack_theta_dot_cost_val;
        
        const c_float g = 9.81; // gravitational acceleration
        c_float alpha = 0.1;
        c_float beta_succ = 1.1;
        c_float beta_fail = 0.5;

        QP::QP_Params qp_params;
        QP controller;

        QP::Variable theta,theta_dot,torque;

        QP::Variable d_theta;
        QP::Variable slack_theta,slack_theta_abs;
        QP::Variable slack_theta_dot,slack_theta_dot_abs;
  
        QP::Parameter theta0,theta0_dot;
        QP::Parameter theta_bar;
        QP::Parameter p_theta,p_scalar;
        QP::Parameter trust_region;
        QP::Parameter slack_theta_cost,slack_theta_dot_cost;

        void controller_formulate();
        float merit_function(std::vector<c_float> theta_,std::vector<c_float> theta_dot_,std::vector<c_float> torque_);
        void parameter_generate(std::vector<c_float> theta_iter,
                                std::vector<c_float> &p_theta_val,
                                std::vector<c_float> &p_scalar_val);
  
};