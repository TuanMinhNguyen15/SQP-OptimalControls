#pragma once

#include "osqp-interface/qp.hpp"
#include <vector>
#include <cmath>
#include <random>

class CartPole {
    public:
        struct Params{
            c_float m,M;
            c_float l;
            c_float u_lb,u_ub;

            c_int horizon = 100;
            c_float u_cost = 1;
            c_float slack_cost = 100;
        };

        struct Problem_Params{
            std::vector<c_float> p1_x2_val,p1_x4_val,p1_u_val,p1_scalar_val;
            std::vector<c_float> p2_x2_val,p2_x4_val,p2_u_val,p2_scalar_val;
            std::vector<c_float> x2_bar_val,x4_bar_val,u_bar_val;
        };

        struct Solution {
            std::vector<c_float> x1,x2,x3,x4;
            std::vector<c_float> u,du;
            std::vector<c_float> slack_x1,slack_x2,slack_x3,slack_x4;
        };

        CartPole(Params params);

        Solution optimize(c_float x1_int,c_float x2_init,c_float x3_init,c_float x4_init);

    private:
        // Config Variables
        c_float Ts = 0.01;
        c_int slack_iter_max = 10;
        c_int sqp_iter_max = 200;
        c_float d_threshold = 1e-3;
        c_float slack_threshold = 1e-2;

        c_float x2_trust_region_init = 5; // 5
        c_float x4_trust_region_init = 20; // 20
        c_float u_trust_region_init = 10;  // 10

        const c_float g = 9.81; // gravitational acceleration
        c_float alpha = 0.1;
        c_float beta_succ = 1.1;
        c_float beta_fail = 0.5;
        c_float slack_penalty = 1.5; // 1.5

        Params params_;
        QP::QP_Params qp_params;
        QP controller;

        // Optimzaion Variables
        QP::Variable x1,x2,x3,x4;
        QP::Variable u;
        QP::Variable d_x2,d_x4,d_u;

        QP::Variable slack_x1,slack_x2,slack_x3,slack_x4;
        QP::Variable slack_x1_abs,slack_x2_abs,slack_x3_abs,slack_x4_abs;

        // Optimization Parameters
        QP::Parameter x_init;
        QP::Parameter x2_bar,x4_bar,u_bar;
        QP::Parameter x2_trust_region,x4_trust_region,u_trust_region;
        QP::Parameter p1_x2,p1_x4,p1_u,p1_scalar;
        QP::Parameter p2_x2,p2_x4,p2_u,p2_scalar;
        QP::Parameter slack_x1_cost,slack_x2_cost,slack_x3_cost,slack_x4_cost;

        std::vector<c_float> slack_x1_cost_val,slack_x2_cost_val,slack_x3_cost_val,slack_x4_cost_val;

        // Utility Functions
        void problem_formulation();
        c_float merit_function(std::vector<c_float> x1_,std::vector<c_float> x2_,
                               std::vector<c_float> x3_,std::vector<c_float> x4_,
                               std::vector<c_float> u_);
        Problem_Params parameter_generate(std::vector<c_float> x2_,std::vector<c_float> x4_,std::vector<c_float> u_);
};