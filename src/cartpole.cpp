#include "cartpole.hpp"

CartPole::CartPole(Params params):params_(params){
    slack_x1_cost_val.resize(params_.horizon,params_.slack_cost);
    slack_x2_cost_val.resize(params_.horizon,params_.slack_cost);
    slack_x3_cost_val.resize(params_.horizon,params_.slack_cost);
    slack_x4_cost_val.resize(params_.horizon,params_.slack_cost);

    // Variables
    x1.update_size(params_.horizon+1);
    x2.update_size(params_.horizon+1);
    x3.update_size(params_.horizon+1);
    x4.update_size(params_.horizon+1);
    u.update_size(params_.horizon);
    d_x2.update_size(params_.horizon);
    d_x4.update_size(params_.horizon);
    d_u.update_size(params_.horizon);

    slack_x1.update_size(params_.horizon);
    slack_x2.update_size(params_.horizon);
    slack_x3.update_size(params_.horizon);
    slack_x4.update_size(params_.horizon);

    slack_x1_abs.update_size(params_.horizon);
    slack_x2_abs.update_size(params_.horizon);
    slack_x3_abs.update_size(params_.horizon);
    slack_x4_abs.update_size(params_.horizon);

    // Parameters
    x_init.update_size(4);

    x2_bar.update_size(params_.horizon);
    x4_bar.update_size(params_.horizon);
    u_bar.update_size(params_.horizon);

    x2_trust_region.update_size(1);
    x4_trust_region.update_size(1);
    u_trust_region.update_size(1);

    p1_x2.update_size(params_.horizon);
    p1_x4.update_size(params_.horizon);
    p1_u.update_size(params_.horizon);
    p1_scalar.update_size(params_.horizon);

    p2_x2.update_size(params_.horizon);
    p2_x4.update_size(params_.horizon);
    p2_u.update_size(params_.horizon);
    p2_scalar.update_size(params_.horizon);

    slack_x1_cost.update_size(params_.horizon);
    slack_x2_cost.update_size(params_.horizon);
    slack_x3_cost.update_size(params_.horizon);
    slack_x4_cost.update_size(params_.horizon);

    // QP Setup
    qp_params.vars = {&x1,&x2,&x3,&x4,
                      &u,
                      &d_x2,&d_x4,&d_u,
                      &slack_x1,&slack_x2,&slack_x3,&slack_x4,
                      &slack_x1_abs,&slack_x2_abs,&slack_x3_abs,&slack_x4_abs};

    qp_params.params = {&x_init,
                        &x2_bar,&x4_bar,&u_bar,
                        &x2_trust_region,&x4_trust_region,&u_trust_region,
                        &p1_x2,&p1_x4,&p1_u,&p1_scalar,
                        &p2_x2,&p2_x4,&p2_u,&p2_scalar,
                        &slack_x1_cost,&slack_x2_cost,&slack_x3_cost,&slack_x4_cost};

    controller.update_qp_params(qp_params);
    problem_formulation();
}

void CartPole::problem_formulation(){
    // Initial conditions
    controller.add_constraint(x_init[0],x1[0],x_init[0]);
    controller.add_constraint(x_init[1],x2[0],x_init[1]);
    controller.add_constraint(x_init[2],x3[0],x_init[2]);
    controller.add_constraint(x_init[3],x4[0],x_init[3]);

    for (int i = 0; i < params_.horizon; i++){
        // Objective function
        controller.add_cost(params_.u_cost*u[i]*u[i]);
        controller.add_cost(slack_x1_cost[i]*slack_x1_abs[i]);
        controller.add_cost(slack_x2_cost[i]*slack_x2_abs[i]);
        controller.add_cost(slack_x3_cost[i]*slack_x3_abs[i]);
        controller.add_cost(slack_x4_cost[i]*slack_x4_abs[i]);

        // Dynamics constraints
        controller.add_constraint(0,-x1[i+1] + x1[i] + Ts*x3[i] + slack_x1[i] ,0); // slack_x1[i]
        controller.add_constraint(0,-x2[i+1] + x2[i] + Ts*x4[i] + slack_x2[i] ,0); // slack_x2[i]
        controller.add_constraint(-p1_scalar[i],-x3[i+1] + x3[i] + p1_x2[i]*d_x2[i] + p1_x4[i]*d_x4[i] + p1_u[i]*d_u[i] + slack_x3[i] ,-p1_scalar[i]); // slack_x3[i]
        controller.add_constraint(-p2_scalar[i],-x4[i+1] + x4[i] + p2_x2[i]*d_x2[i] + p2_x4[i]*d_x4[i] + p2_u[i]*d_u[i] + slack_x4[i] ,-p2_scalar[i]); // slack_x4[i]
    
        // Difference constraints
        controller.add_constraint(x2_bar[i],-d_x2[i] + x2[i],x2_bar[i]);
        controller.add_constraint(x4_bar[i],-d_x4[i] + x4[i],x4_bar[i]);
        controller.add_constraint(u_bar[i],-d_u[i] + u[i],u_bar[i]);

        // Trust region constraints
        controller.add_constraint(-x2_trust_region[0],d_x2[i],x2_trust_region[0]);
        controller.add_constraint(-x4_trust_region[0],d_x4[i],x4_trust_region[0]);
        controller.add_constraint(-u_trust_region[0],d_u[i],u_trust_region[0]);

        // Distance contraints
        controller.add_constraint(-5,x1[i],5);
        
        // Input constraints
        controller.add_constraint(params_.u_lb,u[i],params_.u_ub);

        // Slack constraints
        controller.add_constraint(0,slack_x1_abs[i] - slack_x1[i],QP::POS_INF);
        controller.add_constraint(0,slack_x1_abs[i] + slack_x1[i],QP::POS_INF);
        
        controller.add_constraint(0,slack_x2_abs[i] - slack_x2[i],QP::POS_INF);
        controller.add_constraint(0,slack_x2_abs[i] + slack_x2[i],QP::POS_INF);
        
        controller.add_constraint(0,slack_x3_abs[i] - slack_x3[i],QP::POS_INF);
        controller.add_constraint(0,slack_x3_abs[i] + slack_x3[i],QP::POS_INF);
        
        controller.add_constraint(0,slack_x4_abs[i] - slack_x4[i],QP::POS_INF);
        controller.add_constraint(0,slack_x4_abs[i] + slack_x4[i],QP::POS_INF);
    
    }
    // Terminal distance contraints
    controller.add_constraint(-5,x1[params_.horizon],5);

    // Terminal conditions
    controller.add_constraint(0,x1[params_.horizon],0);
    controller.add_constraint(0,x2[params_.horizon],0);
    controller.add_constraint(0,x3[params_.horizon],0);
    controller.add_constraint(0,x4[params_.horizon],0);

    // Formulate
    controller.get_settings()->verbose = 0;
    controller.get_settings()->polish = 0;
    controller.get_settings()->eps_prim_inf = 1e-3;
    controller.get_settings()->eps_dual_inf = 1e-3;
    controller.formulate();
}

c_float CartPole::merit_function(std::vector<c_float> x1_,std::vector<c_float> x2_,
                               std::vector<c_float> x3_,std::vector<c_float> x4_,
                               std::vector<c_float> u_){

    c_float cost = 0;
    c_float M = params_.M;
    c_float m = params_.m;
    c_float l = params_.l;

    for (int i = 0; i < params_.horizon; i++){
        cost += params_.u_cost*std::pow(u_[i],2);

        cost += slack_x1_cost_val[i]*std::abs(-x1_[i+1] + x1_[i] + Ts*x3_[i]);
        cost += slack_x2_cost_val[i]*std::abs(-x2_[i+1] + x2_[i] + Ts*x4_[i]);
        cost += slack_x3_cost_val[i]*std::abs(-x3_[i+1] + x3_[i] + (m*l*std::pow(x4_[i],2)*std::sin(x2_[i]) + u_[i] - m*g*std::cos(x2_[i])*std::sin(x2_[i]))*Ts/(M + m*std::pow(std::sin(x2_[i]),2)) );
        cost += slack_x4_cost_val[i]*std::abs(-x4_[i+1] + x4_[i] + (-m*l*std::pow(x4_[i],2)*std::sin(x2_[i])*std::cos(x2_[i]) - u_[i]*std::cos(x2_[i]) + (M+m)*g*std::sin(x2_[i]))*Ts/(l*(M + m*std::pow(std::sin(x2_[i]),2))) );
    }

    return cost;
}

CartPole::Problem_Params CartPole::parameter_generate(std::vector<c_float> x2_,std::vector<c_float> x4_, std::vector<c_float> u_){
    Problem_Params problem_params;
    c_float M = params_.M;
    c_float m = params_.m;
    c_float l = params_.l;

    problem_params.p1_x2_val.resize(params_.horizon);
    problem_params.p1_x4_val.resize(params_.horizon);
    problem_params.p1_u_val.resize(params_.horizon);
    problem_params.p1_scalar_val.resize(params_.horizon);

    problem_params.p2_x2_val.resize(params_.horizon);
    problem_params.p2_x4_val.resize(params_.horizon);
    problem_params.p2_u_val.resize(params_.horizon);
    problem_params.p2_scalar_val.resize(params_.horizon);

    problem_params.x2_bar_val.resize(params_.horizon);
    problem_params.x4_bar_val.resize(params_.horizon);
    problem_params.u_bar_val.resize(params_.horizon);

    for (int i = 0; i < params_.horizon; i++){
        // p1
        problem_params.p1_x2_val[i] = Ts*( (m*l*std::pow(x4_[i],2)*std::cos(x2_[i]) + m*g*std::pow(std::sin(x2_[i]),2) - m*g*std::pow(std::cos(x2_[i]),2))*(M + m*std::pow(std::sin(x2_[i]),2)) 
                                         - (m*l*std::pow(x4_[i],2)*std::sin(x2_[i]) + u_[i] - m*g*std::cos(x2_[i])*std::sin(x2_[i]))*2*m*std::sin(x2_[i])*std::cos(x2_[i]))/std::pow(M + m*std::pow(std::sin(x2_[i]),2),2);
    
        problem_params.p1_x4_val[i] = Ts*(2*m*l*x4_[i]*std::sin(x2_[i]))/(M + m*std::pow(std::sin(x2_[i]),2));

        problem_params.p1_u_val[i] = Ts*1/(M + m*std::pow(std::sin(x2_[i]),2));

        problem_params.p1_scalar_val[i] = Ts*(m*l*std::pow(x4_[i],2)*std::sin(x2_[i]) + u_[i] - m*g*std::cos(x2_[i])*std::sin(x2_[i]))/(M + m*std::pow(std::sin(x2_[i]),2));  // + u_[i]

        // p2
        problem_params.p2_x2_val[i] = Ts*( (-m*l*std::pow(x4_[i],2)*std::pow(std::cos(x2_[i]),2) + m*l*std::pow(x4_[i],2)*std::pow(std::sin(x2_[i]),2) + u_[i]*std::sin(x2_[i]) + (M+m)*g*std::cos(x2_[i]))
                                          *(l*M + l*m*std::pow(std::sin(x2_[i]),2)) 
                                          - (-m*l*std::pow(x4_[i],2)*std::sin(x2_[i])*std::cos(x2_[i]) - u_[i]*std::cos(x2_[i]) + (M+m)*g*std::sin(x2_[i]))
                                           *(2*l*m*std::sin(x2_[i])*std::cos(x2_[i])))/std::pow(l*M + l*m*std::pow(std::sin(x2_[i]),2),2);

        problem_params.p2_x4_val[i] = Ts*(-2*m*l*x4_[i]*std::sin(x2_[i])*std::cos(x2_[i]))/(l*M + l*m*std::pow(std::sin(x2_[i]),2));

        problem_params.p2_u_val[i] = Ts*(-std::cos(x2_[i]))/(l*M + l*m*std::pow(std::sin(x2_[i]),2));

        problem_params.p2_scalar_val[i] = Ts*(-m*l*std::pow(x4_[i],2)*std::sin(x2_[i])*std::cos(x2_[i]) - u_[i]*std::cos(x2_[i]) + (M+m)*g*std::sin(x2_[i]))/(l*M + l*m*std::pow(std::sin(x2_[i]),2)); // - u_[i]*std::cos(x2_[i])
    
        // bars
        problem_params.x2_bar_val[i] = x2_[i];
        problem_params.x4_bar_val[i] = x4_[i];
        problem_params.u_bar_val[i] = u_[i];

        // std::cout << problem_params.p1_scalar_val[i] << std::endl;
    }

    return problem_params;
}

CartPole::Solution CartPole::optimize(c_float x1_init,c_float x2_init,c_float x3_init,c_float x4_init){
    Solution solution;

    std::vector<c_float> x1_current, x2_current, x3_current, x4_current;
    std::vector<c_float> u_current,du_current;
    std::vector<c_float> slack_x1_current,slack_x2_current,slack_x3_current,slack_x4_current;
    
    Problem_Params problem_params;

    c_float cost_current,cost_new_predicted,cost_new_actual;
    c_float predicted_decrease,actual_decrease;
    c_float d_x2_mag,d_x4_mag,d_u_mag;
    c_float x2_trust_region_val,x4_trust_region_val,u_trust_region_val;

    c_int sqp_iter,slack_iter;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> normal_dist(0,10);

    // Initial guesses
    x1_current.resize(params_.horizon+1);
    x2_current.resize(params_.horizon+1);
    x3_current.resize(params_.horizon+1);
    x4_current.resize(params_.horizon+1);
    u_current.resize(params_.horizon,0);

    // x1_current[0] = x1_init;
    // x2_current[0] = x2_init;
    // x3_current[0] = x3_init;
    // x4_current[0] = x4_init;
    // x1_current[params_.horizon] = 0;
    // x2_current[params_.horizon] = 0;
    // x3_current[params_.horizon] = 0;
    // x4_current[params_.horizon] = 0;
    for (int i = 0; i < params_.horizon+1; i++){
        x1_current[i] = x1_init*(1. - static_cast<c_float>(i)/static_cast<c_float>(params_.horizon));
        x3_current[i] = x3_init*(1. - static_cast<c_float>(i)/static_cast<c_float>(params_.horizon));
        // x3_current[i] = -x1_init/(params_.horizon*Ts);

        x2_current[i] = x2_init*(1. - static_cast<c_float>(i)/static_cast<c_float>(params_.horizon));
        x4_current[i] = x4_init*(1. - static_cast<c_float>(i)/static_cast<c_float>(params_.horizon));
        // x4_current[i] = -x2_init/(params_.horizon*Ts);
    
        // if (i != params_.horizon) u_current[i] = normal_dist(gen);
    }


    // Initial conditions
    x_init.set_data({x1_init,x2_init,x3_init,x4_init});

    slack_iter = 0;
    for (;slack_iter < slack_iter_max; slack_iter++){
        x2_trust_region_val = x2_trust_region_init;
        x4_trust_region_val = x4_trust_region_init;
        u_trust_region_val = u_trust_region_init;

        slack_x1_cost.set_data(slack_x1_cost_val);
        slack_x2_cost.set_data(slack_x2_cost_val);
        slack_x3_cost.set_data(slack_x3_cost_val);
        slack_x4_cost.set_data(slack_x4_cost_val);

        cost_current = merit_function(x1_current,x2_current,x3_current,x4_current,u_current);

        sqp_iter = 0;
        for (;sqp_iter < sqp_iter_max;sqp_iter++){
            // Generate problem parameters
            problem_params = parameter_generate(x2_current,x4_current,u_current);
            x2_bar.set_data(problem_params.x2_bar_val);
            x4_bar.set_data(problem_params.x4_bar_val);
            u_bar.set_data(problem_params.u_bar_val);

            x2_trust_region.set_data({x2_trust_region_val});
            x4_trust_region.set_data({x4_trust_region_val});
            u_trust_region.set_data({u_trust_region_val});

            p1_x2.set_data(problem_params.p1_x2_val);
            p1_x4.set_data(problem_params.p1_x4_val);
            p1_u.set_data(problem_params.p1_u_val);
            p1_scalar.set_data(problem_params.p1_scalar_val);

            p2_x2.set_data(problem_params.p2_x2_val);
            p2_x4.set_data(problem_params.p2_x4_val);
            p2_u.set_data(problem_params.p2_u_val);
            p2_scalar.set_data(problem_params.p2_scalar_val);

            controller.update();
            controller.solve();

            // for (auto e : u.solution){
            //     std::cout << e << std::endl;
            // }

            std::cout << "status: " << controller.get_status() << " | ";

            // Check for convergence
            // d_x2_mag = 0;
            // d_x4_mag = 0;
            // d_u_mag = 0;

            // // std::cout << "x2_trust_region = " << x2_trust_region_val << " , x4_trust_region = " << x4_trust_region_val << " , u_trust_region = " << u_trust_region_val << " | ";

            // for (int i = 0; i < params_.horizon; i++){
            //     d_x2_mag += std::abs(d_x2.solution[i]);
            //     d_x4_mag += std::abs(d_x4.solution[i]);
            //     d_u_mag += std::abs(d_u.solution[i]);

            // }

            // if (d_x2_mag <= d_threshold && d_x4_mag <= d_threshold && d_u_mag <= d_threshold){
            //     break;
            // }

            if (x2_trust_region_val <= d_threshold && x4_trust_region_val <= d_threshold && u_trust_region_val <= d_threshold){
                break;
            }

            // Update
            cost_new_predicted = controller.get_obj_val();
            cost_new_actual = merit_function(x1.solution,x2.solution,x3.solution,x4.solution,u.solution);

            predicted_decrease = cost_current - cost_new_predicted;
            actual_decrease = cost_current - cost_new_actual;

            std::cout << "predicted_decrease = " << predicted_decrease << " , actual_decrease = " << actual_decrease << " | ";

            if (actual_decrease >= alpha*predicted_decrease && predicted_decrease >= 0){
                std::cout << "success\n";

                x2_trust_region_val *= beta_succ;
                x4_trust_region_val *= beta_succ;
                u_trust_region_val *= beta_succ;

                x1_current = x1.solution;
                x2_current = x2.solution;
                x3_current = x3.solution;
                x4_current = x4.solution;
                u_current = u.solution;
                du_current = d_u.solution;

                slack_x1_current = slack_x1.solution;
                slack_x2_current = slack_x2.solution;
                slack_x3_current = slack_x3.solution;
                slack_x4_current = slack_x4.solution;

                cost_current = cost_new_actual;
            }
            else{
                std::cout << "fail\n";
                x2_trust_region_val *= beta_fail;
                x4_trust_region_val *= beta_fail;
                u_trust_region_val *= beta_fail;
            }

            // std::cout << "----------------------\n";

        }

        std::cout << "current_cost = " << cost_current << std::endl;
        std::cout << "----------------------\n";

        // Subdue slack violation
        bool good = true;
        for (int i = 0; i < params_.horizon; i++){
            // x1
            if (std::abs(slack_x1_current[i]) >= slack_threshold){
                slack_x1_cost_val[i] *= slack_penalty;
                good = false;
            }

            // x2
            if (std::abs(slack_x2_current[i]) >= slack_threshold){
                slack_x2_cost_val[i] *= slack_penalty;
                good = false;
            }

            // x3
            if (std::abs(slack_x3_current[i]) >= slack_threshold){
                slack_x3_cost_val[i] *= slack_penalty;
                good = false;
            }

            // x4
            if (std::abs(slack_x4_current[i]) >= slack_threshold){
                slack_x4_cost_val[i] *= slack_penalty;
                good = false;
            }
        }

        if (good){
            std::cout << "converged!!!!\n";
        }
    }

    solution.x1 = x1_current;
    solution.x2 = x2_current;
    solution.x3 = x3_current;
    solution.x4 = x4_current;
    solution.u  = u_current;
    solution.du = du_current;

    solution.slack_x1 = slack_x1_current;
    solution.slack_x2 = slack_x2_current;
    solution.slack_x3 = slack_x3_current;
    solution.slack_x4 = slack_x4_current;

    return solution;
}