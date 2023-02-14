#include "pendulum.hpp"

Pendulum::Pendulum(Params params):params_(params){
    slack_theta_cost_val.resize(params_.horizon,params_.slack_cost);
    slack_theta_dot_cost_val.resize(params_.horizon,params_.slack_cost);

    theta.update_size(params_.horizon+1);
    theta_dot.update_size(params_.horizon+1);
    torque.update_size(params_.horizon);

    d_theta.update_size(params_.horizon);

    slack_theta.update_size(params_.horizon);
    slack_theta_abs.update_size(params_.horizon);
    slack_theta_dot.update_size(params_.horizon);
    slack_theta_dot_abs.update_size(params_.horizon);

    theta0.update_size(1);
    theta0_dot.update_size(1);

    theta_bar.update_size(params_.horizon);

    p_theta.update_size(params_.horizon);
    p_scalar.update_size(params_.horizon);

    trust_region.update_size(1);

    slack_theta_cost.update_size(params_.horizon);
    slack_theta_dot_cost.update_size(params_.horizon);

    qp_params.vars = {&theta,&theta_dot,&torque,
                      &d_theta,
                      &slack_theta,&slack_theta_abs,
                      &slack_theta_dot,&slack_theta_dot_abs};

    qp_params.params = {&theta0,&theta0_dot,
                        &theta_bar,
                        &p_theta,&p_scalar,
                        &trust_region,
                        &slack_theta_cost,&slack_theta_dot_cost};

    controller.update_qp_params(qp_params);
    controller_formulate();
}

void Pendulum::controller_formulate(){
    // Initial condition constraints
    controller.add_constraint(theta0[0],theta[0],theta0[0]);
    controller.add_constraint(theta0_dot[0],theta_dot[0],theta0_dot[0]);

    for (int i = 0; i <= params_.horizon-1; i++){
        // Objective function
        controller.add_cost(params_.torque_cost*torque[i]*torque[i]);
        controller.add_cost(slack_theta_cost[i]*slack_theta_abs[i]);
        controller.add_cost(slack_theta_dot_cost[i]*slack_theta_dot_abs[i]);

        // Dynamics constraints
        controller.add_constraint(0,-theta[i+1] + theta[i] + params_.Ts*theta_dot[i] + slack_theta[i],0);
        controller.add_constraint(-p_scalar[i],-theta_dot[i+1] + theta_dot[i] + p_theta[i]*d_theta[i] + (params_.Ts/(params_.m*std::pow(params_.l,2)))*torque[i] + slack_theta_dot[i],-p_scalar[i]);
        
        // Difference constraints
        controller.add_constraint(theta_bar[i],-d_theta[i]+theta[i],theta_bar[i]);

        // Trust region constraints
        controller.add_constraint(-trust_region[0],d_theta[i],trust_region[0]);

        // Torque constraints
        controller.add_constraint(params_.torque_lb,torque[i],params_.torque_ub);
        
        // Slack constraints
        controller.add_constraint(0,slack_theta_abs[i]-slack_theta[i],QP::POS_INF);
        controller.add_constraint(0,slack_theta_abs[i]+slack_theta[i],QP::POS_INF);
        controller.add_constraint(0,slack_theta_dot_abs[i]-slack_theta_dot[i],QP::POS_INF);
        controller.add_constraint(0,slack_theta_dot_abs[i]+slack_theta_dot[i],QP::POS_INF);
    }

    // Terminal condition constraints
    controller.add_constraint(0,theta[params_.horizon],0);
    controller.add_constraint(0,theta_dot[params_.horizon],0);

    // Formulate
    controller.get_settings()->verbose = 0;
    controller.formulate();
    
}

float Pendulum::merit_function(std::vector<c_float> theta_,std::vector<c_float> theta_dot_,std::vector<c_float> torque_){
    float cost = 0;
  
    for (int i = 0; i < params_.horizon; i++){
        cost += params_.torque_cost*std::pow(torque_[i],2);
        cost += slack_theta_cost_val[i]*std::abs(-theta_[i+1] + theta_[i] + params_.Ts*theta_dot_[i]);
        cost += slack_theta_dot_cost_val[i]*std::abs(-theta_dot_[i+1] + theta_dot_[i] + params_.Ts*(g/params_.l)*std::sin(theta_[i]) + (params_.Ts/(params_.m*std::pow(params_.l,2)))*torque_[i]);
    }

    return cost;
}

void Pendulum::parameter_generate(std::vector<c_float> theta_,std::vector<c_float> &p_theta_val,std::vector<c_float> &p_scalar_val){
    for (int i = 0; i < params_.horizon; i++){
        p_theta_val[i] = params_.Ts*(g/params_.l)*std::cos(theta_[i]);
        p_scalar_val[i] = params_.Ts*(g/params_.l)*std::sin(theta_[i]);
    }
}

// Pendulum::Solution Pendulum::optimize(c_float theta0,c_float theta_dot0){
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::normal_distribution<float> normal_dist(0,1);
//     int num_trials = 10;

//     std::vector<c_float> theta_ideal,theta_guess;
//     theta_ideal.resize(params_.horizon);
//     theta_guess.resize(params_.horizon);

//     float diff = -theta0/(params_.horizon-1);
//     for (int i = 0; i < params_.horizon; i++){
//         theta_ideal[i] = theta0 + static_cast<float>(i)*diff;
//     }

//     c_float cost_best = QP::POS_INF;
//     Solution solution,solution_best;
//     for (int i = 0; i < num_trials; i++){
//         theta_guess = theta_ideal;
//         if (i != 0) {
//             for (int j = 1; j < params_.horizon; j++){
//                 theta_guess[j] += normal_dist(gen);
//             }
//         }

//         solution = solve(theta_guess,theta_dot0);
        
//         if (solution.cost < cost_best){
//             solution_best = solution;
//             cost_best = solution.cost;
//         }

//     }

//     return solution_best;
// }

Pendulum::Solution Pendulum::solve(c_float theta0_val,c_float theta0_dot_val){
    Solution solution;

    std::vector<c_float> theta_current,theta_dot_current,torque_current;
    std::vector<c_float> p_theta_val,p_scalar_val;
    std::vector<c_float> theta_bar_val;

    std::vector<c_float> slack_theta_current,slack_theta_dot_current;
    
    c_float cost_current,cost_new_predicted,cost_new_actual;
    c_float predicted_decrease,actual_decrease;
    c_float d_mag;
    c_float d_theta_mag,d_theta_dot_mag,d_torque_mag;
  
    uint8_t iter = 0;
    c_float trust_region_val = params_.trust_region;

    theta_current.resize(params_.horizon+1);
    theta_dot_current.resize(params_.horizon+1,0);
    torque_current.resize(params_.horizon,0);

    float diff = -theta0_val/(params_.horizon);
    for (int i = 0; i < params_.horizon+1; i++){
        theta_current[i] = theta0_val + static_cast<float>(i)*diff;
    }

    p_theta_val.resize(params_.horizon);
    p_scalar_val.resize(params_.horizon);

    // Sequential Quadratic Program

    // initial condition
    theta0.set_data({theta0_val});
    theta0_dot.set_data({theta0_dot_val});

    // slack costs
    slack_theta_cost.set_data(slack_theta_cost_val);
    slack_theta_dot_cost.set_data(slack_theta_dot_cost_val);

    // initial cost
    cost_current = merit_function(theta_current,theta_dot_current,torque_current);

    do
    {
        theta_bar_val.clear();
        theta_bar_val.insert(theta_bar_val.begin(),theta_current.begin(),theta_current.end()-1);
        parameter_generate(theta_current,p_theta_val,p_scalar_val);
        theta_bar.set_data(theta_bar_val);
        p_theta.set_data(p_theta_val);
        p_scalar.set_data(p_scalar_val);
        trust_region.set_data({trust_region_val});

        controller.update();
        controller.solve();

        std::cout << "status: " << controller.get_status() << std::endl;

        // check for convergence
        d_mag = 0;
        for (int i = 0; i < params_.horizon; i++){
            d_mag += std::abs(d_theta.solution[i]);
        }
        if (d_mag <= params_.threshold){
            break;
        }

        // update
        cost_new_predicted = controller.get_obj_val();
        cost_new_actual = merit_function(theta.solution,theta_dot.solution,torque.solution);
     
        predicted_decrease = cost_current - cost_new_predicted;
        actual_decrease = cost_current - cost_new_actual;

        std::cout << "predicted_decrease = " << predicted_decrease << " , actual_decrease = " << actual_decrease << std::endl;
     

        if (actual_decrease >= alpha*predicted_decrease){
            std::cout << "success \n";
            trust_region_val *= beta_succ;

            theta_current = theta.solution;
            theta_dot_current = theta_dot.solution;
            torque_current = torque.solution;
            cost_current = cost_new_actual;

            slack_theta_current = slack_theta.solution;
            slack_theta_dot_current = slack_theta_dot.solution;
        }
        else{
            std::cout << "fail \n";
            trust_region_val *= beta_fail;
        }

        std::cout << "------------------\n";
    } while (++iter < params_.max_iter);
    
    
    if (iter == params_.max_iter){
        std::cout << "max iteration reached\n";
    }
    else{
        std::cout << "solution converged\n";
    }

    std::cout << "current_cost = " << cost_current << std::endl;

    solution.theta = theta_current;
    solution.theta_dot = theta_dot_current;
    solution.torque = torque_current;
    solution.cost = cost_current;
    solution.slack_theta = slack_theta_current;
    solution.slack_theta_dot = slack_theta_dot_current;
    

    return solution;
}
