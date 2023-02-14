#include "osqp-interface/qp.hpp"
#include <iostream>

void parameter_generate(std::vector<c_float> theta_iter,std::vector<c_float> &p_theta_val,std::vector<c_float> &p_scalar_val,float Ts,float g,float l);


int main(){
    // Pendulum properties
    float m = 1;
    float l = 1;
    float torque_lb = -1;
    float torque_ub =  1;
    const float g = 9.81;

    float theta0 = 3.14;
    float theta_dot0 = 0;

    // Solver properties
    int horizon = 100;
    float torque_cost = 1;
    float slack_cost = 100;
    float Ts = 0.01;
    float threshold = 1e-3;
    int max_iter = 500;

    float theta_trust_region = 3;
    float alpha = 0.1;
    float beta_succ = 1.1;
    float beta_fail = 0.5;

    // Solver variables and parameters
    QP::Variable theta(horizon+1);
    QP::Variable theta_dot(horizon+1);
    QP::Variable torque(horizon);

    QP::Variable d_theta(horizon);
    QP::Variable slack_theta(horizon);
    QP::Variable slack_theta_abs(horizon);
    QP::Variable slack_theta_dot(horizon);
    QP::Variable slack_theta_dot_abs(horizon);

    QP::Parameter state_init(2);
    QP::Parameter theta_bar(horizon);
    QP::Parameter d_theta_bound(1);

    QP::Parameter p_theta(horizon);
    QP::Parameter p_scalar(horizon);

    QP::QP_Params qp_params;
    qp_params.vars   = {&theta,&theta_dot,&torque,
                        &slack_theta,&slack_theta_abs,
                        &slack_theta_dot,&slack_theta_dot_abs,
                        &d_theta};
                  
    qp_params.params = {&state_init,
                        &p_theta,&p_scalar,
                        &theta_bar,
                        &d_theta_bound};

    QP controller(qp_params);

    // Problem Formulation
    // Initial condition constraints
    controller.add_constraint(state_init[0],theta[0],state_init[0]);
    controller.add_constraint(state_init[1],theta_dot[0],state_init[1]);

    for (int i = 0; i <= horizon-1; i++){
        // Objective function
        controller.add_cost(torque_cost*torque[i]*torque[i]);
        controller.add_cost(slack_cost*slack_theta_abs[i]);
        controller.add_cost(slack_cost*slack_theta_dot_abs[i]);

        // Dynamics constraints
        controller.add_constraint(0,-theta[i+1] + theta[i] + Ts*theta_dot[i] + slack_theta[i],0);
        controller.add_constraint(-p_scalar[i],-theta_dot[i+1] + theta_dot[i] + p_theta[i]*d_theta[i] + (Ts/(m*std::pow(l,2)))*torque[i] + slack_theta_dot[i],-p_scalar[i]);
        controller.add_constraint(theta_bar[i],-d_theta[i]+theta[i],theta_bar[i]);

        // Torque constraints
        controller.add_constraint(torque_lb,torque[i],torque_ub);
        
        // Slack constraints
        controller.add_constraint(0,slack_theta_abs[i]-slack_theta[i],QP::POS_INF);
        controller.add_constraint(0,slack_theta_abs[i]+slack_theta[i],QP::POS_INF);
        controller.add_constraint(0,slack_theta_dot_abs[i]-slack_theta_dot[i],QP::POS_INF);
        controller.add_constraint(0,slack_theta_dot_abs[i]+slack_theta_dot[i],QP::POS_INF);

        // Trust region constraints
        controller.add_constraint(-d_theta_bound[0],d_theta[i],d_theta_bound[0]);
    }

    // Terminal constraints
    controller.add_constraint(0,theta[horizon],0);
    controller.add_constraint(0,theta_dot[horizon],0);

    // Formulate
    controller.get_settings()->verbose = 0;
    controller.formulate();

    // SQP
    std::vector<c_float> p_theta_val,p_scalar_val;
    std::vector<c_float> theta_iter,theta_test;
    std::vector<c_float> torque_iter,theta_dot_iter;
    std::vector<c_float> torque_test,theta_dot_test;
    
    c_float current_cost,predicted_cost,actual_cost;
    c_float predicted_decrease,actual_decrease;
    c_float d_mag;
    uint8_t iter = 0;

    p_theta_val.resize(horizon);
    p_scalar_val.resize(horizon);

    // initial guess
    theta_test.resize(horizon);
    theta_iter.clear();
    float diff = -theta0/(horizon-1);
    for (int i = 0; i < horizon; i++){
        theta_iter.push_back(theta0 + static_cast<float>(i)*diff);
    }

    // Sequential Quadratic Program

    // initial condition
    state_init.set_data({theta0,theta_dot0});

    // initial cost
    parameter_generate(theta_iter,p_theta_val,p_scalar_val,Ts,g,l);
    theta_bar.set_data(theta_iter);
    p_theta.set_data(p_theta_val);
    p_scalar.set_data(p_scalar_val);
    d_theta_bound.set_data({0});

    controller.update();
    controller.solve();

    theta_dot_iter = theta_dot.solution;
    torque_iter    = torque.solution;
    current_cost = controller.get_obj_val();

    do
    {
        parameter_generate(theta_iter,p_theta_val,p_scalar_val,Ts,g,l);
        theta_bar.set_data(theta_iter);
        p_theta.set_data(p_theta_val);
        p_scalar.set_data(p_scalar_val);
        d_theta_bound.set_data({theta_trust_region});

        controller.update();
        controller.solve();

        std::cout << "status: " << controller.get_status() << std::endl;

        torque_test = torque.solution;
        theta_dot_test = theta_dot.solution;

        // check for convergence
        d_mag = 0.;
        for (auto d : d_theta.solution){
            d_mag += std::abs(d);
        }
        if (d_mag <= threshold){
            break;
        }
        
     
        // find predicted cost
        predicted_cost = controller.get_obj_val();
     
        // find actual cost
        
        for (int i = 0; i < horizon; i++){
            theta_test[i] = theta_iter[i] + d_theta.solution[i];
        }
        
        parameter_generate(theta_test,p_theta_val,p_scalar_val,Ts,g,l);
        theta_bar.set_data(theta_test);
        p_theta.set_data(p_theta_val);
        p_scalar.set_data(p_scalar_val);
        d_theta_bound.set_data({0.});

        controller.update();
        controller.solve();

        actual_cost = controller.get_obj_val();

        // update trust region
        predicted_decrease = current_cost - predicted_cost;
        actual_decrease    = current_cost - actual_cost;

        // std::cout << "predicted_decrease = " << predicted_decrease << " , actual_decrease = " << actual_decrease << std::endl;


        if (actual_decrease >= alpha*predicted_decrease){
            // std::cout << "success \n";
            theta_trust_region *= beta_succ;
            theta_iter = theta_test;
            theta_dot_iter = theta_dot_test;
            torque_iter = torque_test;
            current_cost = actual_cost;
        }
        else{
            // std::cout << "fail \n";
            theta_trust_region *= beta_fail;
        }
    } while (iter++ <= max_iter);
    
    
    if (iter >= max_iter){
        std::cout << "max iteration reached\n";
    }
    else{
        std::cout << "solution converged\n";
    }

    std::cout << "current_cost = " << current_cost << std::endl;


    
    return 0;
}

void parameter_generate(std::vector<c_float> theta_iter,std::vector<c_float> &p_theta_val,std::vector<c_float> &p_scalar_val,float Ts,float g,float l){
    p_theta_val.clear();
    p_scalar_val.clear();
    
    for (auto theta : theta_iter){
        p_theta_val.push_back(Ts*(g/l)*std::cos(theta));
        p_scalar_val.push_back(Ts*(g/l)*std::sin(theta));
    }
}
