#include "osqp-interface/qp.hpp"
#include <iostream>
#include <cmath>

float slack_cost = 5;

/*
Target function
y = x^4 - 3x^2 + x
*/

// Merit function
float merit_function(float x_,float y_){
    float cost = 0;
    cost += y_;
    cost += slack_cost*std::abs(-y_ + std::pow(x_,4) - 3*std::pow(x_,2) + x_);

    return cost;
}

// parameter values generation
void param_gen(float x_,float y_,float &p_x,float &p_scalar){
    // p_scalar = -y_ + std::pow(x_,4) - 3*std::pow(x_,2) + x_;
    p_scalar = std::pow(x_,4) - 3*std::pow(x_,2) + x_;
    p_x = 4*std::pow(x_,3)-6*x_+1;
}

int main(){
    QP::Variable x(1),y(1),dx(1),s(1),s_abs(1);
    QP::Parameter x_(1),p_scalar(1),p_x(1);
    QP::Parameter d_trust(1);

    QP::QP_Params qp_params;
    qp_params.vars = {&x,&y,&dx,&s,&s_abs};
    qp_params.params = {&x_,&p_scalar,&p_x,&d_trust};
    QP qp(qp_params);

    // Problem formulation
    qp.add_cost(y[0] + slack_cost*s_abs[0]);

    // qp.add_constraint(-p_scalar[0],-1*dy[0] + p_x[0]*dx[0] + s[0],-p_scalar[0]);
    qp.add_constraint(-p_scalar[0],-1*y[0] + p_x[0]*dx[0] + s[0],-p_scalar[0]);
    qp.add_constraint(x_[0],x[0]-dx[0],x_[0]);
    // qp.add_constraint(y_[0],y[0]-dy[0],y_[0]);
    qp.add_constraint(-d_trust[0],dx[0],d_trust[0]);
    // qp.add_constraint(-d_trust[0],dy[0],d_trust[0]);
    qp.add_constraint(0,s_abs[0]-s[0],QP::POS_INF);
    qp.add_constraint(0,s_abs[0]+s[0],QP::POS_INF);

    qp.get_settings()->verbose = 0;
    qp.formulate();

    // SQP
    int max_iter = 200;
    int iter = 0;
    float cost_current,cost_new_predicted,cost_new_actual;
    float predicted_decrease,actual_decrease;
    float d_mag;
    float step_size_start = 0.1;
    float step_size;
    float x_new,y_new;
    float p_x_val,p_scalar_val;

    float x_current = 10;
    float y_current = 100;
    float trust = 1;
    cost_current = merit_function(x_current,y_current);

    do
    {
        param_gen(x_current,y_current,p_x_val,p_scalar_val);
        x_.set_data({x_current});
        // y_.set_data({y_current});
        p_x.set_data({p_x_val});
        p_scalar.set_data({p_scalar_val});
        d_trust.set_data({trust});

        qp.update();
        qp.solve();

        // std::cout << "x_current = " << x_current << " , y_current = " << y_current << std::endl;
        // std::cout << "dx = " << dx.solution[0] << " , dy = " << dy.solution[0] << std::endl;
        // std::cout << "x_new = " << x.solution[0] << " , y_new = " << y.solution[0] << std::endl;
        
        // check for convergence
        d_mag = std::abs(dx.solution[0]);
        if (d_mag <= 1e-3){
            break;
        }

        // update 
        cost_new_predicted = qp.get_obj_val();
        cost_new_actual = merit_function(x.solution[0],y.solution[0]);

        predicted_decrease = cost_current - cost_new_predicted;
        actual_decrease = cost_current - cost_new_actual;

        std::cout << "predicted_decrease = " << predicted_decrease << " , actual_decrease = " << actual_decrease << std::endl;

        if (actual_decrease >= 0.1*predicted_decrease){
            // std::cout << "success\n";
            trust *= 1.1;

            x_current = x.solution[0];
            y_current = y.solution[0];
            cost_current = cost_new_actual;
        }
        else{
            // std::cout << "fail\n";
            trust *= 0.5;
        }

        // std::cout << "---------------------\n";

    } while (++iter < max_iter);

    std::cout << "number of iterations: " << iter << std::endl;
    std::cout << "x = " << x_current << std::endl;
    std::cout << "y = " << y_current << std::endl;
    
    return 0;
}