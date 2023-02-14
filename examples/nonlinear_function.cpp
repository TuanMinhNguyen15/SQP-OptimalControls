#include "osqp-interface/qp.hpp"

/*
y = f(x) = x^4 - 2x^2 + 0.5x
*/

void param_val_gen(c_float x_bar, c_float &alpha_val, c_float &beta_val){
    alpha_val = 4*std::pow(x_bar,3) - 4*x_bar + 0.5;
    beta_val  = std::pow(x_bar,4) - 2*std::pow(x_bar,2) + 0.5*x_bar;
}

int main(){
    c_float alpha_val,beta_val,x_bar_val,lb_val,ub_val;
    c_float trust_region = 10;
    c_float d_mag;
    c_float x_iter,x_test;
    c_float current_cost,predicted_cost,actual_cost;
    c_float predicted_decrease,actual_decrease;
    int iter = 0;
    int iter_max = 100;

    QP::Variable x(1),y(1),d(1);
    QP::Parameter alpha(1),beta(1),x_bar(1),lb(1),ub(1);

    QP::QP_Params qp_params;
    qp_params.vars = {&x,&y,&d};
    qp_params.params = {&alpha,&beta,&x_bar,&lb,&ub};

    QP qp(qp_params);

    // objective function
    qp.add_cost(y[0]);
    
    // constraints
    qp.add_constraint(-beta[0],-y[0]+alpha[0]*d[0],-beta[0]);
    qp.add_constraint(x_bar[0],-d[0]+x[0],x_bar[0]);
    qp.add_constraint(lb[0],d[0],ub[0]);

    // formulate
    qp.get_settings()->verbose = 0;
    qp.formulate();

    // solve

    // initial guess
    x_iter = -5;

    param_val_gen(x_iter,alpha_val,beta_val);
    alpha.set_data({alpha_val});
    beta.set_data({beta_val});
    x_bar.set_data({x_iter});
    lb.set_data({0});
    ub.set_data({0});

    qp.update();
    qp.solve();

    current_cost = qp.get_obj_val();

    bool found = false;
    do
    {
        param_val_gen(x_iter,alpha_val,beta_val);
        alpha.set_data({alpha_val});
        beta.set_data({beta_val});
        x_bar.set_data({x_iter});
        lb.set_data({-trust_region});
        ub.set_data({trust_region});
        
        qp.update();
        qp.solve();

        // check for convergence
        if (std::abs(d.solution[0]) <= 1e-3){
            found = true;
            break;
        }

        // find predicted cost
        predicted_cost = qp.get_obj_val();

        // find actual cost
        x_test = x_iter + d.solution[0];
        param_val_gen(x_test,alpha_val,beta_val);
        alpha.set_data({alpha_val});
        beta.set_data({beta_val});
        x_bar.set_data({x_test});
        lb.set_data({0});
        ub.set_data({0});

        qp.update();
        qp.solve();

        actual_cost = qp.get_obj_val();

        // update trust region
        predicted_decrease = current_cost - predicted_cost;
        actual_decrease    = current_cost - actual_cost;

        std::cout << "predicted_decrease = " << predicted_decrease << " , actual_decrease " << actual_decrease << std::endl;

        if (actual_decrease >= 0.1*predicted_decrease){
            std::cout << "success\n";
            trust_region *= 1.1;
            x_iter = x_test;
            current_cost = actual_cost;
        }
        else{
            std::cout << "fail\n";
            trust_region *= 0.5;
        }

        iter++;
    } while (iter < iter_max);
    
    if (found == true){
        std::cout << "solution converged: \n" << "x = " << x_iter << " , y = " << current_cost << std::endl;
    }
    else{
        std::cout << "maximum iteration reached\n";
    }
    return 0;
}