#include "osqp-interface/qp.hpp"

int main(){
    QP::Variable x(1),y(1);
    QP::Parameter p(1);

    QP::QP_Params qp_params;
    qp_params.vars = {&x,&y};
    qp_params.params = {&p};
    QP qp(qp_params);

    p.set_data({1});

    qp.add_cost(y[0]);

    qp.add_constraint(-p[0],-y[0]+0*x[0],-p[0]);
    // qp.add_constraint(-1,x[0],1);
    // qp.add_constraint(1,x[0],1);

    qp.formulate();
    qp.solve();

    std::cout << "obj_val = " << qp.get_obj_val() << std::endl;
    std::cout << "y = " << y.solution[0] << std::endl;
    std::cout << "x = " << x.solution[0] << std::endl;

    return 0;
}