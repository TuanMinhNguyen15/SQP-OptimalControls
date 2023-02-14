#include "pendulum.hpp"
#include <stdio.h>


int main(){
    FILE* theta_data = NULL;
    FILE* theta_dot_data = NULL;
    FILE* torque_data = NULL;
    FILE* slack_theta_data = NULL;
    FILE* slack_theta_dot_data = NULL;

    Pendulum::Solution solution;
    Pendulum::Params params;
    params.l = 1;
    params.m = 1;
    params.torque_lb = -2;
    params.torque_ub =  2;

    params.horizon = 200;
    params.torque_cost = 1;
    params.slack_cost  = 300;

    params.max_iter = 200;
    Pendulum pendulum(params);

    solution = pendulum.solve(3.1416,0);

    // std::cout << "Objective value = " << solution.cost << std::endl;

    theta_data = fopen("theta_data.tmp","w");
    theta_dot_data = fopen("theta_dot_data.temp","w");
    torque_data = fopen("torque_data.temp","w");
    slack_theta_data = fopen("slack_theta_data.tmp","w");
    slack_theta_dot_data = fopen("slack_theta_dot_data.temp","w");

    for (int i = 0; i < params.horizon; i++){
        fprintf(theta_data, "%d %f\n",i,solution.theta[i]);
        fprintf(theta_dot_data, "%d %f\n",i,solution.theta_dot[i]);
        fprintf(torque_data, "%d %f\n",i,solution.torque[i]);
        fprintf(slack_theta_data, "%d %f\n",i,solution.slack_theta[i]);
        fprintf(slack_theta_dot_data, "%d %f\n",i,solution.slack_theta_dot[i]);   
    }
    fprintf(theta_data, "%d %f\n",params.horizon,solution.theta[params.horizon]);
    fprintf(theta_dot_data, "%d %f\n",params.horizon,solution.theta_dot[params.horizon]);

    fclose(theta_data);
    fclose(theta_dot_data);
    fclose(torque_data);
    fclose(slack_theta_data);
    fclose(slack_theta_dot_data);
    return 0;
}