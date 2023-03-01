#include "cartpole.hpp"
#include <stdio.h>

int main(){
    FILE* x1_data = fopen("x1_data.tmp","w");
    FILE* x2_data = fopen("x2_data.tmp","w");
    FILE* x3_data = fopen("x3_data.tmp","w");
    FILE* x4_data = fopen("x4_data.tmp","w");
    FILE* u_data = fopen("u_data.tmp","w");
    FILE* du_data = fopen("du_data.tmp","w");

    FILE* slack_x1_data = fopen("slack_x1.temp","w");
    FILE* slack_x2_data = fopen("slack_x2.temp","w");
    FILE* slack_x3_data = fopen("slack_x3.temp","w");
    FILE* slack_x4_data = fopen("slack_x4.temp","w");

    CartPole::Params params;
    params.M = 1;
    params.m = 0.3;
    params.l = 0.5;

    params.u_lb = -10;
    params.u_ub = 10;

    params.u_cost = 1;
    params.slack_cost = 100;

    params.horizon = 200;

    CartPole cartpole(params);

    // SQP
    c_float x1_init = 0;
    c_float x2_init = 3.1416;
    c_float x3_init = 0;
    c_float x4_init = 0;

    CartPole::Solution solution;
    solution = cartpole.optimize(x1_init,x2_init,x3_init,x4_init);

    // Data recording
    for (int i = 0; i < params.horizon; i++){
        fprintf(x1_data,"%d %f\n",i,solution.x1[i]);
        fprintf(x2_data,"%d %f\n",i,solution.x2[i]);
        fprintf(x3_data,"%d %f\n",i,solution.x3[i]);
        fprintf(x4_data,"%d %f\n",i,solution.x4[i]);
        
        fprintf(u_data,"%d %f\n",i,solution.u[i]);
        fprintf(du_data,"%d %f\n",i,solution.du[i]);

        fprintf(slack_x1_data,"%d %f\n",i,solution.slack_x1[i]);
        fprintf(slack_x2_data,"%d %f\n",i,solution.slack_x2[i]);
        fprintf(slack_x3_data,"%d %f\n",i,solution.slack_x3[i]);
        fprintf(slack_x4_data,"%d %f\n",i,solution.slack_x4[i]);
    }
    fprintf(x1_data,"%d %f\n",params.horizon,solution.x1[params.horizon]);
    fprintf(x2_data,"%d %f\n",params.horizon,solution.x2[params.horizon]);
    fprintf(x3_data,"%d %f\n",params.horizon,solution.x3[params.horizon]);
    fprintf(x4_data,"%d %f\n",params.horizon,solution.x4[params.horizon]);

    fclose(x1_data);
    fclose(x2_data);
    fclose(x3_data);
    fclose(x4_data);
    fclose(u_data);
    fclose(du_data);

    fclose(slack_x1_data);
    fclose(slack_x2_data);
    fclose(slack_x3_data);
    fclose(slack_x4_data);
    return 0;
}