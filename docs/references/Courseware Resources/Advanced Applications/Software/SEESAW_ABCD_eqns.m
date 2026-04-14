% Matlab equation file: "SEESAW_ABCD_eqns.m"
% Open-Loop State-Space Matrices: A, B, C, and D
% for the Quanser Seesaw Experiment.

A = [0 0 1 0;
     0 0 0 1;
     -mc*Dt*g/Jsw (-g*mc*Jsw+mc*Dt*g*msw*Dc)/mc/Jsw (-mc*Dt^2*Beq-Beq*Jsw)/mc/Jsw -Dt*Bsw/Jsw;
     -g*mc/Jsw g*msw*Dc/Jsw -Dt*Beq/Jsw -Bsw/Jsw];

B = [0; 0; (Jsw+mc*Dt^2)/mc/Jsw; Dt/Jsw];
C = eye(2,4);
D = zeros(2,1);

%Actuator Dynamics
A(3,3) = A(3,3) - B(3)*eta_g*Kg^2*eta_m*Kt*Km/r_mp^2/Rm;
A(4,3) = A(4,3) - B(4)*eta_g*Kg^2*eta_m*Kt*Km/r_mp^2/Rm;
B = eta_g*Kg*eta_m*Kt/r_mp/Rm*B;