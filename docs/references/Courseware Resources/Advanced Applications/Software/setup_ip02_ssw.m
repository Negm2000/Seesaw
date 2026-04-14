% SETUP_IP02_SSW
%
% IP02 Seesaw (SEESAW) Control Lab: 
% Design of a LQR position controller
% 
% SETUP_IP02_SSW sets the SEESAW and IP02 system's 
% model parameters accordingly to the user-defined configuration.
% SETUP_IP02_SSW can also set the controllers' parameters, 
% accordingly to the user-defined desired specifications.
%
% Copyright (C) 2012 Quanser Consulting Inc.
% Quanser Consulting Inc.

clear

% ########## USER-DEFINED IP02 with SEESAW CONFIGURATION ##########
% Type of IP02 cart load: set to 'NO_WEIGHT', 'WEIGHT'
IP02_WEIGHT_TYPE = 'NO_WEIGHT';
% IP02_WEIGHT_TYPE = 'WEIGHT';
% Turn on or off the safety watchdog on the cart position: set it to 1 , or 0 
X_LIM_ENABLE = 1;       % safety watchdog turned ON
%X_LIM_ENABLE = 0;      % safety watchdog turned OFF
% Safety limits on the IP02 cart displacement (m)
X_MAX = 0.35;            % cart displacement maximum safety position (m)
X_MIN = - X_MAX;        % cart displacement minimum safety position (m)
% Amplifier Gain used: set VoltPAQ-X1 gain switch to 1
K_AMP = 1;
% Amplifier Type: set to 'VoltPAQ' or 'Q3'
AMP_TYPE = 'VoltPAQ';
% Digital-to-Analog Maximum Voltage (V); for MultiQ cards set to 10
VMAX_DAC = 10;

% ########## USER-DEFINED CONTROLLER DESIGN ##########
% Type of Controller: set it to 'LQR_AUTO', 'LQR_GUI_TUNING', 'MANUAL'  
CONTROLLER_TYPE = 'LQR_AUTO';    % LQR controller design: automatic mode
%CONTROLLER_TYPE = 'MANUAL';    % controller design: manual mode
% Initial Condition on xc, i.e. cart position at t = 0 (m)
X0(1) = 0;
% Initial Condition on theta, i.e. the seesaw tilt angle at t = 0 (deg)
X0(2) = 6;
% conversion to radians
X0(2) = X0(2) / 180 * pi;
% initial state variables (at t=0)
X0 = [ X0(1); X0(2); 0; 0; 0 ];
% Initialization of Simulink diagram parameters
    % Cart Encoder Resolution
    global K_EC K_EP
    % Specifications of a second-order low-pass filter
    wcf = 2 * pi * 10; % filter cutting frequency
    zetaf = 0.9;        % filter damping ratio

% Integral anti-windup limits for the limiter integrator (rad.s)
MAX_I_WINDUP = 10 / 180 * pi;
MIN_I_WINDUP = - MAX_I_WINDUP;

% variables required in the Simulink diagrams
global VMAX_AMP IMAX_AMP KE_SW

% Set the model parameters accordingly to the user-defined IP01 or IP02 system configuration.
% These parameters are used for model representation and controller design.
[ Rm, Jm, Kt, eta_m, Km, Kg, eta_g, mc, r_mp, Beq ] = config_ip02( IP02_WEIGHT_TYPE, AMP_TYPE );

% Set the model parameters for the SEESAW accordingly to the user-defined system configuration.
[ msw, Kgs, Dt, Dc, Jsw, Bsw, g ] = config_ssw( );

% Initialization of the State-Space Representation of the Open-Loop System
% First, for the following state vector: X = [ xc; theta; xc_dot; theta_dot ]
% Call the following Maple-generated file to initialize the State-Space Matrices: A, B, C, and D
SEESAW_ABCD_eqns

% Then add the 5th state: integration of the error on theta.
% Resulting state vector: X = [ xc; theta; xc_dot; theta_dot; theta_int ]
% Adjusted state-space model
A( 5, 1:5 ) = zeros( 1, 5 ); A( 5, 2 ) = 1
B( 5 ) = 0
C( 1:2 , 5 ) = zeros( 2, 1 ); %C( 5, 5 ) = 1;
%D( 5 ) = 0;

 if strcmp ( CONTROLLER_TYPE, 'LQR_AUTO' )
        Q = diag( [ 1000 5000 0 0 2000 ] );
        R(1,1) = [ 0.5 ];
        INT_MAX = 3.5;
    % Automatically calculate the LQR controller gain
    [ K ] = d_ip02_ssw_lqr( A, B, C, D, Q, R, X0 );
    % Display the calculated gains
    disp( ' ' )
    %disp( 'Calculated LQR controller gain elements: ' )
    disp( [ 'K(1) = ' num2str( K(1) ) ' V/m' ] )
    disp( [ 'K(2) = ' num2str( K(2) ) ' V/rad' ] )
    disp( [ 'K(3) = ' num2str( K(3) ) ' V.s/m' ] )
    disp( [ 'K(4) = ' num2str( K(4) ) ' V.s/rad' ] )
    disp( [ 'K(5) = ' num2str( K(5) ) ' V/s/rad' ] )
elseif strcmp ( CONTROLLER_TYPE, 'MANUAL' )
    K = [ 0 0 0 0 0 ];
    disp( ' ' )
    disp( 'STATUS: manual mode' ) 
    disp( 'The model parameters of your Seesaw with IP01 or IP02 system have been set.' )
    disp( 'You can now design your state-feedback position controller.' )
    disp( ' ' )
else
    error( 'Error: Please set the type of controller that you wish to implement.' )
end
