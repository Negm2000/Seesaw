% D_IP02_SSW_LQR
%
% Control Lab: Design of a LQR Controller
% for an IP02 system mounted on a SEESAW(-E)
%
% D_IP02_SSW_LQR designs a LQR controller for the SEESAW and IP02 system,
% and returns the corresponding gain vector: K
%
% Copyright (C) 2012 Quanser Consulting Inc.
% Quanser Consulting Inc.


function [ K ] = d_ip02_ssw_lqr( A, B, C, D, Q, R, X0 )
%PLOT_RESPONSE = 'YES';
PLOT_RESPONSE = 'NO';
%SYS_ANALYSIS = 'YES';
SYS_ANALYSIS = 'NO';

% Open Loop system
%IP01_2_SEESAW_OL_SYS = ss( A, B, C, D, 'statename', { 'xc' 'theta' 'xc_dot' 'theta_dot' 'theta_int' }, 'inputname', 'Vm', 'outputname', { 'xc' 'theta' 'xc_dot' 'theta_dot' 'theta_int' } );

% calculate the LQR gain vector, K
[ K, S, EIG_CL ] = lqr( A, B, Q, R );

% Closed-Loop State-Space Model
A_CL = A - B * K;
B_CL = B;
C_CL = C;
D_CL = D;

% Closed-Loop System
%IP01_2_SEESAW_CL_SYS = ss( A_CL, B_CL, C_CL, D_CL, 'statename', { 'xc' 'theta' 'xc_dot' 'theta_dot' 'theta_int' }, 'inputname', 'Vm', 'outputname', { 'xc' 'theta' 'xc_dot' 'theta_dot' 'theta_int' } );

if strcmp( PLOT_RESPONSE, 'YES' )
    % initialization
    close all
    fig_h = 1; % figure handle number

    % closed-loop response to xc0 == X0(1) != 0 and theta0 == X0(2) != 0
    figure( fig_h )
    tss_IC = 0 : 0.001 : 5;
    [ yss_IC, tss_IC, xss_IC ] = initial( IP01_2_SEESAW_CL_SYS, X0, tss_IC );
    subplot( 2, 1, 1 )
    plot( tss_IC, xss_IC( :, 1 )*1000 )
    grid on
    title( [ 'Response to Initial Conditions: x_c_0 = ' num2str( X0(1)*1e3 ) ' mm and \theta_0 = ' num2str( X0(2)*180/pi ) ' deg' ] )
    xlabel( 'Time (s)' )
    ylabel( 'x_c (mm)' )
    subplot( 2, 1, 2 )
    plot( tss_IC, xss_IC( :, 2 ) * 180 / pi )
    grid on
    xlabel( 'Time (s)' )
    ylabel( '\theta (deg)' )
    set( fig_h, 'name', strcat( 'Closed-Loop System: SEESAW + IP01_2 + LQR' ) )
    fig_h = fig_h + 1;
    
    % corresponding control effort: Vm = - K * X
    figure( fig_h )
    Vm_IC = -K * xss_IC';
    plot( tss_IC, Vm_IC )
    grid on
    title( [ 'Response to Initial Conditions: x_c_0 = ' num2str( X0(1)*1e3 ) ' mm and \theta_0 = ' num2str( X0(2)*180/pi ) ' deg' ] )
    xlabel( 'Time (s)' )
    ylabel( 'V_m (V)' )
    grid on
    set( fig_h, 'name', strcat( 'Closed-Loop System: SEESAW + IP01_2 + LQR' ) )
    fig_h = fig_h + 1;
end


% carry out some additional system analysis
if strcmp( SYS_ANALYSIS, 'YES' )
    ULABELS = [ 'V_m' ];
    XLABELS = [ 'xc theta xc_dot theta_dot theta_int' ];
    YLABELS = [ 'xc theta xc_dot theta_dot theta_int' ];
    % print the Open-Loop State-Space Matrices
    disp( 'Open-Loop System' )
    printsys( A, B, C, D, ULABELS, YLABELS, XLABELS )
    % open-loop pole-zero structure: NMP zero, RHP pole
    [ z_ol, p_ol, k_ol ] = ss2zp( A, B, C, D, 1 )
    % print the Closed-Loop State-Space Matrices
    disp( 'Closed-Loop System' )
    printsys( A_CL, B_CL, C_CL, D_CL, ULABELS, YLABELS, XLABELS )
    % closed-loop pole-zero structure (NMP zero)
    [ z_cl, p_cl, k_cl ] = ss2zp( A_CL, B_CL, C_CL, D_CL, 1 )
    % Closed-Loop poles, damping, and natural frequency
    damp( IP01_2_SEESAW_CL_SYS )
end
% end of function 'd_ip01_2_ssw_lqr( )'
