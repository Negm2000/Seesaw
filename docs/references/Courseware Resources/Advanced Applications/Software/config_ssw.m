% CONFIG_SSW
%
% CONFIG_SSW accepts the user-defined configuration 
% of the Quanser SEESAW-E module. CONFIG_SSW then sets up the 
% SEESAW-E configuration-dependent model variables accordingly, 
% and finally returns the calculated model parameters of the 
% SEESAW-E Quanser module.
%
% SEESAW-E system nomenclature:
% Msw       Mass of the one-Seesaw-plus-one-IP01-or-IP02-Track System        (kg)
% Kgs       Seesaw Geartrain Gear Ratio
% Dt        Distance from Pivot to the IP01 or IP02 Track                     (m)
% Dc        Distance from Pivot to the Centre Of Gravity of the 
%               one-Seesaw-plus-one-IP01-or-IP02-Track System                 (m)
% Jsw       Moment of Inertia of the one-Seesaw-plus-one-IP01-or-IP02-Track
%               System, about its Center Of Gravity                           (kg.m^2)
% Bsw       Viscous Damping Coefficient as seen at the Seesaw Pivot Axis      (N.m.s/rad)
% g         Gravitational Constant on Earth                                   (m/s^2)
% KP_SW     SEESAW Pivot Potentiometer Sensitivity                            (rad/V)
% KE_SW     SEESAW-E Pivot Encoder Resolution                                 (rad/count)
% Copyright (C) 2012 Quanser Consulting Inc.
% Quanser Consulting Inc.


%% returns the model parameters accordingly to the USER-DEFINED SEESAW-E configuration
function [ Msw, Kgs, Dt, Dc, Jsw, Bsw, g ] = config_ssw( )
% Gravity Constant
g = 9.81;
% Calculate the SEESAW(-E) model parameters
[ Msw, Kgs, Dt, Dc, Jsw, Bsw ] = calc_ssw_parameters( );
% end of 'setup_ssw_configuration( )'


%% Calculate the SEESAW(-E) model parameters 
function [ msw, Kgs, Dt, Dc, Jsw, Bsw ] = calc_ssw_parameters( )
% Mass of the one-Seesaw-plus-one-IP01-or-IP02-Track System (kg) 
msw = 3.6;
% Seesaw Geartrain Gear Ratio
Kgs = 3;
% Distance from Pivot to the IP01 or IP02 Track (m)
Dt = 0.125;
% Distance from Pivot to the Centre Of Gravity of 
% the one-Seesaw-plus-one-IP01-or-IP02-Track System (m)
Dc = 0.058;
% Moment of Inertia of the one-Seesaw-plus-one-IP01-or-IP02-Track System,
% about its Center Of Gravity (kg.m^2)
Jsw = 0.395;
% estimate Bsw
Bsw = 0;
    % Pivot Encoder Resolution (rad/count)
    global KE_SW
    % KE_SW is positive, since CCW is the positive sense of rotation
    KE_SW = 2 * pi / ( 4 * 1024 ); % = 0.0015
% end of 'calc_ssw_parameters( )'
