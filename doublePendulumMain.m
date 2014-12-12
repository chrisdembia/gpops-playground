%------------------------- Double Pendulum -------------------------------%
% Based on Robot Arm example from GPOPS.                                  %
% This example is taken verbatim from the following reference:            %
%   Benchmarking Optimization Software with COPS Elizabeth D. Dolan       %
%   and Jorge J. More ARGONNE NATIONAL LABORATORY                         %
%-------------------------------------------------------------------------%
clear all
close all

%-------------------------------------------------------------------------%
%------------------ Provide Auxiliary Data for Problem -------------------%
%-------------------------------------------------------------------------%
auxdata.g = 9.81;
auxdata.L1 = 1;
auxdata.L2 = 1;
auxdata.m1 = 1;
auxdata.m2 = 1;

%-------------------------------------------------------------------------%
%----------------- Provide All Bounds for Problem ------------------------%
%-------------------------------------------------------------------------%
t0 = 0;
tf_lower = 0;
tf_upper = 5;

theta1_init = 0;
theta2_init = 0;
theta1dot_init = 0;
theta2dot_init = 0;
initialstate = [theta1_init, theta2_init, theta1dot_init, theta2dot_init];

theta1_final = pi;
theta2_final = -0.5 * pi;
theta1dot_final = 0;
theta2dot_final = 0;
finalstate = [theta1_final, theta2_final, theta1dot_final, theta2dot_final];

theta1_lower = -pi;
theta1_upper = pi;
theta2_lower = -pi;
theta2_upper = pi;
theta1dot_lower = -100 * pi;
theta1dot_upper = 100 * pi;
theta2dot_lower = -100 * pi;
theta2dot_upper = 100 * pi;
state_lower = [theta1_lower, theta2_lower, theta1dot_lower, theta2dot_lower];
state_upper = [theta1_upper, theta2_upper, theta2dot_upper, theta2dot_upper];

u_lower = -1000;
u_upper = 1000;

%-------------------------------------------------------------------------%
%----------------------- Setup for Problem Bounds ------------------------%
%-------------------------------------------------------------------------%
bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = tf_lower;
bounds.phase.finaltime.upper = tf_upper;
bounds.phase.initialstate.lower = initialstate;
bounds.phase.initialstate.upper = initialstate;
bounds.phase.state.lower = state_lower;
bounds.phase.state.upper = state_upper;
bounds.phase.finalstate.lower = finalstate;
bounds.phase.finalstate.upper = finalstate;
bounds.phase.control.lower = [u_lower, u_lower];
bounds.phase.control.upper = [u_upper, u_upper];

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%
t_guess             = [t0; 2];

theta1_guess        = [theta1_lower; theta1_upper];
theta2_guess        = [theta2_lower; theta2_upper];
theta1dot_guess     = [theta1dot_lower; theta1dot_upper];
theta2dot_guess     = [theta2dot_lower; theta2dot_upper];
state_guess = [theta1_guess, theta2_guess, theta1dot_guess, theta2dot_guess];

u1_guess            = [u_lower; u_upper];
u2_guess            = [u_lower; u_upper];

guess.phase.state   = state_guess;
guess.phase.control = [u1_guess, u2_guess];
guess.phase.time    = [t_guess];
guess.auxdata = auxdata; % TODO bug.

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method          = 'hp-PattersonRao';
mesh.tolerance       = 1e-6;
mesh.maxiteration    = 10;
mesh.colpointsmin    = 3;
mesh.colpointsmax    = 10;
N                    = 10;
mesh.phase.colpoints = 3*ones(1,N);
mesh.phase.fraction  = ones(1,N)/N;

%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%
%-------------------------------------------------------------------------%
setup.mesh                            = mesh;
setup.name                            = 'Double-Pendulum';
setup.functions.endpoint              = @doublePendulumEndpoint;
setup.functions.continuous            = @doublePendulumContinuous;
setup.displaylevel                    = 1;
setup.auxdata                         = auxdata;
setup.bounds                          = bounds;
setup.guess                           = guess;
setup.nlp.solver                      = 'ipopt';
setup.nlp.ipoptoptions.linear_solver  = 'ma57';
setup.derivatives.supplier            = 'sparseCD';
setup.derivatives.derivativelevel     = 'second';
setup.method                          = 'RPM-Integration';
setup.nlp.options.tolerance           = 1e-7;
setup.derivatives.derivativelevel     = 'second';
setup.mesh                            = mesh;
setup.scales.method                   = 'automatic-guess';

%-------------------------------------------------------------------------%
%----------------------- Solve Problem Using GPOPS2 ----------------------%
%-------------------------------------------------------------------------%
output = gpops2(setup);
