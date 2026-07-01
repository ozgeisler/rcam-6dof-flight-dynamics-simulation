clear
clc
close all

% Straight and Level condition

Xdoto =[
    0
    0
    0
    0
    0
    0
    0
    0
    0
    ];
Xo = [
    84.9905
    0
    1.2713
    0
    0
    0
    0
    0.0150
    0];
Uo =[
    0
    -0.1780
    0
    0.0821
    0.0821];
% Define the perturbatin matrices 
dxdot_matrix = 10e-12*ones(9,9);
dx_matrix    = 10e-12*ones(9,9);
du_matrix = 10e-12*ones(9,5);

[E,Ap,Bp] = ImplicitLinMod(@RCAM_model_implicit,Xdoto,Xo,Uo,dxdot_matrix,dx_matrix,du_matrix);

% Calculate the A and B Matrices
A = -inv(E)*Ap;
B = -inv(E)*Bp;

[Alim,Blim,Clim,Dlim] = linmod('RCAM_linmod',Xo,Uo)