function [CL,CD,CY] = RCAM_model_A(X,U)
%----------------------------------CONSTANTS---------------------------
% Nominal Vehicle constants


m = 120000;              % Aircraft Total mass (kg)

cbar = 6.6;              % Mean Aerodynamic  Chord (m)
lt = 24.8;               % Distance by AC of tail and body (m)
S = 260;                 % Wing planform area (m^2)
St = 64;                 % Tail planform area (m^2)
 
Xcg = 0.23*cbar;         % x position of CoG in Fm (m)
Ycg = 0;                 % y position of CoG in Fm (m)
Zcg = 0.10*cbar;        % z position of CoG in Fm 
  
Xac = 0.12*cbar;         % x position of aerodynamic center in Fm (m)
Yac = 0;                 % y position of aerodynamic center in Fm (m)
Zac = 0;                 % z position of aerodynamic center in Fm (m)


% Engine Inputs

Umax = 120000*9.81;       % Maximum thrust provided by one engine (N)

Xapt1 = 0;                % x position of engine 1 force in Fm (m)
Yapt1 = -7.94;            % y position of engine 1 force in Fm (m)
Zapt1 = -1.9;             % z position of engine 1 force in Fm (m)

Xapt2 = 0;                % x position of engine 2 force in Fm (m)
Yapt2 = 7.94;             % y position of engine 2 force in Fm (m)
Zapt2 = -1.9;             % z position of engine 2 force in Fm (m)

%Other constants 
rho = 1.225;                 % Air density (kg/m^3)
g = 9.81;                    % Gravitational acceleration (m/s^2)
depsda = 0.25;               % Change in downwash w.r.t alpha (rad/rad)
alpha_L0 = -11.5*(pi/180);   % Zero lift angle of attack (rad)
n = 5.5;                     % Slope of linear region of lift slope
a3 = -768.5;                 % Coefficient of alpha^3
a2 = 609.2;                  % Coefficient of aloha^2
a1 = -155.2;                 % Coefficient of alpha^2
a0 = 15.212;                 % Coefficient of alpha^0 (modificated to get rid of discontinuity)
alpha_switch = 14.5*(pi/180); % alpha where lift slope goes from linear to non-linear

%---------------------------------State Vector-------------------------
%Extract the State vector
x1 = X(1);          % u
x2 = X(2);          % v
x3 = X(3);          % w
x4 = X(4);          % p
x5 = X(5);          % q
x6 = X(6);          % r
x7 = X(7);          % phi
x8 = X(8);          % theta
x9 = X(9);          % psi 

% Extract the Control Vector
u1 = U(1);          % d_A (aileron)
u2 = U(2);          % d_T (stabilizer)
u3 = U(3);          % d_R (rudder)
u4 = U(4);          % d_th1 (throttle 1)
u5 = U(5);          % d_th2 (throttle 2)

%-----------------Intermediate Variables--------------------------
% Calculate air speed 
Va = sqrt(x1^2 + x2^2 + x3^2);

% Calculate alpha and beta 
alpha = atan2(x3,x1);
beta = asin(x2/Va);

% Calculate the dynamic pressure
Q = 1/2*rho*(Va^2);

% Defining the vectors wbe_b and v_b
wbe_b = [x4;x5;x6];
V_b = [x1;x2;x3];

%----------------------------AerodynamicCoefficients---------------------------------------
%Calculate CL_wb
if alpha<=alpha_switch
    CL_wb = n*(alpha-alpha_L0);
else
    CL_wb = a3*alpha^3 + a2*alpha^2 + a1*alpha + a0;
end
% Calculate CL_t
epsilon = depsda*(alpha -alpha_L0);
alpha_t = alpha - epsilon + u2 + 1.3*x5*(lt/Va);
CL_t = 3.1*alpha_t * (St/S);

% Total Lift force Coefficient
CL = CL_wb + CL_t;

% Total Drag Forc Coefficient (neglecting tail)
CD = 0.13 + 0.07*(5.5*alpha + 0.654)^2;

% Total SideForce Coefficient 
CY = -1.6*beta + 0.24*u3;

end