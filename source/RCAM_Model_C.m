function[FE_b,MEcg_b,Fg_b] = RCAM_Model_C(X,U)

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


%--------------------------------Engine Force and Moment--------------------
F1 = u4*Umax;
F2 = u5*Umax;

% Assuming that engine thrust is aligned with Fb,we have
FE1_b = [F1;0;0];
FE2_b = [F2;0;0];
FE_b = FE1_b + FE2_b;

% Engine Moment due to offset of engine thrust from CoG
mew1 = [Xcg-Xapt1;
     Yapt1 - Ycg;
     Zcg - Zapt1];

mew2 = [Xcg-Xapt2;
     Yapt2 - Ycg;
     Zcg - Zapt2];

MEcg1_b = cross(mew1,FE1_b);
MEcg2_b = cross(mew2,FE2_b);

MEcg_b = MEcg1_b + MEcg2_b;
%------------------Gravity EFfects----------------
% Calculate gravitational forces in the body frame.It causes no moment aout
% Cog.
g_b = [-g*sin(x8);
    g*cos(x8)*sin(x7);
    g*cos(x8)*cos(x7)];

Fg_b = m*g_b;



end