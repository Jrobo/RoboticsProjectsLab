close all
clear
clc

%% The description of the system
%Design Parameters
Ld = 0.05  %H
Lq = 0.011 %H
Rs = 0.62  %OHM
we = 785   %rad/s

%matrices
A = [ -Rs/Ld  we*(Lq/Ld); -we*(Ld/Lq) -Rs/Lq];
B = [1/Ld 0; 0 1/Lq];
C = [1 0 ;0 1];
D = [0 0; 0 0];

%Continuous system
Gs = ss(A,B,C,D);

%sampling time [s]
Ts = 1/8000 % 8kHz sampling frequeancy

%discrete system and matrices
Gz = c2d(Gs,Ts);

Ad = Gz.A;
Bd = Gz.B;
Cd = Gz.C;
Dd = Gz.D;

x0 = [0,0]';
xref = [6.9 ; 8.9];

%% Poles & Zeroes of the system
% Compute poles and zeros for continuous-time system
Poles_Gs = pole(Gs);
Zeros_Gs = tzero(Gs);
pzmap(Gs)

% Compute poles and zeros for discrete-time system
pzmap(Gz)
[Poles_Gz, Zeros_Gz] = pzmap(Gz)

%% Equilibrium state and input

Y_bar = [6.9 ; 8.9]; %Describe equilibrium output 
X_bar = xref;

%continous_time equilibrium input
U_bar_cont = inv(dcgain(Gs))* Y_bar ;

U_bar_dis = inv(dcgain(Gz)) * Y_bar;

%% point 1
Q_LQ = 10*eye(2);
R_LQ = 1;
Klqr_cont=lqr(A,B,Q_LQ,R_LQ)

feedbackSystem_cont = A-B*Klqr_cont;
poles_feedback_cont=eig(feedbackSystem_cont)
% Note that all poles_feedback < 0 and draw the conclusion (asymptotically stable)

Klqr_dis=dlqr(Ad,Bd,Q_LQ,R_LQ)
feedbackSystem_dis = Ad-Bd*Klqr_dis;
poles_feedback_dis=eig(feedbackSystem_dis)
% Note that |poles_feedback| < 1 and draw the conclusion (asymptotically stable)

tsim = 0.1;

%first intial state
x0 = [0 0]';

Vdc  = 650 % volt
umax = [+Vdc/sqrt(3); +Vdc/sqrt(3)]
umin = [-Vdc/sqrt(3); -Vdc/sqrt(3)]

sat_max = umax;
sat_min = umin;


%open('Point1_simulink_z.slx')
%sim('Point 1_simulink_z.slx')
keyboard

Q_LQ = 10*eye(2);
R_LQ = 100;
Klqr_cont=lqr(A,B,Q_LQ,R_LQ)
Klqr_dis=dlqr(Ad,Bd,Q_LQ,R_LQ)
tsim = 0.2
open('Point1_simulink_z.slx')
sim('Point 1_simulink_z.slx')
keyboard

Q_LQ = 1000*eye(2);
R_LQ = eye(2);
Klqr_cont=lqr(A,B,Q_LQ,R_LQ)
Klqr_dis=dlqr(Ad,Bd,Q_LQ,R_LQ)
tsim = 0.02
open('Point1_simulink_z.slx')
sim('Point 1_simulink_z.slx')
keyboard

%% point 2 create the mpc controler
Q_LQ = 1000*eye(2);
R_LQ = eye(2);

tsim = 0.02;
N = 50;
Pr = Q_LQ;

Qsig = blkdiag(kron(eye(N-1),Q_LQ),Pr);
Rsig = kron(eye(N),R_LQ)

% A matrix
Asig = Ad;
for i = 2:N
    Asig = [Asig; Ad^i];
end

%B matrix
Bsig = [];
for i = 1:N
    temp = zeros(size(Bd, 1) * (i - 1), size(Bd, 2));
    for j = 0:N-i
        temp = [temp; Ad^j * Bd];
    end
    Bsig = [Bsig temp];
end

% H, F and S matrices (with Q)
H = Bsig'*Qsig*Bsig + Rsig;
M = Asig'*Qsig*Asig;
F = Asig'*Qsig*Bsig;
% The term 1/2*x_k^T*M*x_k does not contain the control variable so it is
% not needed to solvZe optimization problem: remember that the goal of the
% optimization is to find the optimal control sequence

%open("Point2_simulink_z.slx")
%sim("Point2_simulink_z.slx")

%% point 3
tsim = 0.02;

Q_LQ = 1000*eye(2); % State weight matrix
R_LQ = eye(2);      % Input weight matrix
S = 0;

N1=2;
QMPC=Q_LQ; 
R=R_LQ;
Pr=zeros(2); 

N2=4;
QMPC=Q_LQ; 
R=R_LQ;
Pr=zeros(2); 

N3=50;
QMPC=Q_LQ;
R=R_LQ;
Pr=zeros(2);

open("Point3_simulink_z.slx")
sim("Point3_simulink_z.slx")
keyboard

%% point 4 

tsim = 0.02;

%I changed x0 in order to see better the effect of control saturation
x0 = [15 15]' ;

%We want an optimal N, so I put N = 50 and don't change it in the cases
N1=50;
QMPC1=Q_LQ; %optimal Q
R1=R_LQ;
Pr=zeros(2); 

N2=50;
QMPC2=Q_LQ; %optimal Q
R2=R_LQ;
Pr=zeros(2); 


%Comparison between  LQR and 2 different MPC (with and without saturation)
open("Point4_1_simulink_z.slx")
sim("Point4_1_simulink_z.slx")
keyboard

QMPC1=Q_LQ; %optimal Q
R1=R_LQ;
 
QMPC2=100*Q_LQ; %optimal Q
R2=R_LQ;
 
QMPC3=Q_LQ; %optimal Q
R3=100*R_LQ;

%Comparison between LQR and 3 different MPC
open('Point4_simulink_z.slx')
sim('Point4_simulink_z.slx')
keyboard

%% point 5

x1_max = 15;
x1_min = -15;

x2_max = 15;
x2_min = -15;

xmin = [x1_min; x2_min];
xmax = [x1_max; x2_max];

x0 = [-14 -14]' ;

N=50;
QMPC=Q_LQ; 
R=R_LQ;
Pr=zeros(2);

 
open('Point5_simulink_z.slx')
sim('Point5_simulink_z.slx')
keyboard

%To penelize more means we increse R, so let's try

R1=10*R_LQ;

R2=100*R_LQ;

%Penalizing the input more means increasing the weight R in the cost function
%This typically results in less aggressive control actions since the
%controller will prioritize minimizing control effort over state error

open('Point51_Simulink_z')
sim('Point51_Simulink_z')
keyboard

%% point 6
% Discrete-time system matrices
Ad = Gz.A;
Bd = Gz.B;
Cd = Gz.C;
Dd = Gz.D;

% Extend B and D matrices to include process noise
B_noise = [Bd, ones(size(Bd, 1), 1)];
D_noise = [Dd, zeros(size(Dd, 1), 1)];

%set power of v_x=0.001 in simulink block
power_vx=0.001/100;
%set power of v_y=0.01 in simulink block
power_vy=0.01/100;
 
%Since Q and R are not defined for the Kalman filter, we guess some values
%for them which are also make sense

% set Kalman filter parameters
QK=eye(2);          % variance of v_x
RK=10*QK(1);        % variance of v_y
NK=0;               % covariance v_x,v_y

x0=[0 0]';
x0K=1.5*x0;         %initial guess for x0

N=50;
QMPC=Q_LQ; 
R=R_LQ;
Pr=zeros(2);

%open('Point6_simulink_z.slx')
%sim('Point6_simulink_z.slx')
keyboard

%% Point 7

% the values of the inductances Ld and Lq used in the MPC design are
% different from their actual values in the physical system

Rs = 0.62; % Ohm
we = 785;  % rad/s

% Define real parameters
Ld_r = 0.06; % H (real value)
Lq_r = 0.013; % H (real value)

% Define matrices with real values
A_r = [-Rs/Ld_r, we*Lq_r/Ld_r; -we*Ld_r/Lq_r, -Rs/Lq_r];
B_r = [1/Ld_r, 0; 0, 1/Lq_r];
C_r = [1, 0; 0, 1];
D_r = [0, 0; 0, 0];

% State-space model with real values
Gs_r = ss(A_r, B_r, C_r, D_r);

% Discrete-time state-space model with real values
Gz_r= c2d(Gs_r, Ts);

% Compute poles and zeros for continuous-time system
poles_Gs_r = pzmap(Gs_r);
zeros_Gs_r = tzero(Gs_r);
% Note that Re(poles) < 0 and draw the conclusion (asymptotically stable)

% Compute poles and zeros for continuous-time system
pzmap(Gs)

% Compute poles and zeros for discrete-time system
pzmap(Gz_r);
[poles_Gz_r, zeros_Gz_r] = pzmap(Gz_r);
% Note that |poles| < 1 and draw the conclusion (asymptotically stable)

Y_bar = [6.9; 8.9]; % Desired equilibrium output 

% Compute equilibrium input for the real system
U_bar_disr = inv(dcgain(Gz_r)) * Y_bar;

% Define prediction horizon
N = 50;
% Define LQR weights (tuning parameters)
Q_LQ = 1000 * eye(2); % State weighting matrix
R_LQ = eye(2); % Control weighting matrix
Pr = Q_LQ; % Terminal cost weighting matrix


% Discrete-time system matrices
Ad_r = Gz_r.A;
Bd_r = Gz_r.B;
Cd_r = Gz_r.C;
Dd_r = Gz_r.D;

Klqr_disr=dlqr(Ad_r,Bd_r,Q_LQ,R_LQ)


N=50;
QMPC=100*Q_LQ;
R=R_LQ;
Pr=zeros(2); 

%x0 = [0 0]'
x0 = [15 15]';

open('Point7_simulink_z.slx')
sim('Point7_simulink_z.slx')
keyboard

%% velocity form
[Klqr_dis,S]=dlqr(Ad,Bd,Q_LQ,R_LQ);

du_min = -0.1*ones(2,1); % Example increment limits
du_max = 0.1*ones(2,1);

% Initial conditions
x = [0 0]; % or any initial state
u_prev = [0; 0]; % initial previous input

% Reference
xref = [6.9; 8.9];

% Simulation time
Ts = 1/8000;
Tsim = 0.1; % 100 ms
steps = round(Tsim/Ts);

% Preallocate for logging
X_log = zeros(2, steps+1);
U_log = zeros(2, steps+1);
DU_log = zeros(2, steps);
X_log(:,1) = x;
U_log(:,1) = u_prev;

for k = 1:steps
    % Call your velocity-form MPC function
    du = ZahraMPC7_vel(Ad, Bd, Q_LQ, R_LQ, S, N, du_min, du_max, umin, umax, U_bar_dis, xmin, xmax, xref, x, u_prev, []);
    
    % Update control input
    u = u_prev + du;
    
    % Apply to plant (discrete-time update)
    x = Ad*x + Bd*u;
    
    % Log
    DU_log(:,k) = du;
    U_log(:,k+1) = u;
    X_log(:,k+1) = x;
    
    % Prepare for next step
    u_prev = u;
end

open("Point71_simulink_z.slx")
sim('Point71_simulink_z')
