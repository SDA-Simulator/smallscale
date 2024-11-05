%% MAE 6060 Homework 7
%% Spencer Bullen

% Set up a rigid spinning spacecraft
I=[3000, 0, 0;
   0, 4000, 0;
   0, 0, 5000];

Id = eye(3)*100;
I = I-Id;
invI = inv(I);
invId = inv(Id);
c = .00100;

% For Time-Domain Solution
wbn_init=pi/30*[0 0 .01]'; wdn_init=wbn_init;
q_init=[0 0 0 1]';

% These gains are designed for infinitesimal errors.  The high rate at t=0
% causes a nonlinear response that nonetheless settles (per Lyapunov)
Kp = 15*20000;     
Kd = 250*1500;    

% Wheel array
Iwi=100;  % arbitrary (not given)
ww0=[0 0 0 0].';

% Given wheel Jacobian
A=[eye(3) -sqrt(3)/3*[1 1 1]' ]*Iwi;
pinvA=pinv(A);

% Wheel limits
wmax=150/Iwi;      % 150 Nms max momentum
accelmax=2/Iwi;        % 2 Nm max torque