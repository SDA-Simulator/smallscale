%
clear all
close all
%
I=[3000 0 0
   0 4000 0
   0 0 5000]; 
invI=inv(I);  % precomputing this inverse eliminates needless computation in the sim
%
% For Time-Domain Solution
wbn_init=pi/30*[0 0 0]';
q_init=[0 0 0 1]'; q_init=q_init/norm(q_init);
%
% Control gains
wn=0.01;               % closed-loop frequency, rad/sec
zet=.7071;             % closed-loop damping ratio
Kp=2*wn^2*I;           % The factor of 2 accounts for the fact that the quaternion components are roughly half the angle of rotation
Kd=2*zet*wn*I;
%
% Commanded attitude and rate
sim('Simulator_GUI_Updated_2')
%