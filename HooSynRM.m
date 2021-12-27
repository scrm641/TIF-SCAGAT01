%% Hoo Control based on LMI
% TIF SynRM Control
% Group 69
clear all
close all
% Inicial Parameters
Ld = 0.230; % d-axis Inductance
dLd = 0.1;
Lq = 0.110 ;% q-axis Inductance
dLq = 0.1;%
TLn = 14; % Load Torque
B = 0.0045; % Viscous damping coefficient 
dB = 0.25 ; % Variation of B (%)
Rs = 2.95;%
dRs = 0.25;%
Jm = 0.04838; 
dJ = 0.25;
npp = 2;
a1 = 3*npp*(Ld-Lq)/2;
% Para probar en simulink
Vd = 35;
Vq = 35; 
ide = 16.9492;
iqe = 1.8613;
thetae = 0.9483;
wre = 0.0001;

% Vd = 35;
% Vq = 35; 
% iqe = Vq/Rs;
% ide = Vd/Rs; 
% wre = 0;
% thetae = 0;
%% Linearized model
syms theta wr id iq 
f1 = -Rs*id/Ld  + Lq*wr*iq/Ld;
f2 = -Rs*iq/Lq  - Ld*wr*id/Lq;
f3 = wr;
f4 = (1/Jm)*(-B*wr  - npp*TLn*sin(theta)+ id*iq*a1); %-TL
fx = [f3;f4;f1;f2];
x = [theta;wr;id;iq];
A = jacobian(fx,x);
Eq1 = [thetae wre ide iqe ];
%ag = double(subs(A,{theta,wr,id,iq},{Eq1}));
ag = [ 0                   ,  1          , 0          , 0         ;... 
      -npp/Jm*TLn*cos(wre) , -B/Jm       , a1/Jm*iqe  , a1/Jm*ide ;... 
       0                   ,  iqe*Lq/Ld  , -Rs/Ld     , wre*Lq/Ld ;...
       0                   , -ide*Ld/Lq  , wre*Ld/Lq  , -Rs/Lq]   ;
bg = [0 0;0 0;1/Ld 0;0 1/Lq];
cg = [0 0 0 1; 1 0 0 0]; 
dg = [0 0; 0 0];
G = ss(ag,bg,cg,dg);
G.InputName = { 'Vd','Vq' }; 
G.OutputName = { 'I_q',',\omega_r',};
clf
% Plotting OpenLoop System 
figure(1)
% [u,t] = gensig("square",5,20);
% u1 = 100*u;
% u2 = 1*u;

x0 = [0 0 4 4];
t = 0:0.001:10;
In = 4;
speed = 25;
u1 =  (In*ones(size(t)))';
u2 = speed*mod(t,2*pi)';
u = [u1 u2]; % 10 100
lsim(G,u,t),title('Respuesta en Lazo abierto SynRM')


%% Establishing Pondering functions
clc
%Ms = 2; wb = 100; epsi = 0.6; %v1
%Ms = 2; wb = 3; epsi = 0.01;  % Test full
%Mu = 500; wbc = 8000; epsi1 = 0.6;
%Mu = 25; wbc = 1000; epsi1 = 0.01; % Test full
Ms = 2; wb = 3; epsi = 0.01;
We = tf([1/Ms wb],[1 wb*epsi]);
Mu = 25; wbc = 1000; epsi1 = 0.01;
Wu = tf([1 wbc/Mu],[epsi1 wbc]); %v1
%--------------------------------------------------------------------------
We = mdiag(We,We);
Wu = mdiag(Wu,Wu);
Wb = mdiag(tf([1],[1 0]),1);
% Augmented Plant
systemnames = 'G We Wu';
inputvar = '[r(2) ; u(2) ]';
outputvar = '[We;Wu;  r - G]';
input_to_G = '[u]';
input_to_We = '[r-G]';
input_to_Wu = '[u]';
sysoutname = 'P';
cleanupsysic = 'yes';
sysic;

% Synthesis of the H infinite controller
nmeas = 2; nu = 2;
opts = hinfsynOptions('Method','LMI','Display','on');%% xD change RIC to use ricatti eq
[K,CL,GAM,INFO] = hinfsyn(P,nmeas,nu,opts);

%% Get sensibility function
L1 = G*K;
I = eye(size(L1));
S1 = feedback(I,L1); % S = inv(I+L1);
T1 = I - S1;

%% Numeric Response;
figure(2)
T1.OutputName = {'i_q', '\omega_r' };
T1.C = [0 0 1 0 T1.C(1,5:12);0 1 T1.C(2,3:12)]; 
lsim(T1,u,t)
title('Step Response of SynRM controller via LMI Approach')

% Frequency Response
figure(3)
sigma(I+L1,'--',T1,':',L1,'r--',We/GAM,'k--',GAM/Wu,'k-.',{0.1,100})
legend('1/\sigma(S) performance', '\sigma(T) robustness', '\sigma(L) loopshape',...
       '\sigma(W1) performance bound', '\sigma(1/W3) robustness bound');
   
   

