clear all
close all
clc

Ts = 0.01;
t = 0:Ts:20;
Nt = length(t);
I3 = eye(3);
%% System and Controller parameters
tau = 0.4;
li = 5;
Li = 8;
h = 0.5;
k1 = -0.8;
k2 = 2.5;
k3 = (1-h*k1*k2);
ai_1 = 3*sin(2*pi/10*(t));
%% Continuous-time model of the following vehicle(usync as input)
Ai = [0 1 0;0 0 1;0 0 -1/tau];
Bi = [0;0;1/tau];
Ci = zeros(1,3);
Di = zeros(1,1);
% Descritization of the c-state space model
Aid = (eye(length(Ai))+Ts*Ai);
Bid = Ts*Bi;
Cid = Ci;
Did = Di;
%% Continuous-time model of the preceeding vehicle(acceleration as input
Ai_1 = [0 1 0;0 0 0;0 0 0];
Bi_1 = [0;1;0];
Ci_1 = zeros(1,3);
Di_1 = zeros(1,1);
% discretization of the continuous model of the preceeding vehicle
Ai_1d = (eye(length(Ai_1))+Ts*Ai_1);
Bi_1d = Ts*Bi_1;
Ci_1d = Ci_1;
Di_1d = Di_1;
%% Continuous-time SA-ACC system
Ac = [0 1 0;0 0 1;-k2/(tau*h) -k3/(tau*h) (k1-k3)/(tau)];
Bc = [0;0;k2/tau];
Fc = [0;0;k3/tau];
Wc = [0;0;(k3-k1)/tau];
deltac = [0;0;k2*(Li-li)/(tau*h)];
Cc = [1 0 0;0 1 0];
%% Discretization
Ad = (eye(length(Ac))+Ts*Ac);
Bd = Ts*Bc;
Fd = Ts*Fc;
Wd = Ts*Wc;
deltad = Ts*deltac;
Cd = Cc;

%% Verifying rank condition
disp('Before transformation')
if rank(Cd*Fd)<rank(Fd)
    disp('Unknown input observer does not exist')
else
    disp('Unknown input observer exists')
end
%% Descriptor form
E = [eye(length(Ad)) Wd];
Ae = [Ad zeros(3,1)];
Be = Bd;
Fe = Fd;
deltae = deltad;
Cbar = Cd*Ad;
Ce = [Cd*Ad zeros(2,1)];
% Rank verification
disp('After transformation')
if rank(Cbar*Wd)<rank(Wd)
    disp('Unknown input observer does not exist')
else
    disp('Unknown input observer exists')
end
% check if rank([E Ce]') is full
if rank([E;Ce])==4
    disp('[E;Ce] is invertible')
else
    disp('[E;Ce] is singular')
end
%% Unknown input observer for the descriptor system
g_invEC = inv([E;Ce]'*[E;Ce])*[E;Ce]';
Pz = g_invEC(1:4,1:3);
Qz = g_invEC(:,4:5);
M = Pz*Be;
G = Pz*Fe;
% Observer gain computation
p = 2;          % Number of outputs
Az = Pz*Ae;     
nx = length(Az);
% Check the detectability of the pair (Pz*Ae,Ce)
if rank(obsv(Pz*Ae,Ce))<=length(Pz*Ae) || abs(eig(Pz*Ae))<eye(4)
    x = length(Pz*Ae)-rank(obsv(Pz*Ae,Ce));
    x1 =  'The pair (Pz*Ae,Ce) is detectable and the number of unobservable modes is %4.2f';
    fprintf(x1,x);   
else
    disp('System is not detectable')
end

P = sdpvar(nx,nx);
X = sdpvar(nx,p);
% a = 0.01;
alpha = 20;
LMI = [Az'*P*Az-Az'*X*Ce-Ce'*X'*Az-P,  X*Ce; Ce'*X' , -P ];
LMI2 = [-alpha*eye(p) X';X -P];
pb = [P>=0];
pb = pb+[LMI<=0];
pb = pb+[LMI2<=0];
solvesdp(pb);
Ps = double(P)
Xs = double(X)
K = inv(Ps)*Xs
N = Az-K*Ce
poles = eig(N)
L = K+N*Qz
% 
% %% System simulation
% % Preceeding vehicle
% xp(:,1) = [10 10 0];
% xf(:,1) = [0 0 0];
% psi(:,1) = [xp(1,1)-xf(1,1)-li xp(2,1)-xf(2,1) xp(3,1)-xf(3,1)];
% w(:,1) = [0 0 0 0];
% y(:,1) = Cd*psi(:,1);
% ksi_hat(:,1) = w(:,1)+Qz*y(:,1);
% fc_hat(1) = 0.5;
% for i=1:length(t)
%     ai_1(i) = 3*sin(2*pi/10*t(i));
%     if t(i)<4
%         fc(i) = 0;
%     elseif t(i)>=4 && t(i)<8
%         fc(i) = 2*sin(10*pi/2*t(i))*exp(-0.01*(t(i)));
%     elseif t(i)>=9 && t(i)<16
%         fc(i) = exp(-2*sin(10*pi/2*t(i))*0.01*(t(i)));
%     else
%         fc(i) = 0;
%     end
%     %% Simulation of the preceeding vehicle
%     xp(:,i+1) = Ai_1d*xp(:,i)+Bi_1d*ai_1(i);
%     mu(i) = ai_1(i)+fc(i);
%     %% SAACC Model
%     psi(:,i+1) = Ad*psi(:,i)+Bd*xf(2,i)+Fd*mu(i)-Wd*fc(i)+deltad;
%     y(:,i) = Cd*psi(:,i);
%     if i<3
%     w(:,i+1) = w(:,1);
%     ksi_hat(:,i) = w(:,1)+Qz*y(:,1);
%     else
%         w(:,i+1) = N*w(:,i)+L*y(:,i)+M*xf(2,i-1)+G*mu(i-1)+Pz*deltad;
%         ksi_hat(:,i) = w(:,i)+Qz*y(:,i);
%     end
%     %
%     %% Simulation of the following vehicle
%     % Control law for SA-ACC
%     usyn(i) = h*k1*k2*mu(i)-(k1+h*k1*k2)*fc(i)-k2*xf(2,i)-...
%         (k1+h*k1*k2)*ksi_hat(3,i)+k3/h*ksi_hat(2,i)+k2/h*ksi_hat(1,i)-k2/h*(Li-li);
%     xf(:,i+1) = Aid*xf(:,i)+Bid*usyn(i);
% end

