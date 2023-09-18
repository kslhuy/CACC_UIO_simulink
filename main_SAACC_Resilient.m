clear all; clc;
close all;

global tau h k1 k4 k5 ...
       Li L_des li_1 D ...
       yn_i yn_L mu vi ref u_syn
       
tau = 0.4;
h = 0.5;
k1 = -tau/h;
k4 = 5;
k5 = k4*h; % k2 in the paper

L_des = 7.3;
li_1 = 5;
del = [0 0 k5*(L_des-li_1)/(tau*h)]';
[A,B,F,C,D,E,Ae,Be,Ce,Pz,Q] = model_SAACC(tau, h, k1, k5);
%%
Li = LMI_SACC_PIO_v2();
%%
dt = 0.01;
tf = 20;
%%
xp0 = [10 10 0]; % car i-1
xf0 = [0 0 0];    % car i
xf01 = xf0;
xfn0 = xf0;
xfc0 = xf0;

xps(:,1) = xp0';
xfs(:,1) = xf0';
xf1s(:,1) = xf01';
xfns(:,1) = xfn0';
xfcs(:,1) = xfc0';

xhats(:,1) = [0 0 0 0 0]';
xhat = xhats;
zi0 = xhats';

t = 0;
t0 = t;
y_store = C*xf0';
yL_store = y_store;
fdist(1,1) = 0;
accel_store(1,1) = accel(0);

% Controller parameters
uf = 0;
uf1 = 0;
uf_c = 0;
uf_n = 0;
cntrl_flag = 0;
cnvgc_count = 0;

for i = 1:tf/dt
    ref = accel(i*dt);
    mu = ref + dist(i*dt); % FDI
%     mu = delay_accel(i*dt); % DoS
    rfault = radar_fault(i*dt);
    n_d = noise_dist(i*dt);
    
    % Preceeding vehicle (i-1)
    [tp,xp] = ode45(@sys_pV,[t0 t0+dt],xp0);
    len_tp = length(tp);
    xp0 = xp(len_tp,:);
    
    % Following vehicle (i) without attack
    u_syn = uf;
    [tf,xf] = ode45(@sys_fV,[t0 t0+dt],xf0);
    len_tf = length(tf);
    xf0 = xf(len_tf,:);
    % Following vehicle (i) without attack
    u_syn = uf1;
    [tf1,xf1] = ode45(@sys_fV,[t0 t0+dt],xf01);
    len_tf1 = length(tf1);
    xf01 = xf1(len_tf1,:);
    % Following vehicle (i) with attack, no observer
    u_syn = uf_n;
    [tfn,xfn] = ode45(@sys_fV,[t0 t0+dt],xfn0);
    len_tfn = length(tfn);
    xfn0 = xfn(len_tfn,:);
    % Following vehicle (i) with attack and observer
    u_syn = uf_c;
    [tfc,xfc] = ode45(@sys_fV,[t0 t0+dt],xfc0);
    len_tfc = length(tfc);
    xfc0 = xfc(len_tfc,:);
    
    % Controller w/o attack
    h_ = 0.5;
    ei = [xf0(1)-xp0(1)+L_des;
          xf0(2)-xp0(2)]';
    uf = -k1*ref+(k1+h_*k1*k5)*xf0(3) ...
         -1/h_*(1-h_*k1*k5)*ei(2)-k5/h_*ei(1)-k5*xf0(2); % w/o attack
    % Controller w/o attack (h is varying)
    ei1 = [xf01(1)-xp0(1)+L_des;
          xf01(2)-xp0(2)]';
    uf1 = -k1*ref+(k1+h*k1*k5)*xf01(3) ...
         -1/h*(1-h*k1*k5)*ei1(2)-k5/h*ei1(1)-k5*xf01(2); % w/o attack
    % Controller w/ attack
    h_ = 0.5;
    ei_n = [xfn0(1)-xp0(1)+L_des-n_d;
            xfn0(2)-xp0(2)-rfault]';
    uf_n = -k1*mu+(k1+h_*k1*k5)*xfn0(3) ...
           -1/h_*(1-h_*k1*k5)*ei_n(2)-k5/h_*ei_n(1)-k5*xfn0(2); % w/ attack
    
    % Measurements
    yn_i = [xp0(1,1)-xfc0(1,1)-li_1+n_d;
            xp0(1,2)-xfc0(1,2)+rfault];
    vi = xfc0(1,2);
    y_store(:,i+1) = yn_i;
    
    yn_L = yn_i;
    
    % Observer
    [ti,zi] = ode45(@sys_observerPIO,[t0 t0+dt],zi0);
    len_ti = length(ti);
    zi0 = zi(len_ti,:);
    zii = zi0(1,1:4);
    muu = zi0(1,5);
    xi = zii' + Q*yn_L;
    xhat = [xi;muu];
    
    % Proposed controller
    ei_c = [-yn_i(1,1)-li_1+L_des;
            -yn_i(2,1)]';
    if (i*dt >= 2 && abs(xhat(5)) >= 3 && abs(xhat(4)) < 3) || cntrl_flag == 1
        h = 1;
        uf_c = (h*k1*k5)*mu - k5*xfc0(2) - (k1+h*k1*k5)*xhat(3) ...
                + 1/h*(1-h*k1*k5)*xhat(2) + k5/h*xhat(1) ...
                - k5/h*(L_des-li_1) - (h*k1*k5)*xhat(5);
        cntrl_flag = 1;
    else
        uf_c = -k1*mu+(k1+h*k1*k5)*xfc0(3) ...
               -1/h*(1-h*k1*k5)*ei_c(2)-k5/h*ei_c(1)-k5*xfc0(2); % original
    end
 
    t0 = i*dt;
    t(1,i+1) = t0;
    
    xps(:,i+1) = xp0';
    xfs(:,i+1) = xf0';
    xf1s(:,i+1) = xf01';
    xfns(:,i+1) = xfn0';
    xfcs(:,i+1) = xfc0';
    xhats(:,i+1) = xhat;

    accel_store(1,i+1) = ref;
    yL_store(:,i+1) = yn_L;
    mu_store(1,i+1) = mu;
    rf_store(1,i+1) = rfault;
    nd_store(1,i+1) = n_d;
end
%%
figure
subplot(2,1,1)
hold on
plot(t,y_store(1,:))
plot(t,yL_store(1,:))
grid on
ylabel({'{Meas.}','${[]}$'},'Interpreter','LaTeX')
xlabel('{Time} ${[sec]}$','Interpreter','LaTeX')
set(gca,'FontSize',12,'FontWeight','bold')
subplot(2,1,2)
hold on
plot(t,y_store(2,:))
plot(t,yL_store(2,:))
grid on
%%
figure
subplot(3,1,1)
hold on
plot(t,xps(1,:),'linewidth',2)
grid on
ylabel({'{Distance}','${[m]}$'},'Interpreter','LaTeX')
set(gca,'FontSize',12,'FontWeight','bold')
subplot(3,1,2)
hold on
plot(t,xps(2,:),'linewidth',2)
grid on
ylabel({'{Velocity}','${[m/s]}$'},'Interpreter','LaTeX')
set(gca,'FontSize',12,'FontWeight','bold')
% ylim([25 35])
subplot(3,1,3)
hold on
plot(t,accel_store(1,:),'linewidth',2)
grid on
ylabel({'{Acceleration}','${[m/s^2]}$'},'Interpreter','LaTeX')
xlabel('{Time} ${[sec]}$','Interpreter','LaTeX')
set(gca,'FontSize',12,'FontWeight','bold')
ylim([-2 2])
%%
figure
subplot(3,1,1)
hold on
plot(t,xps(1,:)-xfs(1,:)-li_1,'k','linewidth',2)
plot(t,xps(1,:)-xf1s(1,:)-li_1,'g','linewidth',2)
plot(t,xps(1,:)-xfns(1,:)-li_1,'r:','linewidth',2)
plot(t,xps(1,:)-xfcs(1,:)-li_1,'-.','linewidth',2)

% plot(t,xhats(1,:),'g-.','linewidth',2)
% plot(t,xhatKs(1,:),'y-.','linewidth',2)
threshold = 0;
rdata = xps(1,:)-xfns(1,:)-li_1;
id = cross_threshold(t,rdata,3,threshold,1);
collision_time = t(1,id)
ylabel({'\boldmath$\delta_i$'},'Interpreter','LaTeX', 'Fontsize', 20)
grid on
set(gca,'FontSize',12,'FontWeight','bold')
% ylim([0 20])
ylim([0 25])
% ylim([0 20])

subplot(3,1,2)
hold on
plot(t,xps(2,:)-xfs(2,:),'k','linewidth',2)
plot(t,xps(2,:)-xf1s(2,:),'g','linewidth',2)
plot(t,xps(2,:)-xfns(2,:),'r:','linewidth',2)
plot(t,xps(2,:)-xfcs(2,:),'-.','linewidth',2)
% plot(t,xhats(2,:),'g-.','linewidth',2)
ylabel({'\boldmath$\dot{\delta}_i$'},'Interpreter','LaTeX', 'Fontsize', 20)
grid on
set(gca,'FontSize',12,'FontWeight','bold')
% ylim([-10 10])
% ylim([-10 10])
ylim([-5 10])

subplot(3,1,3)
hold on
plot(t,accel_store-xfs(3,:),'k','linewidth',2)
plot(t,accel_store-xf1s(3,:),'g','linewidth',2)
plot(t,accel_store-xfns(3,:),'r:','linewidth',2)
plot(t,accel_store-xfcs(3,:),'-.','linewidth',2)
% plot(t,xhats(3,:),'g-.','linewidth',2)
% legend('w/o cyber-attack and sensor fault','w/ radar sensor fault','Interpreter','LaTeX')
% legend('w/o cyber-attack and sensor fault','Control w/o observer','Observer based control')
% legend('w/o cyber-attack and sensor fault','w/o cyber-attack and sensor fault (varying $h$)',...
%     'Control w/o observer','Observer based control (varying $h$)','Interpreter','LaTeX')
ylabel({'\boldmath$\ddot{\delta}_i$'},'Interpreter','LaTeX', 'Fontsize', 20)
xlabel('\boldmath{Time} ${[sec]}$','Interpreter','LaTeX')
grid on
set(gca,'FontSize',12,'FontWeight','bold')
ylim([-20 10])

%% False data
figure
subplot(3,1,1)
hold on
plot(t,accel_store,'linewidth',2)
plot(t,mu_store,'r-.','linewidth',2)
legend('$a_{i-1}$','$\mu$','Interpreter','LaTeX')
% xlabel('\boldmath{Time} ${[sec]}$','Interpreter','LaTeX')
grid on
set(gca,'FontSize',12,'FontWeight','bold')
ylabel({'Inter-vehicle'; 'communication'},'FontSize',11)
ylim([-10 35])


subplot(3,1,2)
hold on
plot(t,mu_store-accel_store,'k','linewidth',2)
plot(t,xhats(5,:),'-.','linewidth',2)
threshold = 3;
id = cross_threshold(t,xhats(5,:),3,threshold,2);
detection_time1 = t(1,id)
plot(t,threshold*ones(1,length(t)),'r--','linewidth',1.2)
plot(t,-threshold*ones(1,length(t)),'r--','linewidth',1.2)
% legend('Truth','Estimate')
legend('Truth','Estimate','Threshold')
ylabel({'\boldmath$f_c$'},'Interpreter','LaTeX', 'Fontsize', 20)
% xlabel('\boldmath{Time} ${[sec]}$','Interpreter','LaTeX')
grid on
set(gca,'FontSize',12,'FontWeight','bold')
xlim([0 20])
ylim([-20 35])

subplot(3,1,3)
hold on
plot(t,rf_store(1,:),'k','linewidth',2)
plot(t,xhats(4,:),'-.','linewidth',2)
threshold = 3;
id = cross_threshold(t,xhats(4,:),3,threshold,2);
detection_time2 = t(1,id)
plot(t,threshold*ones(1,length(t)),'r--','linewidth',1.2)
plot(t,-threshold*ones(1,length(t)),'r--','linewidth',1.2)
legend('Truth','Estimate','Threshold')
% legend('$f_{r}$','$\hat{f}_{s}$','Threshold','Interpreter','LaTeX', 'Fontsize', 13)
ylabel({'\boldmath$f_s$'},'Interpreter','LaTeX', 'Fontsize', 20)
xlabel('\boldmath{Time} ${[sec]}$','Interpreter','LaTeX')
grid on
set(gca,'FontSize',12,'FontWeight','bold')
xlim([0 20])
ylim([-15 20])