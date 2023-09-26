close all
%% Results
figure(1)
subplot(3,1,1)
set(gcf,'color','w');
plot(psi_est_1,'r','LineWidth',4);
hold on
plot(psi_1,'b--','LineWidth',4);
grid on;
xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
% ylabel('$$\delta_i$$','Interpreter','LaTeX','Fontsize',12);
d1 = legend('$$\hat{\delta_i}$$','$$\delta_i$$');
d1.Interpreter = "latex";
d1.FontSize = 22;
% d11 = title('Relative position between host and preceding vehicle and its estimation');
% d11.Interpreter = "latex";
% d11.FontSize = 10;

% ax = axes;
% set(ax,'units','normalized','position',[0.2,0.2,0.2,0.2])
% box(ax,'on')
% plot(psi_est_1,'r','LineWidth',1.5,'parent',ax);
% plot(psi_1,'b--','LineWidth',1.5,'parent',ax);
% grid on;
% set(ax,'xlim',[0.0,0.5],'ylim',[5,18])
ylim([0 15]);

subplot(3,1,2)
set(gcf,'color','w');
plot(psi_est_2,'r','LineWidth',4);
hold on
plot(psi_2,'b--','LineWidth',4);
grid on;
xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
% ylabel('\boldmath$\dot{\delta}_i$','Interpreter','LaTeX','Fontsize',12);
d2 = legend('$\dot{\hat{\delta}_i}$','$$\boldmath{\dot{\delta}_i}$$');
d2.Interpreter = "latex";
d2.FontSize = 13;
% d21 = title('Relative velocity between host and preceding vehicle and its estimation');
% d21.Interpreter = "latex";
% d21.FontSize = 10;

% ax = axes;
% set(ax,'units','normalized','position',[0.2,0.2,0.2,0.2])
% box(ax,'on')
% plot(psi_est_2,'r','LineWidth',1.5);
% hold on
% plot(psi_2,'b--','LineWidth',1.5);
% grid on;
% set(ax,'xlim',[0.0,0.05],'ylim',[-5,12])
ylim([-5 10]);

subplot(3,1,3)
set(gcf,'color','w');
plot(psi_est_3,'r','LineWidth',4);
hold on
plot(psi_3,'b--','LineWidth',4);
grid on;
xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
% ylabel('\boldmath$\ddot{\delta}_i$','Interpreter','LaTeX','Fontsize',12);
d3 = legend('$$\ddot{\hat{\delta}_i}$$','$$\boldmath{\ddot{\delta}_i}$$');
d3.Interpreter = "latex";
d3.FontSize = 13;
% d31 = title('Relative acceleration between host and preceding vehicle and its estimation');
% d31.Interpreter = "latex";
% d31.FontSize = 10;
ylim([-15 5 ]);

% ax = axes;
% set(ax,'units','normalized','position',[0.2,0.2,0.2,0.2])
% box(ax,'on')
% plot(psi_est_3,'r','LineWidth',1.5);
% hold on
% plot(psi_3,'b--','LineWidth',1.5);
% grid on;
% set(ax,'xlim',[0.0,0.25],'ylim',[-20,10])


%%
figure(2)
set(gcf,'color','w');
rgb1 = [0.4660 0.6740 0.1880];
rgb2 = [0 0.4470 0.7410];

subplot(2,1,1)

plot(fc,'Color',rgb1,'LineWidth',4);grid on
hold on
plot(fc_est,'b--','LineWidth',4);grid on
xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
ylabel('$$f_{c}$$','Interpreter','LaTeX','Fontsize',12);
d4 = legend('${\hat{f}_c}$','$$f_{c}$$');
d4.Interpreter = "latex";
d4.FontSize = 13;
d41 = title('Cyber-attack and its estimation');
d41.Interpreter = "latex";
d41.FontSize = 22;
ylim([-5 20 ]);





subplot(2,1,2)
plot(a_pre.Time,a_pre.Data, 'b-','LineWidth',4);
hold on 
plot(mu.Time,mu.Data,'r','LineWidth',4);
xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',20);
d51 = title('Inter-Vehicle Comunication');
d51.Interpreter = "latex";
d51.FontSize = 22;

d5 = legend('a_{i-1}' ,'\mu');
d5.Interpreter = "latex";
grid on


ax = axes;
set(ax,'units','normalized','position',[0.2,0.2,0.22,0.2])
grid on
plot(fc.Time,fc.Data,'Color',rgb1,'LineWidth',1.5);
hold on
plot(fc.Time,fc_est.Data,'b--','LineWidth',1.5);grid on
xlabel('Time [sec]', 'Interpreter', 'LaTeX', 'Fontsize', 10);
ylabel('$$f_{c}$$', 'Interpreter', 'LaTeX', 'Fontsize', 10);
set(ax,'xlim',[4.98,5.04],'ylim',[1,2.5])

% figure(3)
% set(gcf,'color','w');
% plot(t,y(1,:),'g','LineWidth',2);
% hold on;
% plot(t,-(xf(1,2:668)-xp(1,2:668)-li),'k','LineWidth',2)
% grid on
% xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
% ylabel({'Inter-vehicle communication'},'Interpreter','LaTeX','FontSize',12)
% d5 = legend('a_{i-1}','\mu','Interpreter','LaTex');
% d5.FontSize = 13;
% d51 = title('True and corrputed transmitted acceleration and its estimation');
% d51.Interpreter = "latex";
% d51.FontSize = 10;