% plot detumble parameter for verifying the C and matlab detumble algorithms

readMatrix_r = csvread('data4test_03_in.csv', 1, 0);
readMatrix_pc = csvread('data4test_03_out_pc.csv', 1, 0);
readMatrix_msp = csvread('data4test_03_out_msp.csv', 1, 0);

% to be able to plot incomplete HIL tests, resize everyone to MSP432's
% iterates
len = size(readMatrix_msp,1);
readMatrix_r = readMatrix_r(1:len,:);
readMatrix_pc = readMatrix_pc(1:len,:);

%% TUMBLING PARAMETER

plot_save1 = figure(1);
pu1 = subplot(3,1,1);
plot(readMatrix_r(:,13)); hold on; plot(readMatrix_r(:,14)); plot(readMatrix_r(:,15)); grid on;
title('p_{tumb} - MATLAB');
xlabel('time/sample');

pu2 = subplot(3,1,2);
plot(readMatrix_pc(:,13)); hold on; plot(readMatrix_pc(:,14)); plot(readMatrix_pc(:,15)); grid on;
title('p_{tumb} - pc');
xlabel('time/sample');

pu3 = subplot(3,1,3);
plot(readMatrix_msp(:,13)); hold on; plot(readMatrix_msp(:,14)); plot(readMatrix_msp(:,15)); grid on;
title('p_{tumb} - msp');
xlabel('time/sample');

linkaxes([pu1,pu2,pu3], 'xy');
saveas(plot_save1,'cmp1','jpg');

%% T_ON (time) for MTQ 

plot_save2 = figure(2);
py1 = subplot(3,3,1);
plot(readMatrix_r(:,10)); grid on;
title('s_{on} - MATLAB');
xlabel('time/sample');
py4 = subplot(3,3,4);
plot(readMatrix_r(:,11)); grid on;
py7 = subplot(3,3,7);
plot(readMatrix_r(:,12)); grid on;

py2 = subplot(3,3,2);
plot(readMatrix_pc(:,10)); grid on;
title('s_{on} - pc');
xlabel('time/sample');
py5 = subplot(3,3,5);
plot(readMatrix_pc(:,11)); grid on;
py8 = subplot(3,3,8);
plot(readMatrix_pc(:,12)); grid on;

py3 = subplot(3,3,3);
plot(readMatrix_msp(:,10)); grid on;
title('s_{on} - msp');
xlabel('time/sample');
py6 = subplot(3,3,6);
plot(readMatrix_msp(:,11)); grid on;
py9 = subplot(3,3,9);
plot(readMatrix_msp(:,12)); grid on;

saveas(plot_save2,'cmp2','jpg');
linkaxes([py1,py2,py3,py4,py5,py6,py7,py8,py9],'xy');

%% S_ON (direction) for MTQ

plot_save3 = figure(3);
pt1 = subplot(3,3,1);
plot(readMatrix_r(:,7)); grid on;
title('t_{on} - MATLAB');
xlabel('time/sample');
pt4 = subplot(3,3,4);
plot(readMatrix_r(:,8)); grid on;
pt7 = subplot(3,3,7);
plot(readMatrix_r(:,9)); grid on;

pt2 = subplot(3,3,2);
plot(readMatrix_pc(:,7)); grid on;
title('t_{on} - pc');
xlabel('time/sample');
pt5 = subplot(3,3,5);
plot(readMatrix_pc(:,8)); grid on;
pt8 = subplot(3,3,8);
plot(readMatrix_pc(:,9)); grid on;

pt3 = subplot(3,3,3);
plot(readMatrix_msp(:,7)); grid on;
title('t_{on} - msp');
xlabel('time/sample');
pt6 = subplot(3,3,6);
plot(readMatrix_msp(:,8)); grid on;
pt9 = subplot(3,3,9);
plot(readMatrix_msp(:,9)); grid on;

linkaxes([pt1,pt2,pt3,pt4,pt5,pt6,pt7,pt8,pt9],'xy');
saveas(plot_save3,'cmp3','jpg');

%% COUNTs for detumbling/tumbling
plot_save4 = figure(4);

pv1 = subplot(2,3,1);
plot(readMatrix_r(:,16)); grid on;
title('c_{tumb} - MATLAB');
pv4 = subplot(2,3,4);
plot(readMatrix_r(:,17)); grid on;
title('c_{detumb} - MATLAB');

pv2 = subplot(2,3,2);
plot(readMatrix_pc(:,16)); grid on;
title('c_{tumb} - pc');
pv5 = subplot(2,3,5);
plot(readMatrix_pc(:,17)); grid on;
title('c_{detumb} - pc');

pv3 = subplot(2,3,3);
plot(readMatrix_msp(:,16)); grid on;
title('c_{tumb} - msp');
pv6 = subplot(2,3,6);
plot(readMatrix_msp(:,17)); grid on;
title('c_{detumb} - msp');

linkaxes([pv1,pv2,pv3,pv4,pv5,pv6], 'xy');
saveas(plot_save4,'cmp4','jpg');