%% plot detumble parameter for verifying the C and matlab detumble algorithms

readMatrix_r = csvread('data4test_03_in.csv', 1, 0);
readMatrix_n = csvread('data4test_03_out.csv', 1, 0);

plot_save1 = figure(1);
pu1 = subplot(2,1,1);
plot(readMatrix_r(:,13)); hold on; plot(readMatrix_r(:,14)); plot(readMatrix_r(:,15)); grid on;
title('Roberts algorithm');
xlabel('time/sample');
ylabel('p_{tumb}');

pu2 = subplot(2,1,2);
plot(readMatrix_n(:,13)); hold on; plot(readMatrix_n(:,14)); plot(readMatrix_n(:,15)); grid on;
title('Nilays algorithm');
xlabel('time/sample');
ylabel('p_{tumb}');
linkaxes([pu1,pu2], 'xy');
saveas(plot_save1,'cmp1','jpg');

%%
plot_save2 = figure(2);
py1 = subplot(3,2,1);
plot(readMatrix_r(:,10)); grid on;
title('Roberts algorithm');
xlabel('time/sample');
ylabel('s_{on}');
py3 = subplot(3,2,3);
plot(readMatrix_r(:,11)); grid on;
py5 = subplot(3,2,5);
plot(readMatrix_r(:,12)); grid on;

py2 = subplot(3,2,2);
plot(readMatrix_n(:,10)); grid on;
title('Nilays algorithm');
xlabel('time/sample');
ylabel('s_{on}');
py4 = subplot(3,2,4);
plot(readMatrix_n(:,11)); grid on;
py6 = subplot(3,2,6);
plot(readMatrix_n(:,12)); grid on;
saveas(plot_save2,'cmp2','jpg');
linkaxes([py1,py2,py3,py4,py5,py6],'xy');
%% 
plot_save3 = figure(3);

pt1 = subplot(3,2,1);
plot(readMatrix_r(:,7)); grid on;
title('Roberts algorithm');
xlabel('time/sample');
ylabel('t_{on}');

pt3 = subplot(3,2,3);
plot(readMatrix_r(:,8)); grid on;

pt5 = subplot(3,2,5);
plot(readMatrix_r(:,9)); grid on;

pt2 = subplot(3,2,2);
plot(readMatrix_n(:,7)); grid on;
title('Nilays algorithm');
xlabel('time/sample');
ylabel('t_{on}');

pt4 = subplot(3,2,4);
plot(readMatrix_n(:,8)); grid on;
pt6 = subplot(3,2,6);
plot(readMatrix_n(:,9)); grid on;
linkaxes([pt1,pt2,pt3,pt4,pt5,pt6],'xy');

saveas(plot_save3,'cmp3','jpg');

%%
plot_save4 = figure(4);

pv1 = subplot(2,2,1);
plot(readMatrix_r(:,16)); grid on;
title('Roberts c_{tumb}');
pv3 = subplot(2,2,3);
plot(readMatrix_r(:,17)); grid on;
title('Roberts c_{detumb}');
pv2 = subplot(2,2,2);
plot(readMatrix_n(:,16)); grid on;
title('Nilays c_{tumb}');
pv4 = subplot(2,2,4);
plot(readMatrix_n(:,17)); grid on;
linkaxes([pv1,pv2,pv3,pv4], 'xy');
title('Nilays c_{detumb}');
saveas(plot_save4,'cmp4','jpg');