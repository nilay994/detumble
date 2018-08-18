%% plot detumble parameter for verifying the C and matlab detumble algorithms
readMatrix_n = csvread('data4test_Out.csv', 1, 0);
readMatrix_r = csvread('data4test_01.csv', 1, 0);

plot_save1 = figure(1);
subplot(2,1,1);
plot(readMatrix_r(:,13)); hold on; plot(readMatrix_r(:,14)); plot(readMatrix_r(:,15)); grid on;
title('Roberts algorithm');
xlabel('time/sample');
ylabel('p_{tumb}');

subplot(2,1,2);
plot(readMatrix_n(:,13)); hold on; plot(readMatrix_n(:,14)); plot(readMatrix_n(:,15)); grid on;
title('Nilays algorithm');
xlabel('time/sample');
ylabel('p_{tumb}');

%%
plot_save1 = figure(2);
subplot(3,2,1);
plot(readMatrix_r(:,10)); grid on;
title('Roberts algorithm');
xlabel('time/sample');
ylabel('s_{on}');
subplot(3,2,3);
plot(readMatrix_r(:,11)); grid on;
subplot(3,2,5);
plot(readMatrix_r(:,12)); grid on;

subplot(3,2,2);
plot(readMatrix_n(:,10)); grid on;
title('Nilays algorithm');
xlabel('time/sample');
ylabel('s_{on}');
subplot(3,2,4);
plot(readMatrix_n(:,11)); grid on;
subplot(3,2,6);
plot(readMatrix_n(:,12)); grid on;

%% 
plot_save2 = figure(3);
subplot(3,2,1);
plot(readMatrix_r(:,7)); grid on;
title('Roberts algorithm');
xlabel('time/sample');
ylabel('t_{on}');
subplot(3,2,3);
plot(readMatrix_r(:,8)); grid on;
subplot(3,2,5);
plot(readMatrix_r(:,9)); grid on;

subplot(3,2,2);
plot(readMatrix_n(:,7)); grid on;
title('Nilays algorithm');
xlabel('time/sample');
ylabel('t_{on}');
subplot(3,2,4);
plot(readMatrix_n(:,8)); grid on;
subplot(3,2,6);
plot(readMatrix_n(:,9)); grid on;