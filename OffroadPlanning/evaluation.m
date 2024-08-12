% Load data
load('recording.mat')
x1 = data{1}.Values.Data;
x2 = data{2}.Values.Data;
x3 = data{3}.Values.Data;
x4 = paths_offset{3,1}(:, 1);
y1 = data{4}.Values.Data;
y2 = data{5}.Values.Data;
y3 = data{6}.Values.Data;
y4 = paths_offset{3,1}(:, 2);


% Plot
figure;
plot(x1, y1, 'k', 'LineWidth', 2);% ground truth trajectory
hold on;
% plot(x2, y2, 'r', 'LineWidth', 2);% amcl trajctory
% plot(x3, y3, 'b', 'LineWidth', 2);% odom trajctory
plot(x4, y4, 'g', 'LineWidth', 2);% planned trajctory
% legend('ground truth', 'amcl', 'odometry', 'planned');
legend('ground truth', 'planned');
xlabel('X(m)');
ylabel('Y(m)');
title('Trajctory Evalutation');
grid on;
hold off;