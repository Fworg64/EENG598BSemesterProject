%this scripts plots total time as height and turnance as color
%while x and y are the control variables

delta_t = .01;
angle_norm_type = 2; %l2 norm
%robot_speed = @(omega) exp(-(abs(.2*omega).^5));
%for perfect differential drive with
top_wheel_speed = 1.0; %m/s
axel_len = .62; %meters
%robot_speed = @(omega) top_wheel_speed * exp(-(abs(omega*(1.5*axel_len)).^2));
%robot_speed = @(omega) top_wheel_speed * exp(-(abs(omega*(1.4*axel_len)).^3));

angle1 = pi/2;
P3 = [2;-1];
angle2 = pi/2;%pi/2; %;+.2618;

initial_ul = 1;
initial_ur = 1;
max_accel = 4.0; %m/s^2

x1 = .01:.1:6;
x2 = .01:.1:6;
total_time_plot = zeros(length(x1),length(x2));
turnance_plot = zeros(length(x1), length(x2));
tic
parfor (index1 = 1:length(x1),length(x1))
    xx_1 = x1(index1);
    total_time_plot_row = zeros(1,length(x2));
    turnance_plot_row = zeros(1,length(x2));
    for index2 = 1:length(x2)
        [total_time_plot_row(index2), curve_length, ...
        turnance_plot_row(index2), omega_dx, delta_x_delta_t] = ...
        patheval(xx_1,x2(index2),...
        angle1,angle2, [0;0],P3,delta_t, angle_norm_type, ...
        top_wheel_speed, axel_len,initial_ul, initial_ur, max_accel, 0);
    end
    total_time_plot(index1, :) = total_time_plot_row;
    turnance_plot(index1, :) = turnance_plot_row;
end
toc

figure()
surf(x1, x2, total_time_plot, turnance_plot)
cb = colorbar; 
caxis([min(min(turnance_plot)), 4*(min(min(turnance_plot)) + 15)]);
xlabel('dist_2')
ylabel('dist_1')
zlabel('Time (s)')
cb.Label.String = 'Turnance';

[min_val, ind] = min(total_time_plot(:));
[row, col] = ind2sub(size(total_time_plot), ind);
fprintf("Min of %.2f (s) found at:\n dist1 = %.2f \n dist2 = %.2f \n", ...
        min_val, x1(row), x2(col));
