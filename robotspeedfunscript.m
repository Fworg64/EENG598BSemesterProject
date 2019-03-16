
delta_t = .01;
angle_norm_type = 'inf'; %l2 norm
top_wheel_speed = 1.0; %m/s
axel_len = .62; %meters


%robot_speed = @(omega) exp(-(abs(omega).^3));
%robot_speed = @(omega) top_wheel_speed * exp(-(abs(omega*(1.4*axel_len)).^3));

angle1 = pi/2;
P3 = [2;-1];
angle2 = pi/2; %;+.2618;

dist1 = .01;
dist2 =	.01;

initial_ul = 1;
initial_ur = 1;
max_accel = 2.2; % m/s^2

plot = 1;

figure()
tic
[total_time, curve_length, turnance, omega_dx, delta_x_delta_t] = ...
    patheval(dist1,dist2,angle1,angle2, [0;0],P3,delta_t, angle_norm_type, ...
    top_wheel_speed, axel_len, initial_ul, initial_ur, max_accel, plot);
toc
curve_length
turnance
total_time 
