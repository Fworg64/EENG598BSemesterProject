
delta_t = .01;
angle_norm_type = 'inf'; %l2 norm
top_wheel_speed = 1.0; %m/s
axel_len = .62; %meters

angle1 = 0;
P3 = [3.5939; 2.3515]%P3%idealCPs(1,1:2)%[3.98;-.0671];
angle2 = -.065; %;+.2618;

dist1 = 1.1486;
dist2 =	.0195;

initial_ul = -1;
initial_ur = 1;
max_accel = .4; % m/s^2 %wheels can get to 1 m/s in 2.5 s
delta_time = .02; %50 Hz control system

do_plot = 1;
do_real_wheels = 1;

figure()
tic
[Uls, Urs, min_total_time, curve_length, turnance, omega_dx, delta_x_delta_t] = ...
    patheval(dist1,dist2,angle1,angle2, [0;0],P3,delta_t, angle_norm_type, ...
    top_wheel_speed, axel_len, initial_ul, initial_ur, max_accel, delta_time,...
    do_plot,do_real_wheels);
toc
curve_length
turnance
min_total_time 
