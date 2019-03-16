%how to derive the speed function for the robot

%given that wheels have top speed of X
%what is top speed for turn rate of W?
%for perfect differential drive with
top_wheel_speed = .7; %m/s
axel_len = .5; %meters
robot_speed_ideal = @(omega) max(0,top_wheel_speed - axel_len/2 * abs(omega));
%omega=0 is both wheels full speed same direction
%speed=0 is a zero point turn

robot_speed_est = @(omega) top_wheel_speed * exp(-(abs(omega*(1.4*axel_len)).^3));

%figure()
clf
example_omega = -6:.01:6;
plot(example_omega, robot_speed_ideal(example_omega));
hold on;
plot(example_omega, robot_speed_est(example_omega));