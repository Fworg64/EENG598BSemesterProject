function [total_time, curve_length,turnance, omega_dx, delta_x_delta_t] =...
    patheval(dist1, dist2, angle1, angle2, P0, P3, delta_t, angle_norm,...
    top_wheel_speed, axel_len,initial_ul, initial_ur,max_accel, plot_on)
%Given 2 distance parameters, and information for a begining and end pose,
%this function returns the length of the path and a measure of how much 
%turning is needed determined by angle_norm (1 for l1, 2 for l2...). Also
%returns the turn rate per meter at t vector for further analysis. 
%Additionally, given a speed(turn_rate/dis) func, computes the time
%to complete the path.
dt = delta_t;
t = 0:dt:1;

P1 = P0 + dist1*[cos(angle1);sin(angle1)];
P2 = P3 - dist2*[cos(angle2);sin(angle2)];

curve = (1 - t).^3 .* P0 ...
        + 3*(1-t).^2 .*t .* P1 ...
        + 3*(1-t).*t.^2 .* P2 ...
        + t.^3 .* P3;
%figure()
if (plot_on > 0)
clf
subplot(4,2,1:4)
plot(curve(1,:), curve(2,:))
title('Path Visual')
hold on;
plot(P1(1), P1(2), 'g*', [P0(1),P1(1)], [P0(2),P1(2)], 'g--')
plot(P2(1), P2(2), 'r*', [P2(1),P3(1)], [P2(2),P3(2)], 'r--')
end
%distance via nasty piecewise integration
curve_length = 0;
delta_x_delta_t = zeros(1,length(t)-1);
for index = 2:length(t)
  delta_x_delta_t(1,index-1) = norm(curve(:,index) - curve(:,index-1),2);
end
curve_length = sum(delta_x_delta_t);

%turnance, want change in angle at each step, not acceleration
first_der = 3*(1 - t).^2 .*(P1 - P0) ...
          + 6*(1-t).*t.*(P2 - P1) ...
          + 3*t.^2.*(P3 - P2);
theta = atan2(first_der(2,:), first_der(1,:));
delta_theta_delta_t = angleDiff(theta(2:end), theta(1:(length(theta)-1)));
t_off = t(1:length(t)-1) + dt/2;
%this is rate of change of theta at each t_off
omega_dx = delta_theta_delta_t ./ delta_x_delta_t;
if (plot_on > 0)
subplot(4,2,5)
stem( t_off, delta_theta_delta_t, 'Marker', 's', 'Color', 'b')
title('Delta radians at segment')
end
turnance = norm(omega_dx,angle_norm); %maybe l2 norm makes more sense?
%note, depends on speed of vehicle at omega value

%compute path time based on starting speed
speeds = zeros(1,length(omega_dx));
omega_dt = zeros(1, length(omega_dx));
max_left_vels = zeros(1, length(omega_dx));
max_right_vels = zeros(1, length(omega_dx));
times_at_t = zeros(1, length(omega_dx));

prev_ul = initial_ul;
prev_ur = initial_ur;
max_accel_abs = .1;
max_wheel_speed = 1;

%get max possible velocities if max correction is used at each point in the
%path, while maximizing speed
%This is the Robust Control Invarient Set. Invarient sets are sets of
% control inputs and states that keep the constraints satisfied.

%TODO make more breaking torque available than speed increase accel

[max_left_vels, max_right_vels, speeds, omega_dt ,times_at_t] ...
    = generate_velocities_from_path(omega_dx, delta_x_delta_t, axel_len,...
    max_accel_abs, max_wheel_speed);

%TODO add plot for max vel envelopes

%TODO Fit controller inputs to Control Invarient Set (Control Envelope)
%and find time to travel along the path using this ideal control input.

 robot_speed_ideal = @(omega) max(0,top_wheel_speed - axel_len/2 * abs(omega));
 max_omega = top_wheel_speed * 2 / axel_len;
% %if speed is non-zero, get time from delta_x / speed
% %if speed is zero, get time from delta_theta / max_omega,
% %  where max_omega = top_wheel_speed * 2 / axel_len
% delta_time_delta_t = zeros(1, length(omega_dt));
 driving_time_at_t = zeros(1, length(omega_dt));
 zero_point_turning_time_at_t = zeros(1, length(omega_dt));
 for index = 1:length(omega_dt)
     if speeds(index) > .01 % should this just be the max of either?
         driving_time_at_t(index) = ...
             delta_x_delta_t(index) ./ speeds(index);
     else
         zero_point_turning_time_at_t(index) = ...
             abs(delta_theta_delta_t(index)) ./ abs(omega_dt(index));
     end
 end
 delta_time_delta_t = driving_time_at_t + zero_point_turning_time_at_t;
total_time = sum(delta_time_delta_t);
%plot rad/s at each point
if (plot_on > 0)
subplot(4,2,6)
stem( t_off, omega_dt, 'Marker', 'd', 'Color', 'b')
hold on
stem( t_off,speeds, 'Marker', '*', 'Color', 'r')
legend('Omega (rad/s)', 'speed (m/s)');
title('Radians per second and speed at t')
%plot given speed function for reference
subplot(4,2,7)
example_omega = -1.5*max_omega:.01:1.5*max_omega;
plot(example_omega, robot_speed_ideal(example_omega));
hold on
colors = linspace(1,10,length(omega_dt));
scatter(omega_dt, speeds, [], colors);
title('Speed vs omega (radians per second)')
subplot(4,2,8)
stem(t_off, delta_time_delta_t,'Marker', '^', 'Color', 'b');
title('Time spent at each segment');
end
end
