function [Uls, Urs, min_total_time, curve_length,turnance, omega_dx, delta_x_delta_t] =...
    patheval(dist1, dist2, angle1, angle2, P0, P3, delta_t, angle_norm,...
    top_wheel_speed, axel_len,initial_ul, initial_ur,max_accel, delta_time, plot_on, do_real_wheels)
%Given 2 distance parameters, and information for a begining and end pose,
%this function returns the length of the path and a measure of how much 
%turning is needed determined by angle_norm (1 for l1, 2 for l2...). Also
%returns the turn rate per meter at t vector for further analysis. 
%Additionally, given a speed(turn_rate/dis) func, computes the time
%to complete the path.
%Use plot_on to enable/disable plotting (1 is on)
%Use do_real_wheels to generate an ideal trajectory for the wheels
%velocity (in development!).
%if real_wheels are not enabled, Uls, Urs is the final ideal velocity
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
subplot(6,2,1:4)
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
subplot(6,2,5)
hold on;
stem( t_off, delta_theta_delta_t, 'Marker', 's', 'Color', 'b')
stem( t_off, delta_x_delta_t, 'Marker', 'd', 'Color', 'r')
title('Delta radians/dist at segment')
legend('Delta Rad', 'Delta X')
end
turnance = norm(omega_dx,angle_norm); %maybe l2 norm makes more sense?
%note, depends on speed of vehicle at omega value

%compute path time based on starting speed
speeds = zeros(1,length(omega_dx));
omega_dt = zeros(1, length(omega_dx));
max_left_vels = zeros(1, length(omega_dx));
max_right_vels = zeros(1, length(omega_dx));
times_at_t = zeros(1, length(omega_dx));

%get max possible velocities if max correction is used at each point in the
%path, while maximizing speed
%This is the Robust Control Invarient Set. Invarient sets are sets of
% control inputs and states that keep the constraints satisfied.

%TODO make more breaking torque available than speed increase accel

[max_left_vels, max_right_vels, speeds, omega_dt ,times_at_t] ...
    = generate_velocities_from_path(omega_dx, delta_x_delta_t, axel_len,...
    max_accel, top_wheel_speed);

%TODO add plot for max wheel vel envelopes and chosen wheel vels

%Fit controller inputs to Control Invarient Set (Control Envelope)
%and find time to travel along the path using this ideal control input.
%TODO figure out allocation
if (do_real_wheels)
Uls = [];
Urs = [];
omega_dx_extend = [omega_dx(1),omega_dx, omega_dx(end)];
right_start_speed = initial_ur;
left_start_speed  = initial_ul;
time_per_segment = zeros(1,length(omega_dx));
%delta_time = .02;
for index = 1:length(omega_dx)
    left_max_end_speed = max_left_vels(index);
    right_max_end_speed = max_right_vels(index);

    [uls_t, urs_t] = generate_segment_wheel_trajectory(...
                      omega_dx_extend(index), omega_dx_extend(index+1),...
                      omega_dx_extend(index+2),...
                      delta_x_delta_t(index), axel_len,...
                      max_accel, top_wheel_speed,...
                      left_max_end_speed, right_max_end_speed,...
                      left_start_speed,   right_start_speed, delta_time);
    left_start_speed  = uls_t(end);
    right_start_speed = urs_t(end);
    prev_len = length(Uls);
    Uls = [Uls, uls_t];
    Urs = [Urs, urs_t];
    time_per_segment(index) = length(Uls)*delta_time - prev_len*delta_time;
end
else
    Uls = max_left_vels(end);
    Urs = max_right_vels(end);
end

 robot_speed_ideal = @(omega) max(0,top_wheel_speed - axel_len/2 * abs(omega));
 max_omega = top_wheel_speed * 2 / axel_len;
% %if speed is non-zero, get time from delta_x / speed
% %if speed is zero, get time from delta_theta / max_omega,
% %  where max_omega = top_wheel_speed * 2 / axel_len
% delta_time_delta_t = zeros(1, length(omega_dt));
 driving_time_at_t = zeros(1, length(omega_dt));
 zero_point_turning_time_at_t = zeros(1, length(omega_dt));
 for index = 1:length(omega_dt)
     if speeds(index) > .01 % pick best conditioned one, should be solved
                            % simultaneously 
         driving_time_at_t(index) = ...
             delta_x_delta_t(index) ./ speeds(index);
     else
         zero_point_turning_time_at_t(index) = ...
             abs(delta_theta_delta_t(index)) ./ abs(omega_dt(index));
     end
 end
 delta_time_delta_t = driving_time_at_t + zero_point_turning_time_at_t;
min_total_time = sum(delta_time_delta_t);
%plot rad/s at each point
if (plot_on > 0)
subplot(6,2,6)
stem( t_off, omega_dt, 'Marker', 'd', 'Color', 'b')
hold on
stem( t_off,speeds, 'Marker', '*', 'Color', 'r')
legend('Omega (rad/s)', 'speed (m/s)');
title('Max Radians per second and Max speed at t')
%plot given speed function for reference
subplot(6,2,7)
example_omega = -1.5*max_omega:.01:1.5*max_omega;
plot(example_omega, robot_speed_ideal(example_omega));
hold on
colors = linspace(0,1,length(omega_dt));
scatter(omega_dt, speeds, [], colors);
%TODO add plot of chosen speeds and omegas (per time)
title('Max Speed vs omega (radians per second)')
caxis([0, 1])
colorbar;
subplot(6,2,8)
stem(t_off, delta_time_delta_t,'Marker', '^', 'Color', 'b');
%TODO add plot of actual time at each segment
title('Min Time spent at each segment');
subplot(6,2,9:12)
wheel_plot_time = 0:delta_time:(length(Uls)*delta_time - delta_time);
plot(wheel_plot_time, Uls, wheel_plot_time, Urs);
legend('Uls', 'Urs');
title('Wheel Velocity Trajectory vs time');
xlim([0, wheel_plot_time(end)]);
xlabel('Time (s)')
ylabel('Velocity Command (m/s)')
%plot max speeds too
%get speed limit vs time
if (do_real_wheels)
UlSpeedLim = zeros(1,length(Uls));
UrSpeedLim = zeros(1,length(Urs));
innerdex = 0;
for index = 1:length(omega_dx)
  length_of_time = time_per_segment(index)/delta_time;
  for adex = 1:length_of_time
    innerdex = innerdex + 1;
    UlSpeedLim(innerdex) = max_left_vels(index);
    UrSpeedLim(innerdex) = max_right_vels(index);
  end
end
hold on;
plot(wheel_plot_time, UlSpeedLim,'b--', wheel_plot_time, UrSpeedLim,'r--');
end
    
end
end

