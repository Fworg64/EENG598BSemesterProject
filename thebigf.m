function [total_time] = thebigf(ControlPoints)

%thebigf
%calculates the total time required to traverse a path defined by the
%control points given as arguments.
%Control points are organized into [delta_x1, delta_y1, delta_theta1;
%                                   delta_x2, delta_y2, delta_theta2;...]

delta_t = .01;
delta_time = .02; %doesnt really matter yet
angle_norm_type = 'Inf';

top_wheel_speed = 1.0;
axel_len = .62;

max_accel = .4;
plot_on = 0;

use_real_wheels =0;

%need to optimize over these two vars for each CP
%d1 = 1.01;
%d2 = 1.01;

function [path_time_eval, Uls, Urs] = shorthand_path(x, CP, initial_ul, initial_ur)
    d1 = x(1);
    d2 = x(2);
    [Uls, Urs, min_time, curve_length, ...
        turnance, omega_dx, delta_x_delta_t] = ...
        patheval(d1,d2,...
        0,CP(3), [0;0],[CP(1); CP(2)], delta_t, angle_norm_type, ...
        top_wheel_speed, axel_len,initial_ul, initial_ur, max_accel,delta_time, plot_on, use_real_wheels);
    path_time_eval = min_time;
end

initial_ul = 0;
initial_ur = 0;
[r, c] = size(ControlPoints);
total_time = 0;

for index = 1:r
    CP = ControlPoints(index, :);
    fun = @(x) shorthand_path(abs(x), CP, initial_ul, initial_ur);
    x = fminsearch(fun, [.5,.5]);
    x = abs(x)
    [path_time_eval, Uls, Urs] = shorthand_path(x, CP, initial_ul, initial_ur);
    initial_ul = Uls(end);
    initial_ur = Urs(end);
    %just add min times for now
    total_time = total_time + path_time_eval; %otherwise use time from length of Uxs and delta_time
end

end
