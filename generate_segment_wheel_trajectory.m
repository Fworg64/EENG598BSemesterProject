function [left_vels, right_vels] = ...
         generate_segment_wheel_trajectory(omega_dx, delta_x_delta_t,...
                                           axel_len, max_accel_abs,...
                                           max_abs_left_wheel_speed,...
                                           max_abs_right_wheel_speed, ...
                                           max_end_left_speed, ...
                                           max_end_right_speed, ...
                                           starting_left_speed, ...
                                           starting_right_speed, dt]
%  Generate segment wheel trajectory
% Given the current segments omega_dx, delta_x_delta_t, the robots axel len
% and the max speeds for the wheels and the max speed for the end of the
% segment, as well as the current speeds going into the segment and the 
% max acceleration of each wheel and the difference in time for each
% setpoint, calculates the setpoints that the controller should follow
% to stay within the parameters and limits for that segment. These are
% returned as row vectors for left_vels and right_vels where the k'th 
% position is the setpoint for t = T_k with T_0 being the start.

% Use Ul_dot = a * Ur_dot + a_dot * Ur
% a and a_dot can be calculated from path param, Ur_t0 is given,
% find max wheel_dot's to increase speed and stay on path.
% if max speed is achieved, keep max speed but need to slow down in
% time to hit end velocity w/o going over.
                                       
                                       
                                       
end