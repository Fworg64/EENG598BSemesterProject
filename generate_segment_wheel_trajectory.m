function [left_vels, right_vels] = ...
         generate_segment_wheel_trajectory( ...
           omega_dx_prev, omega_dx_curr, omega_dx_next, ...
           delta_x_delta_t, axel_len, max_accel_abs, ...
           max_abs_wheel_speed, ...
           max_end_left_speed, max_end_right_speed, ...
           starting_left_speed, starting_right_speed, dt)
%  Generate segment wheel trajectory
% ---INPUTS SHOULD BE SCALARS---
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

omega_dx = [omega_dx_prev, omega_dx_curr, omega_dx_next];
% 1. Calculate a (or b)
  %Left wheel slower, a in [-1, 1]
  a = (1 - omega_dx.* axel_len/2)./... % a is for ul on the left
      (1 + omega_dx.* axel_len/2);    % new_ul must = a*new_ur
  %Right wheel slower, b in [-1, 1]
  b = (1 + omega_dx.* axel_len/2)./... % b is for ur on the left
      (1 - omega_dx.* axel_len/2);    % new_ur must = b*new_ul
  %changes between t's
% 2. Calculate a_dot (or b_dot)
  
  %take central difference of a,b with edge repeat
  a_diff = a(2:end) - a(1:end-1);
  a_diff = [0,a_diff,0]; %edge of a repeated
  b_diff = b(2:end) - b(1:end-1);
  b_diff = [0,b_diff, 0];
  delta_a_delta_x = zeros(1,length(omega_dx));
  delta_b_delta_x = zeros(1,length(omega_dx));

% 3. Find change in a per change in x (da/dx)
%    Note that a_dot(time) = delta_a_delta_x * speed(time);
%    Also note that delta_x_delta_t, t is path param, not time.
  extend_delta_x = [delta_x_delta_t(1), delta_x_delta_t, delta_x_delta_t(end)];
  for index_from_prev = 2%copy pasta%1:length(omega_dx)
      delta_a_delta_x(index_from_prev) = .5*(a_diff(index_from_prev)/extend_delta_x(index_from_prev) ...
                             + a_diff(index_from_prev+1)/extend_delta_x(index_from_prev+1)); 
      delta_b_delta_x(index_from_prev) = .5*(b_diff(index_from_prev)/extend_delta_x(index_from_prev) ...
                             + b_diff(index_from_prev+1)/extend_delta_x(index_from_prev+1));
  end
% 4. Find vector of velocities for 'ideal' trajectory
  prealloc_size = 100;
  %Ul_dot = zeros(1,prealloc_size);
  %Ur_dot = zeros(1,prealloc_size);
  Ul = zeros(1,prealloc_size);
  Ur = zeros(1,prealloc_size);
  
  Ul(1) = starting_left_speed;
  Ur(1) = starting_right_speed;
  x_trav = 0;
  theta_trav = 0;
  truidex = 1;
  omega_sign_factor = sign(omega_dx(index_from_prev));
  while ~((x_trav >= delta_x_delta_t) && ...
          (omega_sign_factor*theta_trav >= omega_sign_factor*...
                                omega_dx(index_from_prev)*delta_x_delta_t))
    truidex = truidex + 1;
    if truidex > 1000
        disp("UH OH");
    end
    index_from_prev = 2; %copy pasta
    %should happen at the same time anyway
    %want to solve problem for well conditioned side
    % 5. calculate max speed accelerations for each wheel
    % 5a. Enforce max terminal and abs speed
    %     can get max speed for any point (from terminal speed constraint)
    %     as function of distance to end and max terminal speed
    %     enforce the more restrictive of this and max abs speed.
    speed = .5 * (Ur(truidex-1) + Ul(truidex-1));
    % think we need actual distance travelled by wheel, not just 
    % delta_x_delta_t % THIS IS TRUE
    % need arc length for each wheel path
    curr_max_right_speed = min(max_abs_wheel_speed, ...
                           real(sqrt(max_end_right_speed^2 +...
                                2*(-max_accel_abs)*(x_trav - delta_x_delta_t))));
    curr_max_left_speed = min(max_abs_wheel_speed, ...
                           real(sqrt(max_end_left_speed^2 +...
                                2*(-max_accel_abs)*(x_trav - delta_x_delta_t))));
  if a(index_from_prev) < 1 && a(index_from_prev) > -1 % Ul = a Ur, so Ur is bigger                    %left wheel getting faster
      %calculate
      %iterate until appropate Ul_dot and Ur_dot are found for this timestep
      %accel_sum = 0;
      Ur_dot_plus = 0;
      Ul_dot_plus = a(index_from_prev)*Ur_dot_plus + ...
                    delta_a_delta_x(index_from_prev)*speed*Ur(truidex-1);
      delta_ul_dot = 10;
      delta_ur_dot = 10;
      chi = .02;
      
      grad_sign = sign(a(index_from_prev)) + 1.0*(a(index_from_prev) == 0);
      grad_sign = 1;
      above_or_below_sign = sign(max_accel_abs - Ul_dot_plus) +...
                            1.0*(max_accel_abs - Ul_dot_plus == 0);
      while (abs(delta_ul_dot) > .01 || abs(delta_ur_dot) > .01)
        Ur_dot_plus = Ur_dot_plus + ...
                      above_or_below_sign*grad_sign*chi*max_accel_abs;
        Ul_dot_plus = a(index_from_prev)*Ur_dot_plus + ...
                      delta_a_delta_x(index_from_prev)*speed*Ur(truidex-1); 
        if ((Ur_dot_plus >= max_accel_abs) || (Ur_dot_plus <= -max_accel_abs) ||...
             (sign(max_accel_abs - Ul_dot_plus) ~= above_or_below_sign))
            delta_ul_dot = 0;
            delta_ur_dot = 0;
        end
      end
      Ur(truidex) = Ur(truidex-1) + Ur_dot_plus * dt;
      Ul(truidex) = Ul(truidex-1) + Ul_dot_plus * dt;
      if Ur(truidex) >= curr_max_right_speed
        Ur(truidex) = curr_max_right_speed;
        Ul(truidex) = a(index_from_prev) * Ur(truidex);
      end
  else %better'd use b
      Ul_dot_plus = 0;
      Ur_dot_plus = b(index_from_prev)*Ul_dot_plus + ...
                    delta_b_delta_x(index_from_prev)*speed*Ul(truidex-1);
      delta_ul_dot = 10;
      delta_ur_dot = 10;
      chi = .02;
       
      grad_sign = sign(b(index_from_prev)) + 1.0*(b(index_from_prev) == 0);
      grad_sign = 1;
      above_or_below_sign = sign(max_accel_abs - Ur_dot_plus) +...
                            1.0*(max_accel_abs - Ur_dot_plus == 0);
      while (abs(delta_ul_dot) > .01 || abs(delta_ur_dot) > .01)
          %note that Ul_dot_plus update eq is the same for each case
          % and the relation bewtwen sign of a, a_dot and condition and
          % increment direction.
        Ul_dot_plus = Ul_dot_plus + ...
                      above_or_below_sign*grad_sign*chi*max_accel_abs;
        Ur_dot_plus = b(index_from_prev)*Ul_dot_plus + ...
                      delta_b_delta_x(index_from_prev)*speed*Ul(truidex-1); 
        if ((Ul_dot_plus >= max_accel_abs) || (Ul_dot_plus <= -max_accel_abs) ||...
           (sign(max_accel_abs - Ur_dot_plus) ~= above_or_below_sign))
            delta_ul_dot = 0;
            delta_ur_dot = 0;
        end
      end
      Ur(truidex) = Ur(truidex-1) + Ur_dot_plus * dt;
      Ul(truidex) = Ul(truidex-1) + Ul_dot_plus * dt;
    if Ul(truidex) >= curr_max_left_speed
      Ul(truidex) = curr_max_left_speed;
      Ur(truidex) = b(index_from_prev) * Ul(truidex);
    end
  end
  % 7. Check end conditions ( done in while loop expression).
  % 7a. Maybe someday we will know how long it will take based on 
  %     problem parameters (resonable).
  x_trav = x_trav + .5*(Ur(truidex) + Ul(truidex))*dt;
  theta_trav = theta_trav + dt*(Ur(truidex) - Ul(truidex))/axel_len;
  if truidex >= prealloc_size
     Ul = [Ul, zeros(1, prealloc_size)];
     Ur = [Ur, zeros(1, prealloc_size)];
  end
  end
  
  left_vels = Ul(1:truidex);
  right_vels = Ur(1:truidex);
                                       
end