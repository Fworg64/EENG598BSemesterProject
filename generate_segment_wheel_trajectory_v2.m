function [left_vels, right_vels] = ...
         generate_segment_wheel_trajectory_v2( ...
           omega_dx_prev, omega_dx_curr, omega_dx_next, ...
           delta_x_delta_t, axel_len,...
           max_pos_accel, max_neg_accel,... %what the wheel can do
           max_abs_wheel_speed, ...
           max_end_left_speed, max_end_right_speed, ...
           starting_left_speed, starting_right_speed, dt)
%  Generate segment wheel trajectory
% ---INPUTS SHOULD BE SCALARS--- except delta_x_delta_t
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
  a_set = (1 - omega_dx.* axel_len/2)./... % a is for ul on the left
      (1 + omega_dx.* axel_len/2);    % new_ul must = a*new_ur
  %Right wheel slower, b in [-1, 1]
  b_set = (1 + omega_dx.* axel_len/2)./... % b is for ur on the left
      (1 - omega_dx.* axel_len/2);    % new_ur must = b*new_ul
  %changes between t's
% 2. Calculate a_dot (or b_dot)
  
  %take central difference of a,b with edge repeat
  a_diff = a_set(2:end) - a_set(1:end-1);
  a_diff = [0,a_diff,0]; %edge of a repeated
  b_diff = b_set(2:end) - b_set(1:end-1);
  b_diff = [0,b_diff, 0];
  delta_a_delta_x = zeros(1,length(omega_dx));
  delta_b_delta_x = zeros(1,length(omega_dx));

% 3. Find change in a per change in x (da/dx)
%    Note that a_dot(time) = delta_a_delta_x * speed(time);
%    Also note that delta_x_delta_t, t is path param, not time.
  extend_delta_x = [delta_x_delta_t];
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
  omega_sign_factor = 2*(omega_dx(2) > 0) - 1;
  
  a = a_set(2);
  b = b_set(2);
  
while ~((x_trav >= delta_x_delta_t(2)) || ...
          (omega_sign_factor*theta_trav >= omega_sign_factor*...
                                omega_dx(index_from_prev)*delta_x_delta_t(2)))
    truidex = truidex + 1;
    if truidex > 1000
        disp("UH OH");
        fprintf("X: %.4f / %.4f\n", x_trav, delta_x_delta_t(2));
        fprintf("theta: %.4f / %.4f\n", omega_sign_factor*theta_trav, ...
              omega_sign_factor*omega_dx(index_from_prev)*delta_x_delta_t(2));
    end
    Ul_dot_upperbound = min(max_pos_accel*dt, (max_end_left_speed  - Ul(truidex-1)));
    Ur_dot_upperbound = min(max_pos_accel*dt, (max_end_right_speed - Ur(truidex-1)));
    Ul_dot_lowerbound = min(Ul_dot_upperbound, max(max_neg_accel*dt, (-max_abs_wheel_speed - Ul(truidex-1))));
    Ur_dot_lowerbound = min(Ur_dot_upperbound, max(max_neg_accel*dt, (-max_abs_wheel_speed - Ur(truidex-1))));
    speed = .5 * (Ur(truidex-1) + Ul(truidex-1));
    a_dot = delta_a_delta_x(2)*speed;
    b_dot = delta_b_delta_x(2)*speed;
    
if (a <= 1.0)
  Ur_dot_plus = Ur_dot_upperbound;
  Ul_dot_plus = a*(Ur_dot_plus + Ur(truidex - 1)) - Ul(truidex - 1);
  if (Ul_dot_plus > Ul_dot_upperbound || Ul_dot_plus < Ul_dot_lowerbound)
      if abs(a) > .01
          if a > 0
              Ul_dot_plus = Ul_dot_upperbound;
          else
              Ul_dot_plus = Ul_dot_lowerbound;
          end
          Ur_dot_plus = (Ul_dot_plus + Ul(truidex - 1))/a - Ul(truidex -1);
      end
  end
  if (Ur_dot_plus > Ur_dot_upperbound || Ur_dot_plus < Ur_dot_lowerbound)
      lambda = (a*Ur(truidex -1) - Ul(truidex - 1))/(-(a^2) - 1);
      Ul_dot_plus = -lambda;
      Ur_dot_plus = a*lambda;
      fprintf("Infeasable answer: %.4f, %.4f", Ul_dot_plus, Ur_dot_plus);
  end 
else %better'd use b
  Ul_dot_plus = Ul_dot_upperbound;
  Ur_dot_plus = b*(Ul_dot_plus + Ul(truidex - 1)) - Ur(truidex - 1);
  if (Ur_dot_plus > Ur_dot_upperbound || Ur_dot_plus < Ur_dot_lowerbound)
      if abs(b) > .01
          if b > 0
              Ur_dot_plus = Ur_dot_upperbound;
          else
              Ur_dot_plus = Ur_dot_lowerbound;
          end
          Ul_dot_plus = (Ur_dot_plus + Ur(truidex - 1))/b - Ur(truidex -1);
      end
  end
  if (Ul_dot_plus > Ul_dot_upperbound || Ul_dot_plus < Ul_dot_lowerbound)
      lambda = (b*Ul(truidex -1) - Ur(truidex - 1))/(-(b^2) - 1);
      Ur_dot_plus = -lambda;
      Ul_dot_plus = b*lambda;
      fprintf("Infeasable answer: %.4f, %.4f", Ul_dot_plus, Ur_dot_plus);
  end 
end

  Ul(truidex) = Ul(truidex-1) + Ul_dot_plus;
  Ur(truidex) = Ur(truidex-1) + Ur_dot_plus;
  a = a + a_dot*dt;
  b = b + b_dot*dt;
  
  %check lower bounds too
  
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