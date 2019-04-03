function [left_vels, right_vels] = ...
         generate_segment_wheel_trajectory( ...
           omega_dx_prev, omega_dx_curr, omega_dx_next, ...
           delta_x_delta_t, axel_len, max_accel_abs, ...
           max_abs_left_wheel_speed, max_abs_right_wheel_speed, ...
           max_end_left_speed, max_end_right_speed, ...
           starting_left_speed, starting_right_speed, dt]
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
  prealloc_size = 10;
  Ul_dot = zeros(1,prealloc_size);
  Ur_dot = zeros(1,prealloc_size);
  Ul = zeros(1,prealloc_size);
  Ur = zeros(1,prealloc_size);
  
  Ul(1) = starting_left_speed;
  Ur(1) = starting_right_speed;
  x_trav = 0;
  theta_trav = 0;
  truidex = 0;
  while ~(x_trav >= delta_x_delta_t && theta_trav >= omega_dx*delta_x_delta_t)
    truidex = truidex + 1;
    index_from_prev = 2; %copy pasta
    %should happen at the same time anyway
    %want to solve problem for well conditioned side
    % 5. calculate max speed accelerations for each wheel
    speed = .5 * (Ur(truidex) + Ul(truidex));
  if a(index_from_prev) < 1 && a(index_from_prev) > -1 % Ul = a Ur, so Ur is bigger                    %left wheel getting faster
          %calculate
          %iterate until appropate Ul_dot and Ur_dot are found for this timestep
          %accel_sum = 0;
          Ur_dot_plus = 0;
          Ul_dot_plus = a(index_from_prev)*Ur_dot_plus + ...
                        delta_a_delta_x(index_from_prev)*speed*Ur(truidex);
          delta_ul_dot = 10;
          delta_ur_dot = 10;
          chi = .02;
      while (abs(delta_ul_dot) > .01 || abs(delta_ur_dot) > .01)
          %note that Ul_dot_plus update eq is the same for each case
          % and the relation bewtwen sign of a, a_dot and condition and
          % increment direction.
        if Ul_dot_plus < max_accel_abs
          if a > 0 %turning outside left wheel
            Ur_dot_plus = Ur_dot_plus + chi*max_accel_abs;%-max_accel_abs;
            Ul_dot_plus = a(index_from_prev)*Ur_dot_plus + ...
                        delta_a_delta_x(index_from_prev)*speed*Ur(truidex);
            if Ur_dot_plus >= max_accel_abs || Ul_dot_plus >= max_accel_abs
                delta_ul_dot = 0;
                delta_ur_dot = 0;
            end
          else %turning inside left wheel
            Ur_dot_plus = Ur_dot_plus - chi*max_accel_abs;%-max_accel_abs;
            Ul_dot_plus = a(index_from_prev)*Ur_dot_plus + ...
                        delta_a_delta_x(index_from_prev)*speed*Ur(truidex);
            if Ur_dot_plus <= -max_accel_abs || Ul_dot_plus >= max_accel_abs
                delta_ul_dot = 0;
                delta_ur_dot = 0;
            end
          end
        else %Ul_dot_plus > max_accel abs ( need to slow down)
          if a > 0 %turning outside left wheel
            Ur_dot_plus = Ur_dot_plus - chi*max_accel_abs;%-max_accel_abs;
            Ul_dot_plus = a(index_from_prev)*Ur_dot_plus + ...
                        delta_a_delta_x(index_from_prev)*speed*Ur(truidex);
            if Ur_dot_plus <= -max_accel_abs || Ul_dot_plus <= max_accel_abs
                delta_ul_dot = 0;
                delta_ur_dot = 0;
            end
          else %turning inside left wheel
            Ur_dot_plus = Ur_dot_plus + chi*max_accel_abs;%-max_accel_abs;
            Ul_dot_plus = a(index_from_prev)*Ur_dot_plus + ...
                        delta_a_delta_x(index_from_prev)*speed*Ur(truidex);
            if Ur_dot_plus >= max_accel_abs || Ul_dot_plus <= max_accel_abs
                delta_ul_dot = 0;
                delta_ur_dot = 0;
            end
          end
        end %end if slower wheel below abs or above
      end %end while
           %left wheel getting slower
  else %better'd use b
      if abs(delta_b_delta_x(index_from_prev)) > .001
          if delta_b_delta_x(index_from_prev)> 0 % straitening out, left wheel faster
                        %right wheel getting faster
              %max correction
              if b(index_from_prev) > 0 %turning outside right wheel
                  Ul_dot(index_from_prev) = -max_accel_abs; %speed up
                  Ur_dot(index_from_prev) = max_accel_abs;
              else %turning inside right wheel 
                  Ul_dot(index_from_prev) = max_accel_abs; % 
                  Ur_dot(index_from_prev) = max_accel_abs;
              end
          else %turning more, left wheel faster
               %right wheel getting slower
               %max_correction
              if b(index_from_prev) > 0 %turning outside right wheel
                  Ul_dot(index_from_prev) = max_accel_abs; 
                  Ur_dot(index_from_prev) = -max_accel_abs;%slow down
              else %turning inside right wheel
                  Ul_dot(index_from_prev) = -max_accel_abs; % 
                  Ur_dot(index_from_prev) = -max_accel_abs;
              end
          end
          %calculate max Ul^2 and use Ul = a*Ur to find vels
          Ul(index_from_prev) = max(-max_wheel_speed,min(max_wheel_speed,...
                      sqrt((Ur_dot(index_from_prev) - Ul_dot(index_from_prev)*b(index_from_prev))*2 ...
                      / (delta_b_delta_x(index_from_prev)*(1 + b(index_from_prev))))));
          Ur(index_from_prev) =  max(-max_wheel_speed,min(max_wheel_speed,b(index_from_prev)*Ul(index_from_prev)));
      else
          Ul_dot(index_from_prev) = max_accel_abs; % 
          Ur_dot(index_from_prev) = b(index_from_prev)*max_accel_abs;
          
          Ur(index_from_prev) = b(index_from_prev)*max_wheel_speed;
          Ul(index_from_prev) = max_wheel_speed;
      end
    end
  
    %now that we know what accels should be,
    % 6. calculate velocities for this/next time step via integration
    % 6a.Remember to keep max speeds and max end speeds in mind! 
    % (could do as 5a also). [Maybe make upper speed limit linear interp
    % of prev end upper limit and this one's end upper limit?].
    
    % 7. Check end conditions ( done in while loop expression).
    % 7a. Maybe someday we will know how long it will take based on 
    %     problem parameters (resonable).
      
  end
  
                                       
end