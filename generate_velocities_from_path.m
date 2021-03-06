function [max_left_vels, max_right_vels, speeds, omega_dt ,times_at_t]...
         = generate_velocities_from_path( omega_dx, delta_x_delta_t,...
                                         axel_len, max_accel_abs, max_wheel_speed)
%GENERATE_VELOCITIES_FROM_PATH Finds max vels at max correction
%   This constitues the Control Invarient Set, the pairs of states
%   and control inputs that are the boundary of the constraints.
  %Left wheel slower, a in [-1, 1]
  a = (1 - omega_dx.* axel_len/2)./... % a is for ul on the left
      (1 + omega_dx.* axel_len/2);    % new_ul must = a*new_ur
  %Right wheel slower, b in [-1, 1]
  b = (1 + omega_dx.* axel_len/2)./... % b is for ur on the left
      (1 - omega_dx.* axel_len/2);    % new_ur must = b*new_ul
  %changes between t's
  
  %take central difference of a,b with edge repeat
  a_diff = a(2:end) - a(1:end-1);
  a_diff = [0,a_diff,0]; %edge of a repeated
  b_diff = b(2:end) - b(1:end-1);
  b_diff = [0,b_diff, 0];
  delta_a_delta_x = zeros(1,length(omega_dx));
  delta_b_delta_x = zeros(1,length(omega_dx));

  extend_delta_x = [delta_x_delta_t(1), delta_x_delta_t, delta_x_delta_t(end)];
  for index = 1:length(omega_dx)
      delta_a_delta_x(index) = .5*(a_diff(index)/extend_delta_x(index) ...
                             + a_diff(index+1)/extend_delta_x(index+1)); 
      delta_b_delta_x(index) = .5*(b_diff(index)/extend_delta_x(index) ...
                             + b_diff(index+1)/extend_delta_x(index+1));
  end
  
  %find max possible velocities if max correction was being applied
  % for each point on the path and its change in curvature.
  Ul_dot = zeros(1,length(omega_dx));
  Ur_dot = zeros(1,length(omega_dx));
  Ul = zeros(1,length(omega_dx));
  Ur = zeros(1,length(omega_dx));
  
  for index = 1:length(omega_dx)
  if a(index) < 1 && a(index) > -1
      if abs(delta_a_delta_x(index)) > .001
          if delta_a_delta_x(index)> 0 % straitening out, right wheel faster
                        %left wheel getting faster
              %max correction
              if a(index) > 0 %turning outside left wheel
                  Ul_dot(index) = max_accel_abs; %speed up
                  Ur_dot(index) = -max_accel_abs;
              else %turning inside left wheel
                  Ul_dot(index) = max_accel_abs; % 
                  Ur_dot(index) = max_accel_abs;
              end
          else %turning more, right wheel faster
               %left wheel getting slower
               %max_correction
              if a(index) > 0 %turning outside left wheel
                  Ul_dot(index) = -max_accel_abs; %slow down
                  Ur_dot(index) = max_accel_abs;
              else %turning inside left wheel
                  Ul_dot(index) = -max_accel_abs; % 
                  Ur_dot(index) = -max_accel_abs;
              end
          end
          %calculate max Ur^2 and use Ul = a*Ur to find vels
          Ur(index) = max(-max_wheel_speed,min(max_wheel_speed,...
                      sqrt((Ul_dot(index) - Ur_dot(index)*a(index))*2 ...
                      / (delta_a_delta_x(index)*(1 + a(index))))));
          Ul(index) = max(-max_wheel_speed,min(max_wheel_speed,a(index)*Ur(index)));
      else
          Ul_dot(index) = a(index)*max_accel_abs; % 
          Ur_dot(index) = max_accel_abs;
          
          Ur(index) = max_wheel_speed;
          Ul(index) = a(index)*max_wheel_speed;
      end
  else %better'd use b
      if abs(delta_b_delta_x(index)) > .001
          if delta_b_delta_x(index)> 0 % straitening out, left wheel faster
                        %right wheel getting faster
              %max correction
              if b(index) > 0 %turning outside right wheel
                  Ul_dot(index) = -max_accel_abs; %speed up
                  Ur_dot(index) = max_accel_abs;
              else %turning inside right wheel 
                  Ul_dot(index) = max_accel_abs; % 
                  Ur_dot(index) = max_accel_abs;
              end
          else %turning more, left wheel faster
               %right wheel getting slower
               %max_correction
              if b(index) > 0 %turning outside right wheel
                  Ul_dot(index) = max_accel_abs; 
                  Ur_dot(index) = -max_accel_abs;%slow down
              else %turning inside right wheel
                  Ul_dot(index) = -max_accel_abs; % 
                  Ur_dot(index) = -max_accel_abs;
              end
          end
          %calculate max Ul^2 and use Ul = a*Ur to find vels
          Ul(index) = max(-max_wheel_speed,min(max_wheel_speed,...
                      sqrt((Ur_dot(index) - Ul_dot(index)*b(index))*2 ...
                      / (delta_b_delta_x(index)*(1 + b(index))))));
          Ur(index) =  max(-max_wheel_speed,min(max_wheel_speed,b(index)*Ul(index)));
      else
          Ul_dot(index) = max_accel_abs; % 
          Ur_dot(index) = b(index)*max_accel_abs;
          
          Ur(index) = b(index)*max_wheel_speed;
          Ul(index) = max_wheel_speed;
      end
  end
  end
  max_left_vels = Ul;
  max_right_vels = Ur;
  speeds = .5 * (Ul + Ur);
  omega_dt = (Ur - Ul)/ axel_len;
  times_at_t = delta_x_delta_t ./ speeds; %should compare with radians/omega too
%    figure()
%    plot(Ul(70:end))
%  hold on
%  plot(Ur(70:end))
%  plot(a(70:end))
%  print("det")
end

