function [speed,omega, new_ul, new_ur, time] = sscv2020(max_wheel_speed, axel_len,...
    d_theta_dx, prev_ul, prev_ur, max_accel, delta_x)
%SSCV2020 Ramps up while maintaining turn
%   Given the max_wheel_speed and the axel_len of a differential drive
%   robot, as well as the requried d_theta per meter, the previous 
%   wheel velocities and the max wheel acceleration and the amount
%   of distance to complete the transition in, returns
%   the new achieved speed and d_theta_d_time (omega) as well as the 
%   new wheel velocities that should be used.
%   omega / speed should = d_theta_dx (thats the point)
%   if d_theta_dx > 0 %increase right wheel speed
%       new_ur = min(prev_ur + max_wheel_change, max_wheel_speed);
%       new_ul = new_ur * (1 - d_theta_dx * axel_len/2)/...
%                         (1 + d_theta_dx * axel_len/2);
%   else %increase left wheel speed
%       new_ul = min(prev_ul + max_wheel_change, max_wheel_speed);
%       new_ur = new_ul * (1 + d_theta_dx * axel_len/2)/...
%                         (1 - d_theta_dx * axel_len/2);
%   end
%   IF THERE IS NO SOLN with the above constraints, then the solution
%   has transitioned to a stop and turn situation. This is done by setting
%   the wheel velocities(s) to zero and initiating a zero point turn. The
%   turn is accelerated until sufficient angle change has been made
%   (ignoring the assumed small distance of this segment).
%infinity means zero point turn
%todo, handle singularity as d_theta_dx = -2/ axel_len

  a = (1 - d_theta_dx * axel_len/2)/... % a is for ul on the left
      (1 + d_theta_dx * axel_len/2);    % new_ul must = new_ur * a
  b = 1/a;
  potential_delta_uls = zeros(1,4);
  potential_delta_urs = zeros(1,4);
  
  %ASSERT: there is no better way to write this
  if prev_ur > max_wheel_speed - max_wheel_change
      max_pos_right_wheel_change = max_wheel_speed - prev_ur;
  else 
      max_pos_right_wheel_change = max_wheel_change;
  end
  if prev_ur < -max_wheel_speed + max_wheel_change
      max_neg_right_wheel_change = -max_wheel_speed - prev_ur;
  else
      max_neg_right_wheel_change = -max_wheel_change;
  end
  if prev_ul > max_wheel_speed - max_wheel_change
      max_pos_left_wheel_change = max_wheel_speed - prev_ur;
  else
      max_pos_left_wheel_change = max_wheel_change;
  end
  if prev_ul < -max_wheel_speed + max_wheel_change
      max_neg_left_wheel_change = -max_wheel_speed - prev_ul;
  else
      max_neg_left_wheel_change = -max_wheel_change;
  end
  
  prev_a = prev_ul/prev_ur;
  if a > prev_ur + max_pos_right_wheel_change 
  
  %%%%get max (new_ul +  new_ur) st: abs(new_ul - prev_ul) < .1 &&... 
  %%%%                               abs(new_ur - prev_ur) < .1 &&...
  %%%%                               abs(new_ul)< 1 && abs(new_ur)< 1 
  % %%              new_ul = new_ur * (1 - d_theta_dx * axel_len/2)/...
  % %%                                (1 + d_theta_dx * axel_len/2);
  % %% also say min
  %subbing in constraints:
  %%%
  potential_delta_uls(1) = a*max_pos_right_wheel_change + a*prev_ur - prev_ul;
  potential_delta_urs(1) =   max_pos_right_wheel_change;
  
  potential_delta_uls(2) = a*max_neg_right_wheel_change + a*prev_ur - prev_ul;
  potential_delta_urs(2) =   max_neg_right_wheel_change;

  potential_delta_urs(3) = (1/a) * max_pos_left_wheel_change + (1/a) * prev_ul - prev_ur;
  potential_delta_uls(3) =         max_pos_left_wheel_change;
  
  potential_delta_urs(4) = (1/a) * max_neg_left_wheel_change + (1/a) * prev_ul - prev_ur;
  potential_delta_uls(4) = max_neg_left_wheel_change ;
  
  %each set must have both magnitude less than max_wheel_change
  valids = [1, 1, 1, 1];
  for index = 1:2
      if abs(potential_delta_uls(index)) > max_wheel_change
          valids(index) = 0;
      end
      if abs(potential_delta_urs(index +2)) > max_wheel_change
          valids(index + 2) = 0;
      end
  end
  disp(valids);
  %pick largest sum
  if sum(valids) > 0
      %pick best one, (highest speed)
      
      for index = 1:4
          
      end
  else
      %pick smallest l-inf norm soln from constraint
  end
  new_ul = prev_ul + potential_delta_uls;
  new_ur = prev_ur + potential_delta_urs;
  speed = .5*(new_ul + new_ur);
  omega = (new_ur - new_ul)/axel_len;
end

