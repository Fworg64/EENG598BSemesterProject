%global num_CP
%x0 = zeros(num_CP,3);
global x_start
global x_final
global Rocks;

num_CP = 2;

Rocks = [3, 1, 1.0]; %rows of x,y,radius
x_start = [0, 0, 0];
x_final = [6, 2, 0];
x0 = [x_final/4;
      3*x_final/2];
%x0 = x_final  
num_vars = length(x0(:));
Aineq = [eye(num_vars);
         -eye(num_vars)];
Bineq = [repmat([10; 10; 3.5], num_CP, 1);repmat([-1; 10; 3.5], num_CP, 1)];

%Aeq = repmat(eye(3),1,num_CP);
%Beq = x_final';
options = optimoptions('fmincon', 'Algorithm', 'interior-point');
tic
[x, fval] = fmincon(@Obj_fun,x0, Aineq,Bineq,[],[],[],[],@nonlin, options);
toc

idealCPs  = x
[total_time, d] = thebigf(x)
plotCPs(idealCPs, d, Rocks);

function obj = Obj_fun(x)
   %global num_CP;
   % inx = reshape(x, num_CP, 3);
    obj = thebigf(x);
end

function [c, ceq] = nonlin(x)
    global Rocks
    global x_start
    global x_final

    ceq = [x_start'+sum(x,1)'-x_final'];

    [~, d] = thebigf(x);
    c = [pathCost(x,d, Rocks)];%; abs(x_start'+sum(x,1)'-x_final')]; 
    %c =[ -((x(1) - Rocks(1,1)).^2 + (x(2) - Rocks(1,2)).^2) + Rocks(1,3).^2 ];
end