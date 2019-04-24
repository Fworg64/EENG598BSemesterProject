num_CP = 2;
%x0 = zeros(num_CP,3);
global x_start
global x_final
global Rocks;

Rocks = [2, 1.5, 1.5]; %rows of x,y,radius
x_start = [0, 0, 0];
x_final = [6, 2, -1.6];
x0 = [x_final/4;
      3*x_final/4];
  
options = optimoptions('fmincon', 'Algorithm', 'sqp');
tic
[x, fval] = fmincon(@Obj_fun,x0, [],[],[],[],[],[],@nonlin, options);
toc

idealCPs  = x
[total_time, d] = thebigf(x)
plotCPs(idealCPs, d, Rocks);

function obj = Obj_fun(x)
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