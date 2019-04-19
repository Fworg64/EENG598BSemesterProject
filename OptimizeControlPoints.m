num_CP = 1;
x0 = zeros(num_CP,3);
tic
[x, fval] = fmincon(@Obj_fun,x0, [],[],[],[],[],[],@nonlin);
toc

idealCPs  = x
[total_time, d] = thebigf(x)

function obj = Obj_fun(x)
    obj = thebigf(x);
end

function [c, ceq] = nonlin(x)
    x_start = [0, 0, 0];
    x_final = [4, 1, .1];
    ceq = [x_start'+sum(x,1)'-x_final'];
    c =[];
end