d_theta_dx = -25:.01:25;
a_plot = (1 - d_theta_dx*axel_len/2) ./ (1+ d_theta_dx * axel_len/2);
figure()
plot(d_theta_dx, min(3,max(-10,a_plot)))
hold on
plot(d_theta_dx, min(3,max(-10,1./a_plot)))

plot(d_theta_dx, -.7 + 1.7*exp(-abs(d_theta_dx*(axel_len/2))));