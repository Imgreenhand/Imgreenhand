clear
clc

% run("system_parameter.mlx")
% choose a model to simulate
% run("model\MIT_model.mlx")
% run("model\three_symmetry_planes_model.mlx")
run("model\port_starboard_symmetry_model.mlx")


% Fourth_Order Runge-Kutta Numerical Integralthis
t0 = 0;
tfinal = 10;
h = 0.1;
tarray = t0:h:tfinal;
step = length(t0 : h : tfinal-h);
b = W;


% based on simplified full-order state space model
xk = zeros(12,step+1);
nu_dot = zeros(12,step+1);
tau = zeros(6,step+1);

% xk(:,1) = [0.0, 0.0, 0.0, 0*pi/180, 0*pi/180, 0*pi/180, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0];
xk(:,1) = [1.0, 0.3, 1.0, 50*pi/180, 50*pi/180, 30*pi/180, 0.0, 0.0, 0.0, 20*pi/180, 20*pi/180, 20*pi/180]; % initial condition2
% uk = [0 0 0 0 0 0].';    % input variables

F = @(Ak, Gk, xk, uk) Ak*xk + B*uk - [inv(M);zeros(6)]*(Gk-D);

for k = 1:step
    t = 0 + h*k

    Ak = subs(A, [u, v, w, p, q, r, phi, theta, psi], [xk(1,k), xk(2,k), xk(3,k), xk(4,k), xk(5,k), xk(6,k), xk(10,k), xk(11,k), xk(12,k)]);
    Gk = subs(geta, [phi, theta, Buoyancy], [xk(10,k), xk(11,k), b]);

     k1 = h*F(Ak, Gk, xk(:, k), uk);                      % O(h^4)
     k2 = h*F(Ak, Gk, xk(:, k)+k1/2, uk);
     k3 = h*F(Ak, Gk, xk(:, k)+k2/2, uk);
     k4 = h*F(Ak, Gk, xk(:, k)+k3/2, uk);
     xk(:, k+1) = xk(:, k) + (k1 + 2*k2 + 2*k3 + k4)/6;

%      k1 = F(Ak, Gk, xk(:, k), uk);
%      k2 = F(Ak, Gk, xk(:, k)+h*k1/2, uk);
%      k3 = F(Ak, Gk, xk(:, k)+h*k2/2, uk);
%      k4 = F(Ak, Gk, xk(:, k)+h*k3, uk);
%      xk(:, k+1) = xk(:, k) + h*(k1 + 2*k2 + 2*k3 + k4)/6;

     nu_dot(:, k+1) = k1;
     tau(:, k+1) = uk;
    
    % psi(0, 360)
%     if xk(12, k+1) > 2*pi
%         xk(12,k+1) = xk(12,k+1)-2*pi;
%     end

end

xplot_ylabel = ["u (m/s)" "v (m/s)" "w (m/s)" "p (deg/s)" "q (deg/s)" "r (deg/s)" "x (m)" "y (m)" "z (m)" "phi (deg)" "theta (deg)" "psi (deg)"];
xplot_title = ["Surge" 'Sway' 'Heave' 'Roll Rate' 'Pitch Rate' 'Yaw Rate' 'Axial' 'Lat' 'Depth' 'Roll' 'Pitch' 'Yaw'];

for i = 1:12
    subplot(2,6,i)
    if (i >= 4 && i<=6) || (i >=10 && i<= 12)
        plot(tarray, xk(i,:)*180/pi, '-')
    else
        plot(tarray, xk(i,:), '-')
    end
    ylabel(xplot_ylabel(i))
    title(xplot_title(i))
    xlim([-1 t])
    grid on
end


figure
x = xk(7,:);
y = xk(8,:);
z = xk(9,:);

plot3(x, y, z, 'b', 'LineWidth', 2);
hold on
plot3(x(1), y(1), z(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r')
text(x(1), y(1), z(1)+0.05, 'Start', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold')
plot3(x(end), y(end), z(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r')
text(x(end), y(end), z(end)+0.05, 'End', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold')
hold off

xlabel('X');
ylabel('Y');
zlabel('Z');
title('AUV 3-D trajectory');
grid on;

