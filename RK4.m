clear
clc

% run("D:\桌面\UVMS\Matlab\AUV_Shark H_infity Roubust Control\code\system_parameter.mlx")
% choose a model to simulate
% run("D:\桌面\UVMS\Matlab\AUV_Shark H_infity Roubust Control\code\MIT_model.mlx")
% run("model\three_symmetry_planes_model.mlx")
run("D:\桌面\UVMS\Matlab\AUV_Shark H_infity Roubust Control stable\code\model\port_starboard_symmetry_model.mlx")


% run("dataset\prediction_dataset.m")


% Fourth_Order Runge-Kutta Numerical Integralthis
t0 = 0;
tfinal = 190;
h = 0.1;
tarray = t0:h:tfinal;
step = length(t0 : h : tfinal-h);
b = W;


% based on simplified full-order state space model
xk = zeros(12,step+1);
nu_dot = zeros(12,step+1);
tau = zeros(6,step+1);
% xk(:,1) = [1, 1, 1, 50*pi/180, 50*pi/180, 50*pi/180, 0, 0, 0, 20*pi/180, 20*pi/180, 20*pi/180];  % initial condition1
% xk(:,1) = [0.0, 0.0, 0.0, 0*pi/180, 0*pi/180, 0*pi/180, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0];
xk(:,1) = [1.0, 0.3, 1.0, 50*pi/180, 50*pi/180, 30*pi/180, 0.0, 0.0, 0.0, 20*pi/180, 20*pi/180, 20*pi/180]; % initial condition2
% uk = [0 0 0 0 0 0].';    % input variables
F = @(Ak, Gk, xk, uk) Ak*xk + B*uk - [inv(M);zeros(6)]*(Gk-D);
% 6-DoF sin wave thrust
% SinWave = [
%     % struct('t_start', 0, 't_end', 10, 'uk', [0 0 0 0 0 0].');% stop initial
%     struct('t_start', 10, 't_end', 20, 'uk', @(t)[-5*sin(2*pi/20*t) 0 0 -0.5*sin(2*pi/20*t) 0 0].');% u p
%     struct('t_start', 20, 't_end', 30, 'uk', @(t)[-10*sin(2*pi/20*t) 0 0 -sin(2*pi/20*t) 0 0].');
%     struct('t_start', 30, 't_end', 50, 'uk', @(t)[0 -2*sin(2*pi/20*t) 0 0 0 0].');% v  
%     struct('t_start', 50, 't_end', 70, 'uk', @(t)[0 0 -2*sin(2*pi/20*t) 0 0 0].');% w
%     struct('t_start', 70, 't_end', 90, 'uk', @(t)[0 0 -3*sin(2*pi/20*t) 0 0 0].');
%     struct('t_start', 90, 't_end', 110, 'uk', @(t)[0 0 0 0 0 -0.5*sin(2*pi/20*t)].');% r
%     struct('t_start', 110, 't_end', 130, 'uk', @(t)[0 0 0 0 0 -sin(2*pi/20*t)].');
%     struct('t_start', 130, 't_end', 140, 'uk', @(t)[0 0 0 0 -sin(2*pi/20*t) 0].');% q
%     struct('t_start', 140, 't_end', 160, 'uk', @(t)[5*sin(2*pi/20*t) 0 2*sin(2*pi/20*t) 0 0 0].');% u w
%     struct('t_start', 160, 't_end', 180, 'uk', @(t)[10*sin(2*pi/20*t) 0 3*sin(2*pi/20*t) 0 0 0].');
%     struct('t_start', 180, 't_end', 200, 'uk', @(t)[6*sin(2*pi/20*t) 0 0 0 sin(2*pi/20*t) 0].');% u q
%     struct('t_start', 200, 't_end', 220, 'uk', @(t)[8*sin(2*pi/20*t) 0 0 0 0.5*sin(2*pi/20*t) 0].');
%     struct('t_start', 220, 't_end', 240, 'uk', @(t)[4*sin(2*pi/20*t) sin(2*pi/20*t) 0 0 0 0].');% u v
%     struct('t_start', 240, 't_end', 260, 'uk', @(t)[2*sin(2*pi/20*t) 2*sin(2*pi/20*t) 0 0 0 0].');
%     struct('t_start', 260, 't_end', 280, 'uk', @(t)[3*sin(2*pi/20*t) 0 0 0 0 0.5*sin(2*pi/20*t)].');% u r
%     struct('t_start', 280, 't_end', 300, 'uk', @(t)[2*sin(2*pi/20*t) 0 0 0 0 sin(2*pi/20*t)].');
%     struct('t_start', 300, 't_end', 320, 'uk', @(t)[2*sin(2*pi/20*t)  0.5*sin(2*pi/20*t) 0 0 0 sin(2*pi/20*t)].');% u v r
%     struct('t_start', 320, 't_end', 340, 'uk', @(t)[3*sin(2*pi/20*t) sin(2*pi/20*t) 0 0 0 0.5*sin(2*pi/20*t)].');
%     struct('t_start', 340, 't_end', 360, 'uk', @(t)[3*sin(2*pi/20*t) sin(2*pi/20*t) sin(2*pi/20*t) 0 0 0].');% u v w
%     struct('t_start', 360, 't_end', 380, 'uk', @(t)[sin(2*pi/20*t) 0.5*sin(2*pi/20*t) sin(2*pi/20*t) 0.5*sin(2*pi/20*t) 0 0].');% u v w p
%     struct('t_start', 380, 't_end', 400, 'uk', @(t)[sin(2*pi/20*t) 0 0 0 0 0].');% u
%     struct('t_start', 400, 't_end', 420, 'uk', @(t)[0 sin(2*pi/20*t) 0 0 0 0].');% v
%     struct('t_start', 420, 't_end', 440, 'uk', @(t)[0 0 0 sin(2*pi/20*t) 0 0].');% p
%     struct('t_start', 440, 't_end', 490, 'uk', @(t)[sin(2*pi/20*t) 0 2*sin(2*pi/20*t) 0 0 sin(2*pi/20*t)].');% u w r
%     % struct('t_start', 490, 't_end', 500, 'uk', [0 0 0 0 0 0].'), % stop
%     ];
% 
% % static acceleration-deceleration
% StaticAccDcc = [
%     struct('t_start', 10, 't_end', 20, 'uk', @(t)[1 0 0 0 0 0].'); % u
%     struct('t_start', 20, 't_end', 30, 'uk', @(t)[3 0 0 0 0 0].');
%     struct('t_start', 30, 't_end', 50, 'uk', @(t)[2 0 0 0 0 0].');
%     struct('t_start', 50, 't_end', 70, 'uk', @(t)[5 0 0 0 0 0].');
%     struct('t_start', 70, 't_end', 90, 'uk', @(t)[1 0 0 0 0 0].');
%     struct('t_start', 90, 't_end', 100, 'uk', @(t)[0 0 0 0 0 0].'); % stop
%     struct('t_start', 100, 't_end', 110, 'uk', @(t)[0 0.5 0 0 0 0].'); % v
%     struct('t_start', 110, 't_end', 130, 'uk', @(t)[0 1 0 0 0 0].');
%     struct('t_start', 130, 't_end', 140, 'uk', @(t)[0 0.5 0 0 0 0].');
%     struct('t_start', 140, 't_end', 160, 'uk', @(t)[0 3 0 0 0 0].');
%     struct('t_start', 160, 't_end', 180, 'uk', @(t)[0 5 0 0 0 0].');
%     struct('t_start', 180, 't_end', 200, 'uk', @(t)[0 2 0 0 sin(2*pi/20*t) 0].');
%     struct('t_start', 200, 't_end', 210, 'uk', @(t)[0 0 0 0 0 0].'); % stop
%     struct('t_start', 210, 't_end', 220, 'uk', @(t)[0 0 1 0 0 0].'); % w
%     struct('t_start', 220, 't_end', 240, 'uk', @(t)[0 0 3 0 0 0].');
%     struct('t_start', 240, 't_end', 260, 'uk', @(t)[0 0 6 0 0 0].');
%     struct('t_start', 260, 't_end', 280, 'uk', @(t)[0 0 4 0 0 0].');
%     struct('t_start', 280, 't_end', 300, 'uk', @(t)[0 0 1 0 0 0].');
%     struct('t_start', 300, 't_end', 310, 'uk', @(t)[0 0 0 0 0 0].'); % stop
%     struct('t_start', 310, 't_end', 320, 'uk', @(t)[0 0 0 0.2 0 0].'); % p
%     struct('t_start', 320, 't_end', 340, 'uk', @(t)[0 0 0 0.5 0 0].');
%     struct('t_start', 340, 't_end', 360, 'uk', @(t)[0 0 0 -0.5 0 0].');
%     struct('t_start', 360, 't_end', 380, 'uk', @(t)[0 0 0 -0.2 0 0].');
%     struct('t_start', 380, 't_end', 400, 'uk', @(t)[0 0 0 0.2 0 0].');
%     struct('t_start', 400, 't_end', 410, 'uk', @(t)[0 0 0 0 0 0].'); % stop
%     struct('t_start', 410, 't_end', 420, 'uk', @(t)[0 0 0 0 0.5 0].'); % q
%     struct('t_start', 420, 't_end', 440, 'uk', @(t)[0 0 0 0 1 0].');
%     struct('t_start', 440, 't_end', 460, 'uk', @(t)[0 0 0 0 0.3 0].');
%     struct('t_start', 460, 't_end', 470, 'uk', @(t)[0 0 0 0 0 0].'); % stop
%     struct('t_start', 470, 't_end', 480, 'uk', @(t)[0 0 0 0 0 0.5].'); % r
%     struct('t_start', 480, 't_end', 490, 'uk', @(t)[0 0 0 0 0 1].');
% ];
% 
% Turning = [
%     struct('t_start', 10, 't_end', 30, 'uk', [1 0 0 0 0 0].'); % u
%     struct('t_start', 30, 't_end', 100, 'uk', [1 0 0 0 0 1].'); % u r
%     struct('t_start', 100, 't_end', 120, 'uk', [1 0 0 0 0 0].');
%     struct('t_start', 120, 't_end', 180, 'uk', [1 0 0 0 0 2].');
%     struct('t_start', 180, 't_end', 200, 'uk', [2 0 0 0 0 0].');
%     struct('t_start', 200, 't_end', 280, 'uk', [2 0 0 0 0 0.5].');
%     struct('t_start', 280, 't_end', 300, 'uk', [3 0 0 0 0 0].');
%     struct('t_start', 300, 't_end', 380, 'uk', [3 1 0 0 0 0].'); % u v
%     struct('t_start', 380, 't_end', 400, 'uk', [4 0 0 0 0 0].');
%     struct('t_start', 400, 't_end', 480, 'uk', [4 1 0 0 0 1].'); % u v r
%     struct('t_start', 480, 't_end', 490, 'uk', [0 2 0 0 0 1].') % v r
% ];  
% 
% Spiral = [
% 
% 
% 
% ];


for k = 1:step
    t = 0 + h*k

    Ak = subs(A, [u, v, w, p, q, r, phi, theta, psi], [xk(1,k), xk(2,k), xk(3,k), xk(4,k), xk(5,k), xk(6,k), xk(10,k), xk(11,k), xk(12,k)]);
    Gk = subs(geta, [phi, theta, Buoyancy], [xk(10,k), xk(11,k), b]);

    % Kalman Filter
    if t >= 15.0 && t <35.0
        uk = [-5*sin(2*pi/20*t) 0 0 -0.5*sin(2*pi/20*t) 0 0]';% u, p
    elseif t >=35.0 && t <55.0
        uk = [-10*sin(2*pi/20*t) 0 0 -sin(2*pi/20*t) 0 0].';% u, p 
    elseif t >=55.0 && t < 75.0
        uk = [0 0 -5*sin(2*pi/10*t) 0 -0.5*sin(2*pi/10*t) 0].';% w, q
    elseif t >=75.0 && t < 95.0
        uk = [0 0 -8*sin(2*pi/20*t) 0 -1*sin(2*pi/20*t) 0].';% w, q
    elseif t >=95.0 && t <115.0
        uk = [0 -5*sin(2*pi/10*t) 0 0 0 -1*sin(2*pi/10*t)].';% v, r  
    elseif t >=115.0 && t <135.0
        uk = [0 -10*sin(2*pi/20*t) 0 0 0 -0.4*sin(2*pi/20*t)].';% v, r 
    elseif t >= 135.0 && t < 155.0
        uk = [0 -15*sin(2*pi/15*t) 0 0 0 0].';% v
    elseif t >= 155.0 && t < 175.0
        uk = [0 0 0 0 0 -0.2*sin(2*pi/20*t)].';% r       
    else
        uk = [0 0 0 0 0 0].';
    end

    % dataset 1
    % if t >= 0 && t < 500
    %     uk = getControlInput(t, SinWave);
    % elseif t >= 500 && t < 1000   
    %     uk = getControlInput(t-500, StaticAccDcc);  
    % elseif t >= 1000 && t < 1500
    %     uk = getControlInput(t-1000, Turning);
    % elseif t >= 1500 && t < 2000   
    %     uk = getControlInput(t-1500, Spiral);
    % elseif t >= 2000 && t < 2500
    %     uk = getControlInput(t-2000, Zigzag);
    % elseif t >= 2500 && t < 2780
    %      uk = getControlInput(t-2500, Period3211);
    % end
    
    % dataset 2
    % if t >= 0 && t < 500
    %     uk = getControlInput(t, Turning);
    % elseif t >= 500 && t < 1000   
    %     uk = getControlInput(t-500, Spiral);  
    % elseif t >= 1000 && t < 1500
    %     uk = getControlInput(t-1000, StaticAccDcc);
    % elseif t >= 1500 && t < 1780   
    %     uk = getControlInput(t-1500, Period3211);
    % elseif t >= 1780 && t < 2280
    %     uk = getControlInput(t-1780, Zigzag);
    % elseif t >= 2280 && t < 2780
    %      uk = getControlInput(t-2280, SinWave);
    % end

    % dataset 3 
    % if t >= 0 && t < 280
    %     uk = getControlInput(t, Period3211);
    % elseif t >= 280 && t < 780   
    %     uk = getControlInput(t-280, Zigzag);  
    % elseif t >= 780 && t < 1280
    %     uk = getControlInput(t-780, SinWave);
    % elseif t >= 1280 && t < 1780   
    %     uk = getControlInput(t-1280, StaticAccDcc);
    % elseif t >= 1780 && t < 2280
    %     uk = getControlInput(t-1780, Spiral);
    % elseif t >= 2280 && t < 2780
    %      uk = getControlInput(t-2280, Turning);
    % end

    % dataset 4
    % if t >= 0 && t < 500
    %     uk = getControlInput(t, StaticAccDcc);
    % elseif t >= 500 && t < 780   
    %     uk = getControlInput(t-500, Period3211);  
    % elseif t >= 780 && t < 1280
    %     uk = getControlInput(t-780, Turning);
    % elseif t >= 1280 && t < 1780   
    %     uk = getControlInput(t-1280, Spiral);
    % elseif t >= 1780 && t < 2280
    %     uk = getControlInput(t-1780, SinWave);
    % elseif t >= 2280 && t < 2780
    %      uk = getControlInput(t-2280, Zigzag);
    % end

    % dataset 5
    % if t >= 0 && t < 500
    %     uk = getControlInput(t, Zigzag);
    % elseif t >= 500 && t < 1000   
    %     uk = getControlInput(t-500, Spiral);  
    % elseif t >= 1000 && t < 1280
    %     uk = getControlInput(t-1000, Period3211);
    % elseif t >= 1280 && t < 1780   
    %     uk = getControlInput(t-1280, Turning);
    % elseif t >= 1780 && t < 2280
    %     uk = getControlInput(t-1780, SinWave);
    % elseif t >= 2280 && t < 2780
    %      uk = getControlInput(t-2280, StaticAccDcc);
    % end
    
    % prediction dataset
    % if t >= 0 && t < 500
    %     uk = getControlInput(t, Spiral);
    % elseif t >= 500 && t < 1000  
    %     uk = getControlInput(t-500, Turning);  
    % elseif t >= 1000 && t < 1500
    %     uk = getControlInput(t-1000, Zigzag);
    % elseif t >= 1500 && t < 2000   
    %     uk = getControlInput(t-1500, StaticAccDcc);
    % end


    % 6-DoF sin wave thrust
    % if t >= 10 && t <=20
    %     uk = [-5*sin(2*pi/20*t) 0 0 -0.5*sin(2*pi/20*t) 0 0].';% u p
    % elseif t >= 20 && t < 30
    %     uk = [-10*sin(2*pi/20*t) 0 0 -sin(2*pi/20*t) 0 0].';% u p
    % elseif t >= 30 && t < 50
    %     uk = [0 -2*sin(2*pi/20*t) 0 0 0 0].';% v  
    % elseif t >= 50 && t < 70
    %     uk = [0 0 -2*sin(2*pi/20*t) 0 0 0].';% w
    % elseif t >= 70 && t < 90
    %     uk = [0 0 -3*sin(2*pi/20*t) 0 0 0].';% w 
    % elseif t >= 90 && t < 110
    %     uk = [0 0 0 0 0 -0.5*sin(2*pi/20*t)].';% r
    % elseif t >= 110 && t < 130
    %     uk = [0 0 0 0 0 -sin(2*pi/20*t)].';% r 
    % elseif t >= 130 && t < 140
    %     uk = [0 0 0 0 -sin(2*pi/20*t) 0].';% q
    % elseif t >= 140 && t < 160
    %     uk = [5*sin(2*pi/20*t) 0 2*sin(2*pi/20*t) 0 0 0].';% u w
    % elseif t >= 160 && t < 180
    %     uk = [10*sin(2*pi/20*t) 0 3*sin(2*pi/20*t) 0 0 0].';% u w
    % elseif t >= 180 && t < 200
    %     uk = [6*sin(2*pi/20*t) 0 0 0 sin(2*pi/20*t) 0].';% u q
    % elseif t >= 200 && t < 220
    %     uk = [8*sin(2*pi/20*t) 0 0 0 0.5*sin(2*pi/20*t) 0].';% u q
    % elseif t >= 220 && t < 240
    %     uk = [4*sin(2*pi/20*t) sin(2*pi/20*t) 0 0 0 0].';% u v
    % elseif t >= 240 && t < 260
    %     uk = [2*sin(2*pi/20*t) 2*sin(2*pi/20*t) 0 0 0 0].';% u v
    % elseif t >= 260 && t < 280
    %     uk = [3*sin(2*pi/20*t) 0 0 0 0 0.5*sin(2*pi/20*t)].';% u r
    % elseif t >= 280 && t < 300
    %     uk = [2*sin(2*pi/20*t) 0 0 0 0 sin(2*pi/20*t)].';% u r
    % elseif t >= 300 && t < 320
    %     uk = [2*sin(2*pi/20*t)  0.5*sin(2*pi/20*t) 0 0 0 sin(2*pi/20*t)].';% u v r
    % elseif t >= 320 && t < 340
    %     uk = [3*sin(2*pi/20*t) sin(2*pi/20*t) 0 0 0 0.5*sin(2*pi/20*t)].';% u v r
    % elseif t >= 340 && t < 360
    %     uk = [3*sin(2*pi/20*t) sin(2*pi/20*t) sin(2*pi/20*t) 0 0 0].';% u v w
    % elseif t >= 360 && t < 380
    %     uk = [sin(2*pi/20*t) 0.5*sin(2*pi/20*t) sin(2*pi/20*t) 0.5*sin(2*pi/20*t) 0 0].';% u v w p
    % elseif t >= 380 && t < 400
    %     uk = [sin(2*pi/20*t) 0 0 0 0 0].';% u
    % elseif t >= 400 && t < 420
    %     uk = [0 sin(2*pi/20*t) 0 0 0 0].';% v
    % elseif t >= 420 && t < 440
    %     uk = [0 0 0 sin(2*pi/20*t) 0 0].';% p
    % elseif t >= 440 && t < 490
    %     uk = [sin(2*pi/20*t) 0 2*sin(2*pi/20*t) 0 0 sin(2*pi/20*t)].';% u w r
    % else
    %     uk = [0 0 0 0 0 0].';
    % end

    % static acceleration-deceleration
    % if t >= 0 && t < 10
    %     uk = [0 0 0 0 0 0].';% stop initial
    % elseif t >= 10 && t <=20
    %     uk = [1 0 0 0 0 0].';% u 
    % elseif t >= 20 && t < 30
    %     uk = [3 0 0 0 0 0].';
    % elseif t >= 30 && t < 50
    %     uk = [2 0 0 0 0 0].';
    % elseif t >= 50 && t < 70
    %     uk = [5 0 0 0 0 0].';
    % elseif t >= 70 && t < 90
    %     uk = [1 0 0 0 0 0].';
    % elseif t >= 90 && t < 100
    %     uk = [0 0 0 0 0 0].';% stop
    % elseif t >= 100 && t < 110
    %     uk = [0 0.5 0 0 0 0].';%v
    % elseif t >= 110 && t < 130
    %     uk = [0 1 0 0 0 0].'; 
    % elseif t >= 130 && t < 140
    %     uk = [0 0.5 0 0 0 0].';
    % elseif t >= 140 && t < 160
    %     uk = [0 3 0 0 0 0].';
    % elseif t >= 160 && t < 180
    %     uk = [0 5 0 0 0 0].'; 
    % elseif t >= 180 && t < 200
    %     uk = [0 2 0 0 sin(2*pi/20*t) 0].';
    % elseif t >= 200 && t < 210
    %     uk = [0 0 0 0 0 0].';% stop
    % elseif t >= 210 && t < 220
    %     uk = [0 0 1 0 0 0].';% w
    % elseif t >= 220 && t < 240
    %     uk = [0 0 3 0 0 0].';
    % elseif t >= 240 && t < 260
    %     uk = [0 0 6 0 0 0].';
    % elseif t >= 260 && t < 280
    %     uk = [0 0 4 0 0 0].';
    % elseif t >= 280 && t < 300
    %     uk = [0 0 1 0 0 0].';
    % elseif t >= 300 && t < 310
    %     uk = [0 0 0 0 0 0].';% stop
    % elseif t >= 310 && t > 320
    %     uk = [0 0 0 0.2 0 0].';% p
    % elseif t >= 320 && t < 340
    %     uk = [0 0 0 0.5 0 0].';
    % elseif t >= 340 && t < 360
    %     uk = [0 0 0 -0.5 0 0].';
    % elseif t >= 360 && t < 380
    %     uk = [0 0 0 -0.2 0 0].';
    % elseif t >= 380 && t < 400
    %     uk = [0 0 0 0.2 0 0].';
    % elseif t >= 400 && t < 410
    %     uk = [0 0 0 0 0 0].';% stop
    % elseif t >= 410 && t < 420
    %     uk = [0 0 0 0 0.5 0].';% q
    % elseif t >= 420 && t < 440
    %     uk = [0 0 0 0 1 0].';
    % elseif t >= 440 && t < 460
    %     uk = [0 0 0 0 0.3 0].';
    % elseif t >= 460 && t < 470 
    %     uk = [0 0 0 0 0 0].';% stop
    % elseif t >= 470 && t < 480
    %     uk = [0 0 0 0 0 0.5].';% r
    % elseif t >= 480 && t < 490
    %     uk = [0 0 0 0 0 1].';
    % else
    %     uk = [0 0 0 0 0 0].';
    % end

    % turning
    % if t >= 0 && t < 10
    %     uk = [0 0 0 0 0 0].';% stop initial
    % elseif t >= 10 && t <=30
    %     uk = [1 0 0 0 0 0].';% u
    % elseif t >= 30 && t < 100
    %     uk = [1 0 0 0 0 1].';% u r
    % elseif t >= 100 && t < 120
    %     uk = [1 0 0 0 0 0].';
    % elseif t >= 100 && t < 180
    %     uk = [1 0 0 0 0 2].'; 
    % elseif t >= 180 && t < 200
    %     uk = [2 0 0 0 0 0].';
    % elseif t >= 200 && t < 280
    %     uk = [2 0 0 0 0 0.5].';
    % elseif t >= 280 && t < 300
    %     uk = [3 0 0 0 0 0].';
    % elseif t >= 300 && t < 380
    %     uk = [3 1 0 0 0 0].';% u v
    % elseif t >= 380 && t < 400
    %     uk = [4 0 0 0 0 0].';
    % elseif t >= 400 && t < 480
    %     uk = [4 1 0 0 0 1].';% u v r
    % elseif t >= 480 && t < 490
    %     uk = [0 2 0 0 0 1].';% v r
    % else
    %     uk = [0 0 0 0 0 0].';% stop
    % end

    % spiral
    % if t >= 10 && t <=20
    %     uk = [3 0 0 0 0 0].';% u 
    % elseif t >= 20 && t < 80
    %     uk = [3 0 0 0 0 1].';% u (v) r
    % elseif t >= 80 && t < 160
    %     uk = [3*exp(0.02*(t-80)) 0 0.5 0 0 1].';% u (v) w r 
    % elseif t >= 160 && t < 220
    %     uk = [3*exp(0.02*(t-160)) 0 1 0 0 1.5].'; 
    % elseif t >= 220 && t < 280
    %     uk = [1 0 0.5 0 0 1*exp(0.02*(t-220))].';
    % elseif t >= 280 && t < 340
    %     uk = [1 0.2*exp(0.02*(t-280)) 0.5 0 0 0].';% u v w (r)
    % elseif t >= 340 && t < 400
    %     uk = [1 0.2*exp(0.02*(t-340)) 0.5*exp(0.02*(t-340)) 0 0 0].';
    % elseif t >= 400 && t < 490
    %     uk = [1*exp(0.02*(t-360)) 0.2*exp(0.02*(t-360)) 0.5 0 0 0.5*exp(0.02*(t-360))].';% u v w r
    % else
    %     uk = [0 0 0 0 0 0].';
    % end

    % zigzag

    % 3-2-1-1 

    % under the water z >= 0
    % for the model is based on submerged vehicle
    % on the surface, assume that w>=0, q<=0, theta<=0
%      if xk(9, k+1) < 0
%          if abs(xk(9, k+1)) < d
%              b = (1 - abs(xk(9, k+1))/d)*W;
%          else
%              b = 0;
%              uk(3,:) = 0 ;
%          end
% 
%      elseif xk(9, k+1) >= 0
%          b = W;
%      end

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

kalman_input = [xk.', tau.'];
% LSTM 
input_1 = reshape(xk, [1, 12, step+1]); 
input_2 = reshape(tau, [1, 6, step+1]);
output = reshape(nu_dot(1:6,:), [1, 6, step+1]);

function uk = getControlInput(t, ukStruct)
    time_step = find(t >= [ukStruct.t_start] & t < [ukStruct.t_end], 1);

    if ~isempty(time_step)
        uk = ukStruct(time_step).uk(t);
    else
        uk = [0 0 0 0 0 0].'; % Default value when time is outside any block
    end
end
