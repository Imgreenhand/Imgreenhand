% MIT model
% for some reason this model do not consider the fisrt-order Damping forces
% like: Xu, Yv, Zw, Kp, Mq, Nr, and the simulation result is not so good 
% comparing with our model
m = 1.97    %lb
W = 63.32
Buoyancy = 63.32
Ix = 0.3
Iy = 2.3
Iz = 2.3

Kpp = -0.8
Kpdot = -0.025
kvdot = 0.0079
Mqq = 6.5
Mqdot = -0.41
Muq = -0.31
Muw = 1.57
Mww = 0
Mwdot = -0.02
Nrr = -5.6
Nrdot = -0.38
Nur = -0.32
Nuv = -1.57
Nvdot = 0

Xuu = -0.84
Xudot = -0.35
Ypdot = 0.0079
Yrdot = 0
Yur = 0.042
Yuv = -0.3
Yvv = -3
Yvdot = -1.48
Zqdot = -0.02
Zuq = -0.042
Zuw = 0.28
Zww = -3.49
Zwdot = -1.32

m11 = m - Xudot
m22 = m - Yvdot
m33 = m - Zwdot
m44 = Ix - Kpdot
m55 = Iy - Mqdot
m66 = Iz - Nrdot

M = [m11            0           0           0           0           0;
     0              m22         0           -Ypdot      0           -Yrdot;
     0              0           m33         0           -Zqdot      0;
     0              -Kvdot      0           m44         0           0;
     0              0           -Mwdot      0           m55         0;
     0              -Nvdot      0           0           0           m66]

% C(nu)+D(nu)
N = -[Xuu*abs(u)                 0                                   0                               0                       -m*w                        m*v;
      0                          Yvv*abs(v)+Yuv*abs(u)               0                               m*w                     0                           -m*u+Yur*u;
      0                          0                                   Zww*abs(w)+Zuw*abs(u)           -m*v                    m*u+Zuq*u                   0;
      0                          0                                   0                               Kpp*abs(p)              0                           (Iy-Iz)*q;
      0                          0                                   Muw*u                           (Iz-Ix)*r               Mqq*abs(q)+Muq*abs(u)       0;
      0                          Nuv*u                               0                               0                       (Ix-Iy)*p                   Nrr*abs(r)+Nur*u]

% G(eta)
Geta = [0          0           0           0                   -W+Buoyancy             0;
        0          0           0           W-Buoyancy          0                       0;
        0          0           0           0                   0                       0;
        0          0           0           ZB*Buoyancy         0                       0;
        0          0           0           0                   ZB*Buoyancy             0;
        0          0           0           0                   0                       0]
Geta = 0
% G
G = [0 0 W-Buoyancy 0 0 0]'

% Bu    tau = [X Y Z K M N]' = Bu*u
Bu = [1     1       0       0;
      0     0       0       0;
      0     0       1       1;
      0     0       -Yvt    Yvt;
      0     0       0       0;
      Yht   -Yht    0       0]

A = [-inv(M)*N      -inv(M)*Geta;
            P         zeros(6)  ]


B = [inv(M);zeros(6)]*Bu

% D(nu)
D = [Du Dv Dw Dp Dq Dr]'

% % 4-th R-K
% t0 = 0
% tfinal = 1
% h = 0.2
% tarray = t0:h:tfinal
% step = length(t0 : h : tfinal-h)
% 
% xk = zeros(12,step+1)
% xk(:,1) = [1, 1, 1, 10*pi/180, 10*pi/180, 10*pi/180, 0, 0, 0, 0, 0, 0]  % initial conditions
% uk = [0   0   0   0]'% u = [Tp   Ts   Tpv   Tsv]
% F = @(Ak, xk, uk) Ak*xk + B*uk - [inv(M);zeros(6)]*(G-D)
% 
% for k = 1:step
%      t = 0 + h*k
%      Ak = subs(A, [u, v, w, p, q, r, phi, theta, psi], [xk(1,k), xk(2,k), xk(3,k), xk(4,k), xk(5,k), xk(6,k), xk(10,k), xk(11,k), xk(12,k)])
%      k1 = F(Ak, xk(:, k), uk);                      % O(h^4)
%      k2 = h*F(Ak, xk(:, k)+k1/2, uk);
%      k3 = h*F(Ak, xk(:, k)+k2/2, uk);
%      k4 = h*F(Ak, xk(:, k)+k3/2, uk);
%      xk(:, k+1) = xk(:, k) + (k1 + 2*k2 + 2*k3 + k4)/6;
% end
% xk(4,:) = xk(4,:)*180/pi
% xk(5,:) = xk(5,:)*180/pi
% xk(6,:) = xk(6,:)*180/pi
% xk(10,:) = xk(10,:)*180/pi
% xk(11,:) = xk(11,:)*180/pi
% xk(12,:) = xk(12,:)*180/pi

% xplot_ylabel = ["u" "v" "w" "p" "q" "r" "x" "y" "z" "phi" "theta" "psi"]
% for i = 1:12
%     if xk(9,i)<0
%         xk(9,i) = 0
%     end
%     subplot(2,6,i)
%     plot(tarray, xk(i,:), 'o-')% 
%     ylabel(xplot_ylabel(i),'rotation',0)
%     title(xplot_title(i))
% %     ylim([-0.2,0.2])
%     grid on
% end