function K = get_k_length(leg_length)
   
    %theta : 摆杆与竖直方向夹角/             R   ：驱动轮半径//
    %x     : 驱动轮位移/                    L   : 摆杆重心到驱动轮轴距离//
    %phi   : 机体与水平夹角/                LM  : 摆杆重心到其转轴距离//
    %T     ：驱动轮输出力矩/                 l   : 机体重心到其转轴距离//
    %Tp    : 髋关节输出力矩/                 mw  : 驱动轮转子质量//
    %N     ：驱动轮对摆杆力的水平分量/        mp  : 摆杆质量//
    %P     ：驱动轮对摆杆力的竖直分量        M   : 机体质量//
    %Nm    ：摆杆对机体力水平方向分量        Iw  : 驱动轮转子转动惯量//
    %Pm    ：摆杆对机体力竖直方向分量        Ip  : 摆杆绕质心转动惯量//
    %Nf    : 地面对驱动轮摩擦力             Im  : 机体绕质心转动惯量//

    syms theta(t) x(t) phi(t) T Tp R L LM l mw mp M Iw Ip IM g  
    syms f1 f2 f3 d_theta d_x d_phi theta0 x0 phi0 

    R1=0.075;                         %驱动轮半径
    L1=leg_length/2;                  %摆杆重心到驱动轮轴距离
    LM1=leg_length/2;                 %摆杆重心到其转轴距离
    l1=0.257;                         %机体质心距离转轴距离
    mw1=1.18;                         %驱动轮质量
    mp1=1.308;                          %杆质量
    M1=17.14;                         %机体质量
   Iw1=0.001618196;                   %驱动轮转动惯量
   Ip1=(1/12) * mp1 * ((L1 + LM1)^2 + 0.05 ^ 2);                  %摆杆转动惯量
   IM1=0.255756269;                   %机体绕质心转动惯量

    NM = M*diff(x + (L + LM )*sin(theta)-l*sin(phi),t,2);
    N = NM + mp*diff(x + L*sin(theta),t,2);
    PM = M*g + M*diff((L+LM)*cos(theta)+l*cos(phi),t,2);
    P = PM +mp*g+mp*diff(L*cos(theta),t,2);

    eqn1 = diff(x,t,2) == (T -N*R)/(Iw/R + mw*R);
    eqn2 = Ip*diff(theta,t,2) == (P*L + PM*LM)*sin(theta)-(N*L+NM*LM)*cos(theta)-T+Tp;
    eqn3 = IM*diff(phi,t,2) == Tp +NM*l*cos(phi)+PM*l*sin(phi);

    % θ≈0,ϕ≈0……
    eqn10 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn1,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);
    eqn20 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn2,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);
    eqn30 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn3,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);

    %求解q''=M^(-1)F
    [f1,f2,f3] = solve(eqn10,eqn20,eqn30,f1,f2,f3);

    % 读A
    A=subs(jacobian([d_theta,f1,d_x,f2,d_phi,f3],[theta0,d_theta,x0,d_x,phi0,d_phi]),[theta0,d_theta,d_x,phi0,d_phi,T,Tp],[0,0,0,0,0,0,0]);
    A=subs(A,[R,L,LM,l,mw,mp,M,Iw,Ip,IM,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.8]);
    A=double(A);%变成浮点数矩阵
    % 读B
    B=subs(jacobian([d_theta,f1,d_x,f2,d_phi,f3],[T,Tp]),[theta0,d_theta,d_x,phi0,d_phi,T,Tp],[0,0,0,0,0,0,0]);
    B=subs(B,[R,L,LM,l,mw,mp,M,Iw,Ip,IM,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.8]);
    B=double(B);%变成浮点数矩阵


%     Q = diag([1 1 50 1 15000 1]);
    % Q = diag([1 1 700 1 30000 1]);

%     Q = diag([1 1 100 5 6000 1]);



%     Q = diag([200 5 1000 1000 10000 50]);南科大参数



% theta x phi
% Q = diag([1 1 400 1 6000 1]);//
%  Q = diag([100 1 400 1 6000 1]);

  Q = diag([1 1 400 1 4000 1]);


    R = [1 0;0 0.25]; % 关节大，轮毂小
    
    K = lqr(A,B,Q,R); % lqr函数返回一个 p x n 的矩阵
  
end










% 
% function K = get_k_length(leg_length)
%  
%     % 定义符号变量
%     syms x(t) x_dot(t) yaw(t) yaw_dot(t) theta_l(t) theta_l_dot(t) ...
%          theta_r(t) theta_r_dot(t) pit(t) pit_dot(t)
%     syms Tl Tr Tpl Tpr R L LM l mw mp M Iw Ip IM g Iyaw
%     
%     % 物理参数（使用与单轮腿相同的参数）
%     R1 = 0.075;                         % 驱动轮半径
%     L1 = leg_length/2;                  % 摆杆重心到驱动轮轴距离
%     LM1 = leg_length/2;                 % 摆杆重心到其转轴距离
%     l1 = 0.257;                         % 机体质心距离转轴距离
%     mw1 = 1.18;                         % 驱动轮质量
%     mp1 = 1.308;                        % 杆质量
%     M1 = 17.14;                         % 机体质量
%     Iw1 = 0.001618196;                  % 驱动轮转动惯量
%     Ip1 = 0.036630621;                  % 摆杆转动惯量
%     IM1 = 0.255756269;                  % 机体绕质心转动惯量（俯仰）
%     Iyaw1 = 0.1;                        % 机体偏航转动惯量（需要根据实际结构确定）
%     g1 = 9.8;                           % 重力加速度
%     W = 0.4918;                         % 轮间距，需要根据实际机器人确定
% 
%     % 状态变量替换
%     syms d_x d_yaw d_theta_l d_theta_r d_pit x0 yaw0 theta_l0 theta_r0 pit0
%     syms f_x f_yaw f_theta_l f_theta_r f_pit
%     
%     % 左右轮的运动学关系
%     
%     % 左右轮的位置
%     x_l = x - (W/2)*sin(yaw);  % 左轮位置
%     x_r = x + (W/2)*sin(yaw);  % 右轮位置
%     
%     % 轮子动力学方程（左右对称）
%     eqn_wheel_l = Iw*diff(theta_l,t,2) == Tl - mw*g*R*sin(pit) - mw*R^2*diff(theta_l,t,2);
%     eqn_wheel_r = Iw*diff(theta_r,t,2) == Tr - mw*g*R*sin(pit) - mw*R^2*diff(theta_r,t,2);
%     
%     % 摆杆动力学方程（左右腿）
%     % 左腿
%     N_l = mp*diff(x_l + L*sin(theta_l),t,2);
%     P_l = mp*g + mp*diff(L*cos(theta_l),t,2);
%     eqn_leg_l = Ip*diff(theta_l,t,2) == P_l*L*sin(theta_l) - N_l*L*cos(theta_l) - Tpl;
%     
%     % 右腿
%     N_r = mp*diff(x_r + L*sin(theta_r),t,2);
%     P_r = mp*g + mp*diff(L*cos(theta_r),t,2);
%     eqn_leg_r = Ip*diff(theta_r,t,2) == P_r*L*sin(theta_r) - N_r*L*cos(theta_r) - Tpr;
%     
%     % 机身动力学
%     % 俯仰方向
%     NM_l = M*diff(x_l + (L+LM)*sin(theta_l) - l*sin(pit),t,2)/2;
%     NM_r = M*diff(x_r + (L+LM)*sin(theta_r) - l*sin(pit),t,2)/2;
%     PM_l = M*g/2 + M*diff((L+LM)*cos(theta_l) + l*cos(pit),t,2)/2;
%     PM_r = M*g/2 + M*diff((L+LM)*cos(theta_r) + l*cos(pit),t,2)/2;
%     
%     eqn_pitch = IM*diff(pit,t,2) == Tpl + Tpr + ...
%                 (NM_l + NM_r)*l*cos(pit) + (PM_l + PM_r)*l*sin(pit);
%     
%     % 偏航方向
%     eqn_yaw = Iyaw*diff(yaw,t,2) == (Tl - Tr)*R/W - ...
%               (N_l + P_l*tan(theta_l))*(W/2) + (N_r + P_r*tan(theta_r))*(W/2);
%     
%     % 水平方向
%     eqn_x = M*diff(x,t,2) == (N_l + N_r) - (Tl + Tr)/R;
%     
%     % 将二阶微分方程转换为一阶形式
%     % 状态向量: [x, x_dot, yaw, yaw_dot, theta_l, theta_l_dot, theta_r, theta_r_dot, pit, pit_dot]
%     
%     % 替换变量
%     subs_vars = [diff(x,t,2), diff(yaw,t,2), diff(theta_l,t,2), ...
%                  diff(theta_r,t,2), diff(pit,t,2), ...
%                  diff(x,t), diff(yaw,t), diff(theta_l,t), ...
%                  diff(theta_r,t), diff(pit,t), ...
%                  x, yaw, theta_l, theta_r, pit];
%     subs_vals = [f_x, f_yaw, f_theta_l, f_theta_r, f_pit, ...
%                  d_x, d_yaw, d_theta_l, d_theta_r, d_pit, ...
%                  x0, yaw0, theta_l0, theta_r0, pit0];
%     
%     eqn1 = subs(eqn_x, subs_vars, subs_vals);
%     eqn2 = subs(eqn_yaw, subs_vars, subs_vals);
%     eqn3 = subs(eqn_leg_l, subs_vars, subs_vals);
%     eqn4 = subs(eqn_leg_r, subs_vars, subs_vals);
%     eqn5 = subs(eqn_pitch, subs_vars, subs_vals);
%     
%     % 求解加速度项
%     [f_x, f_yaw, f_theta_l, f_theta_r, f_pit] = ...
%         solve([eqn1, eqn2, eqn3, eqn4, eqn5], [f_x, f_yaw, f_theta_l, f_theta_r, f_pit]);
%     
%     % 构建状态空间方程
%     state_derivatives = [d_x; f_x; d_yaw; f_yaw; d_theta_l; f_theta_l; ...
%                         d_theta_r; f_theta_r; d_pit; f_pit];
%     
%     states = [x0, d_x, yaw0, d_yaw, theta_l0, d_theta_l, ...
%              theta_r0, d_theta_r, pit0, d_pit];
%     controls = [Tl, Tr, Tpl, Tpr];
%     
%     % 计算雅可比矩阵（线性化）
%     A = jacobian(state_derivatives, states);
%     B = jacobian(state_derivatives, controls);
%     
%     % 在平衡点进行线性化
%     equilibrium = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
%     A_lin = subs(A, [states, controls], equilibrium);
%     B_lin = subs(B, [states, controls], equilibrium);
%     
%     % 代入实际参数
%     param_subs = [R, L, LM, l, mw, mp, M, Iw, Ip, IM, Iyaw, g];
%     param_vals = [R1, L1, LM1, l1, mw1, mp1, M1, Iw1, Ip1, IM1, Iyaw1, g1];
%     
%     A_num = double(subs(A_lin, param_subs, param_vals));
%     B_num = double(subs(B_lin, param_subs, param_vals));
%     
%     % LQR控制器设计
%     Q = diag([1000, 1000, 5, 5, 200, 5, 200, 5, 10000, 50]);
%     R_mat = diag([1, 1, 0.25, 0.25]); % 关节大，轮毂小
%     
%     K = lqr(A_num, B_num, Q, R_mat);
% 
% end