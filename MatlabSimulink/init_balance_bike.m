% ============================================================
% 平衡小车 Cascaded PID — 全连续域 Laplace 模型
% 电机参数匹配驱动 ~1kg 飞轮的小型直流有刷电机 (775级量级)
% ============================================================
clear; clc;

% ---- 采样周期 (仅用于换算离散PID参数→连续域, 仿真本身用连续求解器) ----
Ts = 0.005;        % 代码中的 5ms

% ============================================================
% 一、连续域 PID 参数 (从代码离散参数换算)
%
%  代码离散公式 (前向欧拉+后向差分):
%    I[n] = I[n-1] + code_Ki * e[n]
%    D[n] = code_Kd * (e[n] - e[n-1])
%
%  连续域并行 PID 传递函数:
%    C(s) = Kp + Ki_cont/s + Kd_cont*s
%
%  换算关系:
%    code_Ki = Ki_cont * Ts   →  Ki_cont = code_Ki / Ts
%    code_Kd = Kd_cont / Ts   →  Kd_cont = code_Kd * Ts
%    Kp 不变
% ============================================================

% --- vel_encoder (速度外环, 完整 PID) ---
vel_Kp = 0.065;
vel_Ki_cont = 0.000045 / Ts;     % = 0.009  (1/s)
vel_Kd_cont = 0.015 * Ts;        % = 7.5e-5 (s)
vel_integral_max = 500;
vel_out_max = 2000;

% --- rol_angle (角度中环, 纯 P) ---
angle_Kp = 7.5;
angle_Ki_cont = 0.0;
angle_Kd_cont = 0.0;
angle_integral_max = 550;
angle_out_max = 2000;

% --- rol_gyro (角速度内环, 纯 P) ---
gyro_Kp = 26.5;
gyro_Ki_cont = 0.0;
gyro_Kd_cont = 0.0;
gyro_integral_max = 500;
gyro_out_max = 2000;

fprintf('=== PID 连续域参数 ===\n');
fprintf('Vel_PID:   C(s) = %.4f + %.4f/s + %.6f*s\n', vel_Kp, vel_Ki_cont, vel_Kd_cont);
fprintf('Angle_PID: C(s) = %.1f  (纯增益)\n', angle_Kp);
fprintf('Gyro_PID:  C(s) = %.1f  (纯增益)\n', gyro_Kp);

% ============================================================
% 二、直流电机参数 (驱动 ~1kg 飞轮, 775级有刷电机量级)
% ============================================================

V_supply = 12.0;     % 供电电压 (V)
%Ra       = 3.0;      % 电枢电阻 (Ω), insufficient torque
Ra       = 0.5;
La       = 0.002;    % 电枢电感 (H)
%Kt       = 0.01;     % 力矩常数 (N·m/A), insufficient torque
Kt       = 0.08;
%Ke       = 0.01;     % 反电动势常数 (V/(rad/s)), SI单位下 Kt=Ke, insufficient torque
Ke       = 0.08;

PWM_to_V = V_supply / 2000;   % = 0.006 V/PWM单位

% ============================================================
% 三、飞轮参数 (~1kg 黄铜/钢圆盘, 直径 ~8cm)
% ============================================================

m_fly   = 1.0;               % 飞轮质量 (kg)
r_fly   = 0.04;              % 飞轮半径 (m)
J_fly   = 0.5*m_fly*r_fly^2; % 飞轮转动惯量 (kg·m²)
J_rotor = 1e-5;              % 转子惯量
J_total = J_fly + J_rotor;   % ≈ 8.1e-4 kg·m²
B_fly   = 1e-4;              % 轴承粘滞摩擦 (N·m/(rad/s))

fprintf('\n=== 飞轮 ===\n');
fprintf('J_total = %.2e kg·m²\n', J_total);

% ============================================================
% 四、电机传递函数 (从电压到飞轮转速)
%
%  电气: I(s) = V_net(s) / (La·s + Ra)
%  机械: ω(s) = τ(s) / (J_total·s + B)
%  力矩: τ(s) = Kt · I(s)
%  反电动势: Vbemf = Ke · ω
%
%  闭环传递函数 (含反电动势):
%    ω(s)/V(s) = Kt / [(La·s+Ra)(J_total·s+B) + Kt·Ke]
%
%  展开分母: La·J_total·s² + (La·B + Ra·J_total)·s + (Ra·B + Kt·Ke)
% ============================================================

% --- 电机电气子传递函数 ---
motor_elec_num = [1];                    % I/V_net
motor_elec_den = [La, Ra];              % 1/(La·s + Ra)

% --- 飞轮机械子传递函数 ---
fly_mech_num = [1];                     % ω/τ
fly_mech_den = [J_total, B_fly];        % 1/(J_total·s + B)

% --- 电机完整传递函数 (仅用于分析, 不直接用于模型) ---
motor_full_num = Kt;
motor_full_den = [La*J_total, ...
                  La*B_fly + Ra*J_total, ...
                  Ra*B_fly + Kt*Ke];

fprintf('\n=== 电机传递函数 ω(s)/V(s) ===\n');
fprintf('           %.4f\n', motor_full_num);
fprintf('  = ───────────────────────────\n');
fprintf('     %.2e s² + %.4f s + %.4f\n', ...
    motor_full_den(1), motor_full_den(2), motor_full_den(3));

% 极点分析
motor_poles = roots(motor_full_den);
fprintf('  极点: s₁ = %.2f rad/s (机械主导, τ≈%.1fs)\n', ...
    max(motor_poles), -1/max(motor_poles));
fprintf('        s₂ = %.0f rad/s (电气, τ≈%.1fms)\n', ...
    min(motor_poles), -1000/min(motor_poles));

% ============================================================
% 五、车身动力学 [倒立摆]
%
%  运动方程: J_body·θ̈ = mgh·θ - τ_motor
%  拉普拉斯变换:
%    θ(s)/τ_motor(s) = -1 / (J_body·s² - mgh)
%                    = -(1/J_body) / (s² - mgh/J_body)
%
%  不稳定极点位于 s = ±√(mgh/J_body)
% ============================================================

m_body  = 3.0;          % 车身质量 (kg)
h_body  = 0.15;         % 质心高度 (m)
g       = 9.81;
mgh     = m_body * g * h_body;           % ≈ 4.41 N·m/rad
J_body  = m_body * h_body^2;             % ≈ 0.068 kg·m²

body_num = -1/J_body;                    % -(1/J_body)
body_den = [1, 0, -mgh/J_body];          % s² - mgh/J_body

fprintf('\n=== 车身传递函数 θ(s)/τ(s) ===\n');
fprintf('          %.4f\n', body_num);
fprintf('  = ──────────────\n');
fprintf('     s² - %.4f\n', mgh/J_body);
fprintf('  极点: s = ±%.2f rad/s (不稳定!)\n', sqrt(mgh/J_body));

% ============================================================
% 六、编码器与单位转换
% ============================================================

PPR   = 4000;                            % 正交编码器每转脉冲数 (4倍频后)
K_enc = PPR * Ts / (2*pi);               % (rad/s → counts/5ms) ≈ 3.183

rad2deg = 180/pi;                        % rad → °

% ============================================================
% 七、初始条件与偏置
% ============================================================

theta0_rad = 0.05;       % 初始车身倾角 (rad), ≈2.86°
gyro_bias  = 0.0;        % 陀螺仪残余偏置 (°/s), 设为 0.03 可观察偏置效应

% ============================================================
% 八、仿真设置
% ============================================================

sim_stop_time = 10;      % 仿真 10 秒

fprintf('\n参数初始化完成。\n');