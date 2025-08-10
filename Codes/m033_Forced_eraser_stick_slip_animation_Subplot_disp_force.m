clc; clear; close all;

%% 参数设定 Parameters
m = 0.1;               % 质量 Mass (kg)              % 质量 (kg)
k = 100;               % 弹簧刚度 Spring stiffness (N/m)  % 弹簧刚度 (N/m)
c = 0.5;               % 阻尼系数 Damping coefficient    % 阻尼系数 (N·s/m)
F0 = 1.0;              % 驱动力幅值 Driving force amplitude % 驱动力幅值 (N)
f = 5.03;              % 驱动频率 Driving frequency (Hz)   % 驱动频率 (Hz)
omega = 2*pi*f;        % 驱动角频率 Angular frequency       % 驱动角频率 (rad/s)
dt = 0.002;            % 时间步长 Time step (s)            % 时间步长 (s)
t_end = 10;            % 仿真总时间 Total simulation time     % 仿真总时间 (s)
margin = 0.1;         % 动态 x 轴窗口边界 Window margin    % 动态 x 轴窗口边界 (m)

%% 初始条件 Initial conditions
x = 0;                 % 位移 Displacement            % 位移 (m)
v = 0;                 % 速度 Velocity                 % 速度 (m/s)
time = 0:dt:t_end;     
N = length(time);
x_save = zeros(size(time));
F_drive = zeros(size(time));
gif_filename = 'm033_Forced_eraser_stick_slip_animation_Subplot_disp_force.gif';  % 保存的 GIF 文件名

%% 数值积分求解 Forced vibration integration
for i = 1:N
    t = time(i);
    % 外力 F = F0 * cos(omega * t)
    Fd = F0 * cos(omega * t);
    % 弹簧力 Fs = -k * x
    Fs = -k * x;
    % 加速度 a = (F_drive + F_spring - c*v) / m
    a = (Fd + Fs - c * v) / m;
    % 前向欧拉积分
    v = v + a * dt;
    x = x + v * dt;

    x_save(i) = x;
    F_drive(i) = Fd;
end

%% 绘图与动画设置 / Plot & Animation Setup
hFig = figure('Position',[100,100,1300,400], ...
              'CloseRequestFcn',@onFigureClose);  % 点击 X 时调用 / Close callback

% 左侧：动画动画子图 Animation subplot
subplot(1,2,1);
hold on; grid on; axis equal;
xlabel('位移 (m)'); ylabel('高度 (m)');
title('质量-弹簧-阻尼 强迫振动动画');
xlim([-margin, margin]);
ylim([-0.02, 0.04]);
% 地面
plot([-margin, margin], [-0.005, -0.005], 'k', 'LineWidth',2);
% 质量块
mass_rect = rectangle('Position',[x_save(1)-0.005/2, -0.005, 0.01, 0.01], ...
    'FaceColor',[0.85 0.33 0.1]);
% 弹簧
[sx, sy] = getSpringPoints(0, x_save(1), 0.005, 4, 0.002, 50);
spring_line = plot(sx, sy, 'm', 'LineWidth',2);
% 驱动力箭头
drive_arrow = quiver(x_save(1), 0.015, F_drive(1)*0.01, 0, ...
    'LineWidth',2, 'Color','b', 'MaxHeadSize',2);
drive_text = annotation('textbox',[0.05,0.80,0.15,0.05], ...
    'String','驱动力: 0 N','EdgeColor','none','FontSize',10,'Color','b');

% 右侧：位移与驱动力随时间变化 Time response subplot
subplot(1,2,2);
hold on; grid on;
title('位移与驱动力随时间变化');
xlabel('时间 (s)');
yyaxis left
ylabel('位移 (m)');
ylim([-0.1, 0.1]);
h_disp = plot(time, x_save, 'r', 'LineWidth',1.2);
yyaxis right
ylabel('驱动力 (N)');
h_drive = plot(time, F_drive, 'b--', 'LineWidth',1.2,'Color',[0 0 1 0.5]);
legend([h_disp,h_drive], {'位移 x','驱动力 F_0 cos(\omega t)'}, ...
    'Location','northeast');
xlim([0 t_end]);

% 动画循环
for i = 1:10:N
    % —— 检查窗口是否已被关闭 ——  
    if ~ishandle(hFig)
        break;   % 窗口不存在了，跳出循环
    end
    t = time(i);
    % 更新左侧动画
    subplot(1,2,1);
    mass_rect.Position = [x_save(i)-0.005/2, -0.005, 0.01, 0.01];
    [sx, sy] = getSpringPoints(0, x_save(i), 0.005, 4, 0.002, 50);
    set(spring_line, 'XData',sx,'YData',sy);
    set(drive_arrow, ...
        'XData', x_save(i), ...
        'UData', F_drive(i)*0.01);
    drive_text.String = sprintf('驱动力: %.2f N', F_drive(i));

    % 更新右侧曲线
    subplot(1,2,2);
    yyaxis left
    set(h_disp, 'XData', time(1:i), 'YData', x_save(1:i));
    yyaxis right
    set(h_drive, 'XData', time(1:i), 'YData', F_drive(1:i));

    drawnow;
    % —— 保存为 GIF ——  
    if t <= 5   % 只保存 5 秒内的动画
        frame = getframe(hFig);                 % 捕获当前 figure 内容
        im = frame2im(frame);                   % 转换为图像矩阵
        [imind, cm] = rgb2ind(im, 256);          % 转为索引图（256 色）
        delay_time = dt * 10;                    % 每帧延时 (s)，对应 i 步长=10
    
        if i == 1
            imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', delay_time);
        else
            imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', delay_time);
        end
    end

end

%% --- 辅助函数：生成弹簧点坐标 ---  
function [xs, ys] = getSpringPoints(x_start, x_end, y0, N_coils, amplitude, numPoints)
    % getSpringPoints 生成弹簧曲线
    % xs, ys = getSpringPoints(x_start, x_end, y0, N_coils, amplitude, numPoints)
    % x_start,x_end：弹簧两端 x 坐标
    % y0：弹簧中心线垂直位置
    % N_coils：圈数
    % amplitude：振幅
    % numPoints：采样点数

    t = linspace(0,1,numPoints);
    xs = linspace(x_start, x_end, numPoints);
    ys = y0 + amplitude * sin(2*pi*N_coils*t);
end
%% —— 回调函数定义 ——  
function onFigureClose(src, ~)
    % 当点击 X 时触发
    % 直接删除 Figure，主循环里检测到无法句柄就会退出
    delete(src);
end