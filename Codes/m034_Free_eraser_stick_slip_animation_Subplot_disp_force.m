clc; clear; close all; % 清理环境 / Clear environment

%% 参数设定 Parameters
m      = 0.1;            % 质量 Mass (kg)            
k      = 100;            % 弹簧刚度 Spring stiffness (N/m)
c      = 0.5;            % 阻尼系数 Damping coefficient (N·s/m)
x0     = 0.03;           % 初始位移 Initial displacement (m)
v0     = 0;              % 初速度 Initial velocity (m/s)
dt     = 0.007;          % 时间步长 Time step (s)
t_end  = 10;             % 仿真总时间 Total simulation time (s)
margin = 0.03;           % 动态 X 轴窗口边界 Window margin (m)

%% 初始条件 / Initial conditions
x       = x0;                   
v       = v0;                   
time    = 0:dt:t_end;          
N       = length(time);        
x_save  = zeros(size(time));   % 位移存储 Displacement storage
F_spring = zeros(size(time));  % 弹簧力存储 Spring force storage
gif_filename = 'm034_Free_eraser_stick_slip_animation_Subplot_disp_force.gif'; % 输出文件名 / Output file name

%% 绘图与动画设置 / Plot & Animation Setup
hFig = figure('Position',[100,100,1300,400], ...
              'CloseRequestFcn',@onFigureClose);  % 点击 X 时调用 / Close callback

% 左侧：动画子图 / Left subplot (animation)
subplot(1,2,1);
hold on; grid on; axis equal;
xlabel('位移 (m)'); ylabel('高度 (m)');
title('质量-弹簧-阻尼 自由振动动画 / Mass‐Spring‐Damper Free Vibration');
xlim([-margin, margin]); ylim([-0.02, 0.04]);
plot([-margin,margin],[-0.005,-0.005],'k','LineWidth',2);  % 地面 / Ground
mass_rect = rectangle('Position',[x-0.005/2,-0.005,0.01,0.01], ...
                      'FaceColor',[0.85 0.33 0.1]);         % 质量块 / Mass block
[sx,sy] = getSpringPoints(0,x,0.005,4,0.002,50);            % 弹簧 / Spring
spring_line = plot(sx,sy,'m','LineWidth',2);

% 显示初始位移 / Display initial displacement
text(-margin+0.005, 0.035, sprintf('初始位移 x_0 = %.2f m', x0), ...
     'FontSize',10, 'Color','k');

% 右侧：位移与弹簧力时程 / Right subplot (time response)
subplot(1,2,2);
hold on; grid on;
title('位移与弹簧力随时间变化 / Displacement & Spring Force vs Time');
xlabel('时间 (s)');
yyaxis left;  ylabel('位移 (m)');
h_disp  = plot(time,x_save,'r','LineWidth',1.5);
yyaxis right; ylabel('弹簧力 (N)');
h_force = plot(time,F_spring,'b--','LineWidth',1.5);
legend([h_disp,h_force],{'位移 x','弹簧力 -k x'}, ...
    'Location','northeast');
xlim([0 t_end]);

%% 数值积分与动画循环 / Integration & Animation Loop
for i = 1:10:N
    if ~ishandle(hFig)  % 检查窗口是否存在 / Exit if closed
        break;
    end

    % —— 计算自由振动 ——  
    a = (-k*x - c*v) / m;      % 加速度 / Acceleration
    v = v + a * dt;            % 速度 / Velocity
    x = x + v * dt;            % 位移 / Displacement

    x_save(i)   = x;           % 存储位移 / Store displacement
    F_spring(i) = -k * x;      % 存储弹簧力 / Store spring force

    % 更新左侧动画 / Update left subplot
    subplot(1,2,1);
    mass_rect.Position = [x-0.005/2,-0.005,0.01,0.01];
    [sx,sy] = getSpringPoints(0,x,0.005,4,0.002,50);
    set(spring_line,'XData',sx,'YData',sy);

    % 更新右侧时程 / Update right subplot
    subplot(1,2,2);
    yyaxis left;  set(h_disp, 'XData',time(1:i),'YData',x_save(1:i));
    yyaxis right; set(h_force,'XData',time(1:i),'YData',F_spring(1:i));

    drawnow;  % 刷新 / Refresh
    % ===== 保存为 GIF / Save to GIF =====
    t = time(i);                          % 当前时间 / current time
    if t <= 10                            % 只导出前 10 秒 / only first 10 s
        % 如果只想截左侧动画区：用 ax = subplot(1,2,1); 再 getframe(ax)
        frame = getframe(hFig);           % 抓取当前图像 / capture current figure
        im = frame2im(frame);             % 转为 RGB 图像 / to RGB image
        [imind, cm] = rgb2ind(im, 256);   % 转为索引色（GIF）/ indexed image for GIF
        delay_time = 1 * dt * 10;             % 帧间隔＝dt×步长10 / per-frame delay
    
        if i == 1                         % 首帧创建 / first frame creates file
            imwrite(imind, cm, gif_filename, 'gif', ...
                    'Loopcount', inf, 'DelayTime', delay_time);
        else                              % 后续帧追加 / append following frames
            imwrite(imind, cm, gif_filename, 'gif', ...
                    'WriteMode', 'append', 'DelayTime', delay_time);
        end
    end
end

% 循环结束后关闭窗口 / Delete figure if still exists
if ishandle(hFig)
    delete(hFig);
end

%% 本地函数 / Local Functions

function onFigureClose(src, ~)
    % 中文：点击 X 时删除窗口
    % EN: Delete figure when close button is clicked
    delete(src);
end

function [xs, ys] = getSpringPoints(x_start, x_end, y0, N_coils, amplitude, numPoints)
    % 中文：生成弹簧坐标点
    % EN: Generate spring coil coordinates
    t  = linspace(0,1,numPoints);
    xs = linspace(x_start, x_end, numPoints);
    ys = y0 + amplitude * sin(2*pi*N_coils*t);
end
