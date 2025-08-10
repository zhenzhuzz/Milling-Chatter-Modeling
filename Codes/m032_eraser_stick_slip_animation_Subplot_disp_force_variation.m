clc; clear; close all;

%% 参数设定
m = 0.1; 
k = 100; 
F0 = 1.0;             % 恒定驱动力 (N)
Fs = 1.2;             % 最大静摩擦力 (N)
Fd_min = 0.4;         % 最小动摩擦力 (N)
v_s = 0.005;          % 速度阈值 (m/s)，滑动状态下低速则重新粘着
v_driver = 0.01;      % 驱动器前进速度 (m/s)
dt = 0.004;          % 时间步长 (s)
t_end = 30;           % 仿真总时间 (s)
margin = 0.03;        % 动态x轴窗口边界

%% 初始条件
x = 0; 
v = 0; 
a = 0; 
state = "stick";
time = 0:dt:t_end;
N = length(time); 
x_save = zeros(size(time));
v_save = zeros(size(time)); 
state_save = strings(size(time));
gif_filename = 'm032_eraser_stick_slip_animation_Subplot_disp_force_variation.gif';  % GIF 文件名

%% 动摩擦力特性（负斜率非线性模型）
Fd_fun = @(v) Fs - (Fs - Fd_min)*tanh(100*abs(v));

%% 数值积分求解运动方程
for i = 1:N
    t = time(i);
    x_driver = v_driver * t;
    F_spring = k*(x_driver - x);
    
    if state == "stick"
        friction_force = -F_spring;
        v = 0; a = 0;
        if abs(F_spring) > Fs, state = "slip"; end
    else
        Fd = Fd_fun(v);
        friction_force = - Fd * sign(v);
        if v == 0, friction_force = -Fd; end
        a = (F_spring + friction_force)/m;
        v = v + a * dt;
        x = x + v * dt;
        if abs(v) < v_s && abs(F_spring) < Fs
            state = "stick"; v = 0; a = 0;
        end
    end
    x_save(i) = x; 
    v_save(i) = v; 
    state_save(i) = state;
end

% 预先计算弹簧力数组，用于右侧子图显示
F_spring_array = k*(v_driver*time - x_save);

%% 动画展示
hFig = figure('Position',[100,100,1300,400], ...
              'CloseRequestFcn',@onFigureClose);  % 点击 X 时调用 / Close callback


% 左侧动画子图
subplot(1,2,1);
hold on; grid on; axis equal;
xlabel('位移 (m)'); ylabel('高度 (m)');
title('橡皮 Stick-Slip 振动动画');
xlim([v_driver*time(1)-margin, v_driver*time(1)+margin]);
ylim([-0.02, 0.04]);
h_floor = plot([v_driver*time(1)-margin, v_driver*time(1)+margin], [-0.005, -0.005], 'k', 'LineWidth',2);
driver_plot = plot(0, 0.005, 'bo', 'MarkerSize',8, 'MarkerFaceColor','b');
rubber = rectangle('Position',[x_save(1), -0.005, 0.005, 0.003], 'FaceColor',[0.85 0.33 0.1]);
[spring_x, spring_y] = getSpringPoints(v_driver*time(1), x_save(1)+0.0025, 0.005, 4, 0.002, 50);
spring_line = plot(spring_x, spring_y, 'm', 'LineWidth',2);


% 固定显示框：显示弹簧力、摩擦力、驱动力数值
spring_text = annotation('textbox',[0.05, 0.85, 0.15, 0.05], 'String','弹簧力: 0 N', 'EdgeColor','none','FontSize',10,'Color','g');
friction_text = annotation('textbox',[0.05, 0.80, 0.15, 0.05], 'String','摩擦力: 0 N', 'EdgeColor','none','FontSize',10,'Color','r');
Fs_text = annotation('textbox',[0.05, 0.75, 0.15, 0.05], 'String', sprintf('最大静摩擦力: %.2f N', Fs), 'EdgeColor','none','FontSize',10);
drive_text = annotation('textbox',[0.05, 0.70, 0.15, 0.05], 'String',sprintf('驱动力: %.2f N', F0), 'EdgeColor','none','FontSize',10,'Color','b');

% 动态力矢量箭头（绘制在左侧子图中）
spring_arrow = quiver(0,0,0.01,0, 'LineWidth',2,'Color','g','MaxHeadSize',2);
friction_arrow = quiver(0,0,0.01,0, 'LineWidth',2,'Color','r','MaxHeadSize',2);
fixed_drive_length = 0.01; % 固定蓝色箭头长度表示恒定驱动力
drive_arrow = quiver(0,0,fixed_drive_length,0, 'LineWidth',2,'Color','b','MaxHeadSize',2);

% 右侧子图：使用 yyaxis 展示位移与力曲线
subplot(1,2,2);
hold on; grid on;
title('橡皮块位移与力随时间变化');
xlabel('时间 (s)');
yyaxis left;
ylabel('位移 (m)');
set(gca,'YColor','r');
h_disp = plot(time, x_save, 'r', 'LineWidth',1.5);
yyaxis right;
ylabel('力 (N)');
set(gca,'YColor','b');
h_force = plot(time, F_spring_array, 'g', 'LineWidth',1.5);
% 添加恒定驱动力曲线（绿色虚线）
h_drive = plot(time, F0*ones(size(time)), 'b--', 'LineWidth',1.5);
xlim([0 t_end]);
ylim([-0.5, 2]);  % 修改右侧轴范围，根据需要调整

legend([h_disp, h_force, h_drive], {'位移', '弹簧力', '驱动力'}, 'Location', 'northeast');

%% 动画循环
for i = 1:10:N
    % —— 检查窗口是否已被关闭 ——  
    if ~ishandle(hFig)
        break;   % 窗口不存在了，跳出循环
    end
    t = time(i);
    x_driver = v_driver * t;
    F_spring = k*(x_driver - x_save(i));
    
    % 更新左侧动画子图
    subplot(1,2,1);
    xlim([x_driver-margin, x_driver+margin]);
    set(h_floor, 'XData',[x_driver-margin, x_driver+margin]);
    driver_plot.XData = x_driver;
    rubber.Position = [x_save(i), -0.005, 0.005, 0.003];
    [spring_x, spring_y] = getSpringPoints(x_driver, x_save(i)+0.0025, 0.005, 4, 0.002, 50);
    set(spring_line, 'XData', spring_x, 'YData', spring_y);
    
    % 更新动态力矢量箭头和 annotation 文本（左侧显示）\n
    spring_scale = 0.01;
    spring_arrow.XData = x_save(i) + 0.005/2;
    spring_arrow.YData = 0;
    spring_arrow.UData = spring_scale * F_spring;
    spring_arrow.VData = 0;
    spring_text.String = sprintf('弹簧力: %.2f N', F_spring);
    
    if state_save(i) == "stick"
        friction_force = -F_spring;
    else
        Fd_current = Fd_fun(v_save(i));
        friction_force = - Fd_current * sign(v_save(i));
    end
    friction_scale = 0.01;
    friction_arrow.XData = x_save(i) + 0.005/2;
    friction_arrow.YData = -0.003 - 0.007;
    friction_arrow.UData = friction_scale * friction_force;
    friction_arrow.VData = 0;
    friction_text.String = sprintf('摩擦力: %.2f N', friction_force);
    
    drive_arrow.XData = x_driver;
    drive_arrow.YData = 0.005;
    drive_arrow.UData = fixed_drive_length;
    drive_arrow.VData = 0;
    drive_text.String = sprintf('驱动力: %.2f N', F0);
    
    % 更新右侧子图\n
    subplot(1,2,2);
    yyaxis left;
    set(h_disp, 'XData', time(1:i), 'YData', x_save(1:i));
    yyaxis right;
    set(h_force, 'XData', time(1:i), 'YData', F_spring_array(1:i));
    set(h_drive, 'XData', time(1:i), 'YData', F0*ones(1,i));
    
    drawnow;
    % —— 保存为 GIF ——  
    frame = getframe(hFig);                   % 捕获当前图像帧
    im = frame2im(frame);                     % 转为图像数据
    [imind, cm] = rgb2ind(im, 256);            % 转为索引图（GIF 格式需要）
    if t <= 20                                 % 只保存 20 秒以内的动画
        if i == 1
            imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', dt*10);
        else
            imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', dt*10);
        end
    end

end

%% --- 辅助函数: 生成弹簧的点坐标 ---
function [xs, ys] = getSpringPoints(x_start, x_end, y0, N_coils, amplitude, numPoints)
    xs = linspace(x_start, x_end, numPoints);
    t = linspace(0,1,numPoints);
    ys = y0 + amplitude*sin(2*pi*N_coils*t);
end

%% —— 回调函数定义 ——  
function onFigureClose(src, ~)
    % 当点击 X 时触发
    % 直接删除 Figure，主循环里检测到无法句柄就会退出
    delete(src);
end
