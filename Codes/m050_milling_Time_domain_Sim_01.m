clc;
clear;
close all;

% milling_simulation([8000], 2e-3, 1e-4, 4, 0.2)
milling_simulation([4520], 2e-3, 1e-4, 4, 1)


function milling_simulation(rpm, cutDepth, feedPerTooth, numTeeth, simTime)
    close all;
    clearvars -global;
    clc;

    global PARAMS;
    
    %% 固定结构参数（封装前版本保持不变）
    % X方向参数
    PARAMS.k_x    = 5.6e6;         % 刚度 (N/m)
    PARAMS.zeta_x = 0.035;     
    PARAMS.wn_x   = 2*pi*600;  
    PARAMS.m_x    = PARAMS.k_x / (PARAMS.wn_x^2);
    PARAMS.c_x    = 2*PARAMS.zeta_x*PARAMS.m_x*PARAMS.wn_x;

    % Y方向参数
    PARAMS.k_y    = 5.6e6;
    PARAMS.zeta_y = 0.035;
    PARAMS.wn_y   = 2*pi*660;
    PARAMS.m_y    = PARAMS.k_y / (PARAMS.wn_y^2);
    PARAMS.c_y    = 2*PARAMS.zeta_y*PARAMS.m_y*PARAMS.wn_y;

    % 铣削过程固定参数
    PARAMS.Kt     = 600e6;        % 切向切削系数 (Pa)
    PARAMS.Kr     = 0.07;         % 径向比例系数
    PARAMS.feed   = feedPerTooth; % 每齿进给量 (m)
    PARAMS.phi_e  = 0;            % 切入角 (rad)
    PARAMS.phi_x  = pi;           % 切出角 (rad)，半浸入
    PARAMS.Z      = numTeeth;      % 刀齿数
    toolBrokenLimit = 100e-3; % 5e-3 m 断刀
    %% 支持向量化：确定分段数量
    segCount = max(length(rpm), length(cutDepth));
    if isscalar(rpm)
        rpm = repmat(rpm, segCount, 1);
    end
    if isscalar(cutDepth)
        cutDepth = repmat(cutDepth, segCount, 1);
    end
    segTime = simTime / segCount; % 每段仿真时间

    overallSols = {};  % 用于存储所有段的解
    overallTime = 0;   % 总体时间起点
    histFun = @(t) [0; 0; 0; 0];  % 初始历史函数
    toolBroken = false; % 工具断刀标志

    %% 外层循环：按段更改参数
    for seg = 1:segCount
        % 更新当前段参数
        PARAMS.N = rpm(seg);
        PARAMS.a = cutDepth(seg);
        % 计算当前段单齿周期
        T = 60/(PARAMS.N * PARAMS.Z);
        PARAMS.toothPeriod = T;
        
        % 当前段起止时间
        segStart = overallTime;
        segEnd   = overallTime + segTime;
        
        % 当前段 pass-by-pass 分段积分
        t0 = segStart;
        segSols = {};  % 存储当前段内各 pass 的解
        passCountEst = ceil((segEnd - segStart) / T);
        
        hWB = waitbar(0, sprintf('Segment %d/%d: Simulating...', seg, segCount));
        for passIndex = 1:passCountEst
            t1 = t0 + T;
            if t1 > segEnd
                t1 = segEnd;
            end
            
            fractionDone = min(passIndex/passCountEst, 1);
            waitbar(fractionDone, hWB, sprintf('Segment %d, Pass %d/%d: [%.4f, %.4f]', seg, passIndex, passCountEst, t0, t1));
            drawnow;
            
            solPiece = dde23(@chatterDDE_global, T, histFun, [t0, t1], ddeset('RelTol',1e-6, 'AbsTol',1e-8));
            segSols{end+1} = solPiece;
            fprintf('Segment %d, Pass %d completed at t=%.4f\n', seg, passIndex, solPiece.x(end));
            
            % 检查 x 或 y 振动位移是否超过 5e-3 m
            if max(abs(solPiece.y(1,:))) > toolBrokenLimit || max(abs(solPiece.y(3,:))) > toolBrokenLimit
                disp('Tool broken');
                toolBroken = true;
                % 保留当前 pass 的结果，然后退出 pass 循环
                t0 = solPiece.x(end);
                break;
            end
            
            t0 = solPiece.x(end);
            if t0 >= segEnd
                break;
            end
            
            % 更新局部历史函数：插值当前段解
            tStart_local = solPiece.x(1);
            tEnd_local = solPiece.x(end);
            tHist  = linspace(tStart_local, tEnd_local, 200);
            yHist  = deval(solPiece, tHist);
            histFun = @(tt) localHistoryFun_global(tt, tStart_local, tEnd_local, tHist, yHist);
        end
        close(hWB);
        
        overallSols = [overallSols, segSols]; %#ok<AGROW>
        overallTime = segEnd;
        
        if toolBroken
            % 若检测到断刀，则退出外层循环
            break;
        end
    end

    %% 合并所有分段解
    [tAll, yAll] = mergeSolutions_global(overallSols);

    %% 绘图
    % (A) 位移与 FFT 分析
    figure('Name','(A) Displacement & FFT','Color','w');
    subplot(3,1,1);
      plot(tAll, yAll(1,:), 'b-', 'LineWidth', 1.2);
      xlabel('Time (s)'); ylabel('x(t)');
      title('x-displacement');
    subplot(3,1,2);
      plot(tAll, yAll(3,:), 'r-', 'LineWidth', 1.2);
      xlabel('Time (s)'); ylabel('y(t)');
      title('y-displacement');
    subplot(3,1,3);
      xVals = yAll(1,:) - mean(yAll(1,:));
      dt = mean(diff(tAll));
      Fs = 1/dt;
      L = length(xVals);
      Xfft = fft(xVals);
      faxis = Fs*(0:(L-1))/L;
      Amp = abs(Xfft)/L;
      plot(faxis, Amp, 'k-', 'LineWidth', 1.2);
      xlabel('Frequency (Hz)'); ylabel('Amplitude');
      xlim([0, 2000]); title('FFT of x(t)'); grid on;

    % (B) 速度图
    figure('Name','(B) Velocity','Color','w');
    subplot(2,1,1);
      plot(tAll, yAll(2,:), 'b-', 'LineWidth', 1.2);
      xlabel('Time (s)'); ylabel('dx/dt');
      title('Velocity in X');
    subplot(2,1,2);
      plot(tAll, yAll(4,:), 'r-', 'LineWidth', 1.2);
      xlabel('Time (s)'); ylabel('dy/dt');
      title('Velocity in Y');

    % (C) Poincaré 图：在每个齿周期取样
    figure('Name','(C) Poincaré','Color','w');
    tPoincare = 0:T:tAll(end);
    xP = zeros(size(tPoincare));
    dxP = zeros(size(tPoincare));
    for i = 1:length(tPoincare)
        if tPoincare(i) <= tAll(end)
            Yp = ddevalPiecewise_global(overallSols, tPoincare(i));
            xP(i) = Yp(1);
            dxP(i) = Yp(2);
        end
    end
    plot(xP, dxP, 'ko', 'MarkerFaceColor', 'b', 'MarkerSize', 4);
    xlabel('x'); ylabel('dx/dt');
    title('Poincaré Map at multiples of tooth period'); grid on;
    
    % 在显示图形后调用封装好的导出函数
    exportVibrationData(tAll, yAll, rpm, cutDepth, feedPerTooth, numTeeth, simTime);
    
    if toolBroken
        disp('Simulation ended due to tool broken.');
    else
        disp('Simulation completed.');
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LOCAL FUNCTIONS (均引用全局变量 PARAMS)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dYdt = chatterDDE_global(t, Y, Z_lag)
    global PARAMS

    x1 = Y(1);  x2 = Y(2);
    y1 = Y(3);  y2 = Y(4);
    x1_tau = Z_lag(1);
    y1_tau = Z_lag(3);

    % 从 PARAMS 中提取参数
    m_x  = PARAMS.m_x;   c_x = PARAMS.c_x;   k_x = PARAMS.k_x;
    m_y  = PARAMS.m_y;   c_y = PARAMS.c_y;   k_y = PARAMS.k_y;
    Kt   = PARAMS.Kt;    Kr  = PARAMS.Kr;    a   = PARAMS.a;
    feed = PARAMS.feed;  Z   = PARAMS.Z;     N   = PARAMS.N;
    phi_e = PARAMS.phi_e; phi_x = PARAMS.phi_x;

    Fx_total = 0; 
    Fy_total = 0;
    for j = 0:(Z-1)
        th = (2*pi*N/60)*t + j*(2*pi/Z);
        th_mod = mod(th, 2*pi);

        if (th_mod >= phi_e) && (th_mod <= phi_x)
            hs    = feed*abs(sin(th_mod));
            u_now = - x1*sin(th_mod) - y1*cos(th_mod);
            u_tau = - x1_tau*sin(th_mod) - y1_tau*cos(th_mod);
            hd    = hs + (u_now - u_tau);
            if hd > 0
                Ft   = Kt*hd*a;
                Fr   = Kt*Kr*hd*a;
                Fx_j = -sin(th_mod)*Fr - cos(th_mod)*Ft;
                Fy_j = -cos(th_mod)*Fr + sin(th_mod)*Ft;
                Fx_total = Fx_total + Fx_j;
                Fy_total = Fy_total + Fy_j;
            end
        end
    end

    dx1dt = x2;
    dx2dt = (1/m_x)*(Fx_total - c_x*x2 - k_x*x1);
    dy1dt = y2;
    dy2dt = (1/m_y)*(Fy_total - c_y*y2 - k_y*y1);

    dYdt = [dx1dt; dx2dt; dy1dt; dy2dt];
end

function v = localHistoryFun_global(tt, tStart, tEnd, tHist, yHist)
    if (tt >= tStart) && (tt <= tEnd)
        v = interp1(tHist', yHist', tt, 'linear','extrap')';
    else
        v = yHist(:,1);
    end
end

function [tAll, yAll] = mergeSolutions_global(piecewiseSols)
    tAll = [];
    yAll = [];
    for i = 1:length(piecewiseSols)
        tseg = piecewiseSols{i}.x;
        yseg = piecewiseSols{i}.y;
        tAll = [tAll, tseg];
        yAll = [yAll, yseg];
    end
    [tAll, idxU] = unique(tAll);
    yAll = yAll(:, idxU);
end

function Yp = ddevalPiecewise_global(piecewiseSols, tquery)
    for i = 1:length(piecewiseSols)
        tstart = piecewiseSols{i}.x(1);
        tend   = piecewiseSols{i}.x(end);
        if (tquery >= tstart) && (tquery <= tend)
            Yp = deval(piecewiseSols{i}, tquery);
            return;
        end
    end
    Yp = deval(piecewiseSols{end}, piecewiseSols{end}.x(end));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 封装导出振动数据的函数
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function exportVibrationData(tAll, yAll, rpm, cutDepth, feedPerTooth, numTeeth, simTime)
    % 根据输入参数格式化生成文件名
    if isscalar(rpm)
        rpmStr = num2str(rpm);
    else
        rpmStr = strjoin(arrayfun(@num2str, rpm, 'UniformOutput', false), '-');
    end
    apStr  = regexprep(regexprep(sprintf('%.0e', cutDepth), 'e\+?', 'e'), 'e(-?)0+', 'e$1');
    fzStr  = regexprep(regexprep(sprintf('%.0e', feedPerTooth), 'e\+?', 'e'), 'e(-?)0+', 'e$1');
    tStr   = regexprep(regexprep(sprintf('%.0e', simTime), 'e\+?', 'e'), 'e(-?)0+', 'e$1');
    zStr   = num2str(numTeeth);

    fileName = sprintf('n%s_ap%s_fz%s_z%s_t%s.txt', rpmStr, apStr, fzStr, zStr, tStr);
    
    % 生成 3 列数据矩阵：时间、x位移、y位移
    data = [tAll(:), yAll(1,:)', yAll(3,:)'];
    
    % 导出为 txt 文件，使用制表符分隔
    writematrix(data, fileName, 'Delimiter', '\t');
    fprintf('Data saved to %s\n', fileName);
end




