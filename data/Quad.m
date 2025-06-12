clear; clc; close all;

% figure(n)
% print('-clipboard', '-dbitmap')  // ubuntu figure 복사 

start_time = 0; % 시작 시간
end_time = 5;   % 끝 시간

pos_plot_flag = true;
input_plot_flag = false;
Body_state_plot_flag =true;
opt_GRF_flag = true;
Body_pos_3D_plot_flag = false;

x_pos_lb = -0.5; x_pos_ub = 0.5;
y_pos_lb = -0.5; y_pos_ub = 0.5;
z_pos_lb = 0; z_pos_ub = 0.5;

x_vel_lb = -2; x_vel_ub = 2;
y_vel_lb = -2; y_vel_ub = 2;
z_vel_lb = -2; z_vel_ub = 2;




tau_lb = -40; tau_ub = 40;

xyz_label = {'$x\ \mathrm{(m)}$', '$y\ \mathrm{(m)}$', '$z\ \mathrm{(m)}$'};
opt_GRF_label = {'$F_x\ \mathrm{(N)}$', '$F_y\ \mathrm{(N)}$', '$F_z\ \mathrm{(N)}$'};
rpy_label = {'$\alpha\ \mathrm{(rad)}$', '$\beta\ \mathrm{(rad)}$', '$\gamma\ \mathrm{(rad)}$'};
xyzdot_label = {'$\dot{x}\ \mathrm{(m/s)}$', '$y\ \mathrm{(m/s)}$', '$z\ \mathrm{(m/s)}$'};
rpydot_label = {'$\alpha\ \mathrm{(rad/s)}$', '$\beta\ \mathrm{(rad/s)}$', '$\gamma\ \mathrm{(rad/s)}$'};


xyz_lb = {-0.2, -0.2, 0};   xyz_ub = {0.2, 0.2, 0.5};
rpy_lb = {-0.1, -0.1, -0.1};   rpy_ub = {0.1, 0.1, 0.1};
xyzdot_lb = {-0.3, -0.3, -0.3};   xyzdot_ub = {0.3, 0.3, 0.3};
rpydot_lb = {-0.3, -0.3, -0.3};   rpydot_ub = {0.3, 0.3, 0.3};

filename{1} = '/home/jinsong/Desktop/mujoco_cpp/mujoco-3.1.6/myproject/Quad_v2/Quad_MPC_nospine/data/data_FL.csv';
filename{2} = '/home/jinsong/Desktop/mujoco_cpp/mujoco-3.1.6/myproject/Quad_v2/Quad_MPC_nospine/data/data_FR.csv';
filename{3} = '/home/jinsong/Desktop/mujoco_cpp/mujoco-3.1.6/myproject/Quad_v2/Quad_MPC_nospine/data/data_RL.csv';
filename{4} = '/home/jinsong/Desktop/mujoco_cpp/mujoco-3.1.6/myproject/Quad_v2/Quad_MPC_nospine/data/data_RR.csv';
filename_Body = '/home/jinsong/Desktop/mujoco_cpp/mujoco-3.1.6/myproject/Quad_v2/Quad_MPC_nospine/data/data_Body.csv';

    
for i = 1:1:4
    Arr_Leg{i} = table2array(readtable(filename{i}));
end


t           = Arr_Leg{1}(:,1);

for i = 1:1:4
    x_pos_ref{i} = Arr_Leg{i}(:,2);
    y_pos_ref{i} = Arr_Leg{i}(:,3);
    z_pos_ref{i} = Arr_Leg{i}(:,4);
    x_pos{i} = Arr_Leg{i}(:,5);
    y_pos{i} = Arr_Leg{i}(:,6);
    z_pos{i} = Arr_Leg{i}(:,7);
    opt_ux{i} = Arr_Leg{i}(:,8);
    opt_uy{i} = Arr_Leg{i}(:,9);
    opt_uz{i} = Arr_Leg{i}(:,10);
    
end


Arr_Body = table2array(readtable(filename_Body));

% [m1,n1] = size(Arr_FL);
% [m2,n2] = size(Arr_Body);

% 
%Body state estimation

x0 = Arr_Body(:,1:12); 
x_ref = Arr_Body(:,13:24);




%%%%%%%%%%%%%%%% Data Ploting %%%%%%%%%%%%%%%%%%%%%%%%%
%time cutting
% 시작 시간과 끝 시간 설정

% t 벡터를 기준으로 시작 시간과 끝 시간에 해당하는 인덱스 찾기
s_idx = find(t >= start_time, 1);
e_idx = find(t >= end_time, 1);
t = t(s_idx:e_idx);


%Data ploting 관련 Parameter
lw =1.2;   %Line Width
LW = 1.7;
Title_size = 18; %Title Fonte Size
Axis_size = 12; %Axis Fonte Size
legend_size =8 ; % Legend Fonte Size


n = 1;

if pos_plot_flag == true
    
        figure(n); n = n+1;       
        
       
        for i = 1:1:4
        subplot(2,2,i);
        plot(t(s_idx:e_idx),x_pos_ref{i}(s_idx:e_idx),'black--','LineWidth', LW);
        hold on
        plot(t(s_idx:e_idx),x_pos{i}(s_idx:e_idx),'r-','LineWidth',lw);
        grid on;
        if i == 2
            legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',10,'Interpreter', 'latex')
        end
        xlabel('Time (seconds)','FontSize', Axis_size);
        ylabel('Position (m)','FontSize', Axis_size);
        ylim([x_pos_lb x_pos_ub]);
        end

        sgtitle("Position (x direction)",'FontName','times new roman', 'Fontsize', Title_size);
           
        
        figure(n); n = n+1; 
        for i = 1:1:4
        subplot(2,2,i);
        plot(t(s_idx:e_idx),y_pos_ref{i}(s_idx:e_idx),'black--','LineWidth', LW);
        hold on
        plot(t(s_idx:e_idx),y_pos{i}(s_idx:e_idx),'r-','LineWidth',lw);
        grid on;
        if i == 2
            legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',10,'Interpreter', 'latex')
        end
        xlabel('Time (seconds)','FontSize', Axis_size);
        ylabel('Position (m)','FontSize', Axis_size);
        ylim([y_pos_lb y_pos_ub]);
        
        end

        sgtitle("Position (y direction)",'FontName','Times New Roman', 'Fontsize', Title_size);
        

        figure(n); n = n+1; 
        for i = 1:1:4
        subplot(2,2,i);
        plot(t(s_idx:e_idx),z_pos_ref{i}(s_idx:e_idx),'black--','LineWidth', LW);
        hold on
        plot(t(s_idx:e_idx),z_pos{i}(s_idx:e_idx),'r-','LineWidth',lw);
        grid on;
        if i == 2
            legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',10,'Interpreter', 'latex')
        end
        xlabel('Time (seconds)','FontSize', Axis_size);
        ylabel('Position (m)','FontSize', Axis_size);
        ylim([z_pos_lb z_pos_ub]);
        end
        
        sgtitle("Position (z direction)",'FontName','Times New Roman', 'Fontsize', Title_size);
end

if opt_GRF_flag == true
    figure(n); n = n+1;

    % Fx
    subplot(3,1,1);
    for i = 1:4
        plot(t(s_idx:e_idx), opt_ux{i}(s_idx:e_idx), 'LineWidth', lw); hold on;
    end
    ylabel(opt_GRF_label{1}, 'FontSize', Axis_size, 'Interpreter', 'latex');
    legend('FL', 'FR', 'RL', 'RR', 'FontName', 'Times New Roman', 'Location', 'northeast', 'FontSize', 10, 'Interpreter', 'latex');
    grid on;

    % Fy
    subplot(3,1,2);
    for i = 1:4
        plot(t(s_idx:e_idx), opt_uy{i}(s_idx:e_idx), 'LineWidth', lw); hold on;
    end
    ylabel(opt_GRF_label{2}, 'FontSize', Axis_size, 'Interpreter', 'latex');
    grid on;

    % Fz
    subplot(3,1,3);
    for i = 1:4
        plot(t(s_idx:e_idx), opt_uz{i}(s_idx:e_idx), 'LineWidth', lw); hold on;
    end
    ylabel(opt_GRF_label{3}, 'FontSize', Axis_size, 'Interpreter', 'latex');
    xlabel('Time (seconds)', 'FontSize', Axis_size);
    grid on;

    sgtitle("Optimal GRF", 'FontName', 'Times New Roman', 'Fontsize', Title_size);
end


if Body_state_plot_flag == true
        
        figure(n); n = n+1; 
            for i = 1:1:3
                subplot(3,1,i);
                plot(t(s_idx:e_idx),x_ref(s_idx:e_idx, i),'LineWidth', lw);
                hold on;
                plot(t(s_idx:e_idx),x0(s_idx:e_idx, i),'LineWidth', lw);
                ylabel(rpy_label{i},'FontSize', Axis_size, 'Interpreter', 'latex');
                ylim([rpy_lb{i} rpy_ub{i}]);
                legend('Ref','Actual','FontName','Times New Roman','location','northeast','FontSize',10,'Interpreter', 'latex')
                grid on;
            end
                sgtitle("Body Angle ",'FontName','Times New Roman', 'Fontsize', Title_size);

        figure(n); n = n+1; 
        for i = 1:1:3
            subplot(3,1,i);
            plot(t(s_idx:e_idx),x_ref(s_idx:e_idx, i+3),'LineWidth', lw);
            hold on;
            plot(t(s_idx:e_idx),x0(s_idx:e_idx, i+3),'LineWidth', lw);
            ylabel(xyz_label{i},'FontSize', Axis_size, 'Interpreter', 'latex');
            ylim([xyz_lb{i} xyz_ub{i}]);
            legend('Ref','Actual','FontName','Times New Roman','location','northeast','FontSize',10,'Interpreter', 'latex')
            grid on;
        end
            sgtitle("Body Position",'FontName','Times New Roman', 'Fontsize', Title_size);

        figure(n); n = n+1; 
        for i = 1:1:3
            subplot(3,1,i);
            plot(t(s_idx:e_idx),x_ref(s_idx:e_idx, i+6),'LineWidth', lw);
            hold on;
            plot(t(s_idx:e_idx),x0(s_idx:e_idx, i+6),'LineWidth', lw);
            ylabel(rpydot_label{i},'FontSize', Axis_size, 'Interpreter', 'latex');
            ylim([rpydot_lb{i} rpydot_ub{i}]);
            legend('Ref','Actual','FontName','Times New Roman','location','northeast','FontSize',10,'Interpreter', 'latex')
            grid on;
        end
            sgtitle("Body Angular Velocity",'FontName','Times New Roman', 'Fontsize', Title_size);

        figure(n); n = n+1; 
        for i = 1:1:3
            subplot(3,1,i);
            plot(t(s_idx:e_idx),x_ref(s_idx:e_idx, i+9),'LineWidth', lw);
            hold on;
            plot(t(s_idx:e_idx),x0(s_idx:e_idx, i+9),'LineWidth', lw);
            ylabel(xyzdot_label{i},'FontSize', Axis_size, 'Interpreter', 'latex');
            ylim([xyzdot_lb{i} xyzdot_ub{i}]);
            legend('Ref','Actual','FontName','Times New Roman','location','northeast','FontSize',10,'Interpreter', 'latex')
            grid on;
        end
            sgtitle("Body Linear Velocity",'FontName','Times New Roman', 'Fontsize', Title_size);

end

if Body_pos_3D_plot_flag == true

    figure(n); n = n+1; 
    plot3( x0(:,4), x0(:,5), x0(:,6), 'LineWidth', 1.5 );
    grid on;
    hold on;
    plot3(x_ref(:,4), x_ref(:,5), x_ref(:,6), 'LineWidth', 1.5 );
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Body Position');

end

setFigurePositions(4);

