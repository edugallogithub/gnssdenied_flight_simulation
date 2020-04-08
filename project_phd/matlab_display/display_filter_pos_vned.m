function display_filter_pos_vned(folder_name, varargin)

fileID_gps = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_gps_vned.txt']);
fgetl(fileID_gps);
input_gps = textscan(fileID_gps, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID_gps);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_filter_pos_vned'];

At_sec                            = input_gps{1}; % time
Av_n_mps_est(:,1)                 = input_gps{2}; % v_n_mps estimated (a posteriori)
Av_n_mps_est(:,2)                 = input_gps{3};
Av_n_mps_est(:,3)                 = input_gps{4};
Av_n_mps_est_apriori(:,1)         = input_gps{5}; % v_n_mps estimated (a priori)
Av_n_mps_est_apriori(:,2)         = input_gps{6};
Av_n_mps_est_apriori(:,3)         = input_gps{7};
Av_n_mps_truth(:,1)               = input_gps{8}; % v_n_mps truth
Av_n_mps_truth(:,2)               = input_gps{9};
Av_n_mps_truth(:,3)               = input_gps{10};
Av_n_mps_sensed(:,1)              = input_gps{11};  % v_n_mps sensed
Av_n_mps_sensed(:,2)              = input_gps{12};
Av_n_mps_sensed(:,3)              = input_gps{13};
Asigma_v_n_mps_est(:,1)           = input_gps{14}; % v_n_mps estimated error std (a posteriori)
Asigma_v_n_mps_est(:,2)           = input_gps{15};
Asigma_v_n_mps_est(:,3)           = input_gps{16};
Asigma_v_n_mps_est_apriori(:,1)   = input_gps{17}; % v_n_mps estimated error std (a priori)
Asigma_v_n_mps_est_apriori(:,2)   = input_gps{18};
Asigma_v_n_mps_est_apriori(:,3)   = input_gps{19};
Amean_v_n_mps_est(:,1)            = input_gps{20}; % v_n_mps estimated error mean (a posteriori)
Amean_v_n_mps_est(:,2)            = input_gps{21};
Amean_v_n_mps_est(:,3)            = input_gps{22};
Amean_v_n_mps_est_apriori(:,1)    = input_gps{23}; % v_n_mpsb estimated error mean (a priori)
Amean_v_n_mps_est_apriori(:,2)    = input_gps{24};
Amean_v_n_mps_est_apriori(:,3)    = input_gps{25};
clear input_gps;

fileID_pos = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_pos_vned.txt']);
if (fileID_pos >= 0)
    fgetl(fileID_pos);
    input_pos = textscan(fileID_pos, '%f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
    fclose(fileID_pos);

    Bt_sec                 = input_pos{1}; % time
    Bv_n_mps_est(:,1)      = input_pos{2}; % v_n_mps integrated
    Bv_n_mps_est(:,2)      = input_pos{3};
    Bv_n_mps_est(:,3)      = input_pos{4};
    Bv_n_mps_truth(:,1)    = input_pos{5}; % v_n_mps truth
    Bv_n_mps_truth(:,2)    = input_pos{6};
    Bv_n_mps_truth(:,3)    = input_pos{7};
    clear input_pos;

    t_sec                      = [At_sec;                        Bt_sec(2:end)];
    v_n_mps_est                = [Av_n_mps_est;                Bv_n_mps_est(2:end,:)];
    v_n_mps_truth              = [Av_n_mps_truth;              Bv_n_mps_truth(2:end,:)];
    v_n_mps_est_apriori        = Av_n_mps_est_apriori;
    sigma_v_n_mps_est          = Asigma_v_n_mps_est;
    sigma_v_n_mps_est_apriori  = Asigma_v_n_mps_est_apriori;
    mean_v_n_mps_est           = Amean_v_n_mps_est;
    mean_v_n_mps_est_apriori   = Amean_v_n_mps_est_apriori;

    if (nargin > 2)
        tmin_sec = varargin{1};
        tmax_sec = varargin{2};
    elseif (nargin == 2)
        tmin_sec = varargin{1};
        tmax_sec = t_sec(end);
    else
        tmin_sec = 0.;
        tmax_sec = t_sec(end);
    end

    I = find((t_sec >= tmin_sec) & (t_sec <= tmax_sec));
    t_sec           = t_sec(I);
    v_n_mps_est     = v_n_mps_est(I,:);
    v_n_mps_truth   = v_n_mps_truth(I,:);
        
    AI = find((At_sec >= tmin_sec) & (At_sec <= tmax_sec));
    At_sec                      = At_sec(AI);
    Av_n_mps_est                = Av_n_mps_est(AI,:);
    Av_n_mps_est_apriori        = Av_n_mps_est_apriori(AI,:);
    Av_n_mps_sensed             = Av_n_mps_sensed(AI,:);
    Av_n_mps_truth              = Av_n_mps_truth(AI,:);
    Asigma_v_n_mps_est          = Asigma_v_n_mps_est(AI,:);
    Asigma_v_n_mps_est_apriori  = Asigma_v_n_mps_est_apriori(AI,:);
    Amean_v_n_mps_est           = Amean_v_n_mps_est(AI,:);
    Amean_v_n_mps_est_apriori   = Amean_v_n_mps_est_apriori(AI,:);
    
    figure('units','normalized','position',[0 0.15 1.0 0.85]);

    ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
    l = plot(At_sec, Av_n_mps_sensed(:,1), ...
             At_sec, Av_n_mps_est_apriori(:,1), ...
             t_sec,  v_n_mps_est(:,1), ...
             t_sec,  v_n_mps_truth(:,1)); hold on;
    xlabel('t [sec]'); 
    ylabel('v_{1}^{N} [mps]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'b';
    l(2).LineStyle = '--';
    l(3).Color = 'b';
    l(4).Color = 'k';
    ax1.YColor = 'k';
    ax1.XMinorGrid = 'on';
    ax1.YMinorGrid = 'on';
    legend(l, 'sensed', 'est a priori', 'est a posteriori', 'truth');
    grid on;

    ax3 = subplot('Position',[0.05 0.38 0.43 0.27]);
    l = plot(At_sec, Av_n_mps_sensed(:,2), ...
             At_sec, Av_n_mps_est_apriori(:,2), ...
             t_sec,  v_n_mps_est(:,2), ...
             t_sec,  v_n_mps_truth(:,2)); hold on;
    xlabel('t [sec]'); 
    ylabel('v_{2}^{N} [mps]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'b';
    l(2).LineStyle = '--';
    l(3).Color = 'b';
    l(4).Color = 'k';
    ax3.YColor = 'k';
    ax3.XMinorGrid = 'on';
    ax3.YMinorGrid = 'on';
    legend(l, 'sensed', 'est a priori', 'est a posteriori', 'truth');
    grid on;

    ax5 = subplot('Position',[0.05 0.05 0.43 0.27]);
    l = plot(At_sec, Av_n_mps_sensed(:,3), ...
             At_sec, Av_n_mps_est_apriori(:,3), ...
             t_sec,  v_n_mps_est(:,3), ...
             t_sec,  v_n_mps_truth(:,3)); hold on;
    xlabel('t [sec]'); 
    ylabel('v_{3}^{N} [mps]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'b';
    l(2).LineStyle = '--';
    l(3).Color = 'b';
    l(4).Color = 'k';
    ax5.YColor = 'k';
    ax5.XMinorGrid = 'on';
    ax5.YMinorGrid = 'on';
    legend(l, 'sensed', 'est a priori', 'est a posteriori', 'truth');
    grid on;

    ax2 = subplot('Position',[0.55 0.71 0.43 0.27]);
    l = plot(At_sec, Av_n_mps_sensed(:,1) - Av_n_mps_truth(:,1), ...
             At_sec, Av_n_mps_est_apriori(:,1) - Av_n_mps_truth(:,1), ...
             At_sec,  - Asigma_v_n_mps_est_apriori(:,1), ...
             At_sec,  Asigma_v_n_mps_est_apriori(:,1), ...
             At_sec,  Amean_v_n_mps_est_apriori(:,1), ...
             t_sec,  v_n_mps_est(:,1) - v_n_mps_truth(:,1), ...
             At_sec,  - Asigma_v_n_mps_est(:,1), ...
             At_sec,  Asigma_v_n_mps_est(:,1), ...    
             At_sec,  Amean_v_n_mps_est(:,1)); hold on;
    xlabel('t [sec]'); 
    legend('sensed error', ...
           'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');
    ylabel('Error v_{1}^{N} [mps]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'k';
    l(3).Color = 'k';
    l(3).LineStyle = '--';
    l(4).Color = 'k';
    l(4).LineStyle = '--';
    l(5).Color = 'k';
    l(5).LineStyle = ':';
    l(6).Color = 'b';
    l(7).Color = 'b';
    l(7).LineStyle = '--';
    l(8).Color = 'b';
    l(8).LineStyle = '--';
    l(9).Color = 'b';
    l(9).LineStyle = ':';
    ax2.YColor = 'k';
    ax2.XMinorGrid = 'on';
    ax2.YMinorGrid = 'on';
    grid on;

    ax4 = subplot('Position',[0.55 0.38 0.43 0.27]);
    l = plot(At_sec, Av_n_mps_sensed(:,2) - Av_n_mps_truth(:,2), ...
             At_sec, Av_n_mps_est_apriori(:,2) - Av_n_mps_truth(:,2), ...
             At_sec, - Asigma_v_n_mps_est_apriori(:,2), ...
             At_sec, Asigma_v_n_mps_est_apriori(:,2), ...
             At_sec, Amean_v_n_mps_est_apriori(:,2), ...
             t_sec,  v_n_mps_est(:,2) - v_n_mps_truth(:,2), ...
             At_sec,  - Asigma_v_n_mps_est(:,2), ...
             At_sec,  Asigma_v_n_mps_est(:,2), ...
             At_sec,  Amean_v_n_mps_est(:,2)); hold on;    
    legend('sensed error', ...
           'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');
    xlabel('t [sec]'); 
    ylabel('Error v_{2}^{N} [mps]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'k';
    l(3).Color = 'k';
    l(3).LineStyle = '--';
    l(4).Color = 'k';
    l(4).LineStyle = '--';
    l(5).Color = 'k';
    l(5).LineStyle = ':';
    l(6).Color = 'b';
    l(7).Color = 'b';
    l(7).LineStyle = '--';
    l(8).Color = 'b';
    l(8).LineStyle = '--';
    l(9).Color = 'b';
    l(9).LineStyle = ':';
    ax4.YColor = 'k';
    ax4.XMinorGrid = 'on';
    ax4.YMinorGrid = 'on';
    grid on;

    ax6 = subplot('Position',[0.55 0.05 0.43 0.27]);
    l = plot(At_sec, Av_n_mps_sensed(:,3) - Av_n_mps_truth(:,3), ...
             At_sec, Av_n_mps_est_apriori(:,3) - Av_n_mps_truth(:,3), ...
             At_sec, - Asigma_v_n_mps_est_apriori(:,3), ...
             At_sec, Asigma_v_n_mps_est_apriori(:,3), ...   
             At_sec, Amean_v_n_mps_est_apriori(:,3), ...
             t_sec,  v_n_mps_est(:,3) - v_n_mps_truth(:,3), ...
             At_sec, - Asigma_v_n_mps_est(:,3), ...
             At_sec, Asigma_v_n_mps_est(:,3), ...    
             At_sec, Amean_v_n_mps_est(:,3)); hold on;    
    legend('sensed error', ...
           'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');     
    xlabel('t [sec]'); 
    ylabel('Error v_{3}^{N} [mps]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'k';
    l(3).Color = 'k';
    l(3).LineStyle = '--';
    l(4).Color = 'k';
    l(4).LineStyle = '--';
    l(5).Color = 'k';
    l(5).LineStyle = ':';
    l(6).Color = 'b';
    l(7).Color = 'b';
    l(7).LineStyle = '--';
    l(8).Color = 'b';
    l(8).LineStyle = '--';
    l(9).Color = 'b';
    l(9).LineStyle = ':';
    ax6.YColor = 'k';
    ax6.XMinorGrid = 'on';
    ax6.YMinorGrid = 'on';
    grid on;

else

    if (nargin > 2)
        tmin_sec = varargin{1};
        tmax_sec = varargin{2};
    elseif (nargin == 2)
        tmin_sec = varargin{1};
        tmax_sec = At_sec(end);
    else
        tmin_sec = 0.;
        tmax_sec = At_sec(end);
    end

    I = find((At_sec >= tmin_sec) & (At_sec <= tmax_sec));
    At_sec                          = At_sec(I);
    Av_n_mps_est                  = Av_n_mps_est(I,:);
    Av_n_mps_est_apriori          = Av_n_mps_est_apriori(I,:);
    Av_n_mps_truth                = Av_n_mps_truth(I,:);
    Av_n_mps_sensed               = Av_n_mps_sensed(I,:);
    Asigma_v_n_mps_est            = Asigma_v_n_mps_est(I,:);
    Asigma_v_n_mps_est_apriori    = Asigma_v_n_mps_est_apriori(I,:);
    Amean_v_n_mps_est             = Amean_v_n_mps_est(I,:);
    Amean_v_n_mps_est_apriori     = Amean_v_n_mps_est_apriori(I,:);
        
    figure('units','normalized','position',[0 0.15 1.0 0.85]);

    ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
    %ax1 = subplot(3,2,1);
    l = plot(At_sec, Av_n_mps_sensed(:,1), ...
             At_sec, Av_n_mps_est_apriori(:,1), ...
             At_sec, Av_n_mps_est(:,1), ...
             At_sec, Av_n_mps_truth(:,1)); hold on;
    xlabel('t [sec]'); 
    ylabel('v_{1}^{N} [mps]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'b';
    l(2).LineStyle = '--';
    l(3).Color = 'b';
    l(4).Color = 'k';
    ax1.YColor = 'k';
    ax1.XMinorGrid = 'on';
    ax1.YMinorGrid = 'on';
    legend(l, 'sensed', 'est a priori', 'est a posteriori', 'truth');
    grid on;

    ax3 = subplot('Position',[0.05 0.38 0.43 0.27]);
    %ax3 = subplot(3,2,3);
    l = plot(At_sec, Av_n_mps_sensed(:,2), ...
             At_sec, Av_n_mps_est_apriori(:,2), ...
             At_sec, Av_n_mps_est(:,2), ...
             At_sec, Av_n_mps_truth(:,2)); hold on;
    xlabel('t [sec]'); 
    ylabel('v_{2}^{N} [mps]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'b';
    l(2).LineStyle = '--';
    l(3).Color = 'b';
    l(4).Color = 'k';
    ax3.YColor = 'k';
    ax3.XMinorGrid = 'on';
    ax3.YMinorGrid = 'on';
    legend(l, 'sensed', 'est a priori', 'est a posteriori', 'truth');
    grid on;

    ax5 = subplot('Position',[0.05 0.05 0.43 0.27]);
    %ax5 = subplot(3,2,5);
    l = plot(At_sec, Av_n_mps_sensed(:,3), ...
             At_sec, Av_n_mps_est_apriori(:,3), ...
             At_sec, Av_n_mps_est(:,3), ...
             At_sec, Av_n_mps_truth(:,3)); hold on;
    xlabel('t [sec]'); 
    ylabel('v_{3}^{N} [mps]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'b';
    l(2).LineStyle = '--';
    l(3).Color = 'b';
    l(4).Color = 'k';
    ax5.YColor = 'k';
    ax5.XMinorGrid = 'on';
    ax5.YMinorGrid = 'on';
    legend(l, 'sensed', 'est a priori', 'est a posteriori', 'truth');
    grid on;

    ax2 = subplot('Position',[0.55 0.71 0.43 0.27]);
    %ax2 = subplot(3,2,2);
    l = plot(At_sec, Av_n_mps_sensed(:,1) - Av_n_mps_truth(:,1), ...
             At_sec, Av_n_mps_est_apriori(:,1) - Av_n_mps_truth(:,1), ...
             At_sec, - Asigma_v_n_mps_est_apriori(:,1), ...
             At_sec, Asigma_v_n_mps_est_apriori(:,1), ...
             At_sec, Amean_v_n_mps_est_apriori(:,1), ...
             At_sec, Av_n_mps_est(:,1) - Av_n_mps_truth(:,1), ...
             At_sec, - Asigma_v_n_mps_est(:,1), ...
             At_sec, Asigma_v_n_mps_est(:,1), ...    
             At_sec, Amean_v_n_mps_est(:,1)); hold on;
    xlabel('t [sec]'); 
    legend('sensed error', ...
           'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');
    ylabel('Error v_{1}^{N} [mps]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'k';
    l(3).Color = 'k';
    l(3).LineStyle = '--';
    l(4).Color = 'k';
    l(4).LineStyle = '--';
    l(5).Color = 'k';
    l(5).LineStyle = ':';
    l(6).Color = 'b';
    l(7).Color = 'b';
    l(7).LineStyle = '--';
    l(8).Color = 'b';
    l(8).LineStyle = '--';
    l(9).Color = 'b';
    l(9).LineStyle = ':';
    ax2.YColor = 'k';
    ax2.XMinorGrid = 'on';
    ax2.YMinorGrid = 'on';
    grid on;

    ax4 = subplot('Position',[0.55 0.38 0.43 0.27]);
    %ax4 = subplot(3,2,4);
    l = plot(At_sec, Av_n_mps_sensed(:,2) - Av_n_mps_truth(:,2), ...
             At_sec, Av_n_mps_est_apriori(:,2) - Av_n_mps_truth(:,2), ...
             At_sec, - Asigma_v_n_mps_est_apriori(:,2), ...
             At_sec, Asigma_v_n_mps_est_apriori(:,2), ...
             At_sec, Amean_v_n_mps_est_apriori(:,2), ...
             At_sec, Av_n_mps_est(:,2) - Av_n_mps_truth(:,2), ...
             At_sec, - Asigma_v_n_mps_est(:,2), ...
             At_sec, Asigma_v_n_mps_est(:,2), ...
             At_sec, Amean_v_n_mps_est(:,2)); hold on;    
    legend('sensed error', ...
           'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');
    xlabel('t [sec]'); 
    ylabel('Error v_{2}^{N} [mps]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'k';
    l(3).Color = 'k';
    l(3).LineStyle = '--';
    l(4).Color = 'k';
    l(4).LineStyle = '--';
    l(5).Color = 'k';
    l(5).LineStyle = ':';
    l(6).Color = 'b';
    l(7).Color = 'b';
    l(7).LineStyle = '--';
    l(8).Color = 'b';
    l(8).LineStyle = '--';
    l(9).Color = 'b';
    l(9).LineStyle = ':';
    ax4.YColor = 'k';
    ax4.XMinorGrid = 'on';
    ax4.YMinorGrid = 'on';
    grid on;

    ax6 = subplot('Position',[0.55 0.05 0.43 0.27]);
    %ax6 = subplot(3,2,6);
    l = plot(At_sec, Av_n_mps_sensed(:,3) - Av_n_mps_truth(:,3), ...
             At_sec, Av_n_mps_est_apriori(:,3) - Av_n_mps_truth(:,3), ...
             At_sec, - Asigma_v_n_mps_est_apriori(:,3), ...
             At_sec, Asigma_v_n_mps_est_apriori(:,3), ...   
             At_sec, Amean_v_n_mps_est_apriori(:,3), ...
             At_sec, Av_n_mps_est(:,3) - Av_n_mps_truth(:,3), ...
             At_sec, - Asigma_v_n_mps_est(:,3), ...
             At_sec, Asigma_v_n_mps_est(:,3), ...    
             At_sec, Amean_v_n_mps_est(:,3)); hold on;    
    legend('sensed error', ...
           'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');     
    xlabel('t [sec]'); 
    ylabel('Error v_{3}^{N} [mps]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'k';
    l(3).Color = 'k';
    l(3).LineStyle = '--';
    l(4).Color = 'k';
    l(4).LineStyle = '--';
    l(5).Color = 'k';
    l(5).LineStyle = ':';
    l(6).Color = 'b';
    l(7).Color = 'b';
    l(7).LineStyle = '--';
    l(8).Color = 'b';
    l(8).LineStyle = '--';
    l(9).Color = 'b';
    l(9).LineStyle = ':';
    ax6.YColor = 'k';
    ax6.XMinorGrid = 'on';
    ax6.YMinorGrid = 'on';
    grid on;
        
end

%savefig([plot_name '.fig']);

%Image = getframe(gcf);
%imwrite(Image.cdata, [plot_name '.jpg']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    






