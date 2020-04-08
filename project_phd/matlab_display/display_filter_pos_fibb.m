function display_filter_pos_fibb(folder_name, varargin)

fileID_gps = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_gps_fibb.txt']);
fgetl(fileID_gps);
input_gps = textscan(fileID_gps, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID_gps);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_filter_pos_vned'];

At_sec                               = input_gps{1}; % time
Af_ibb_mps2_est(:,1)                 = input_gps{2}; % f_ibb_mps2 estimated (a posteriori)
Af_ibb_mps2_est(:,2)                 = input_gps{3};
Af_ibb_mps2_est(:,3)                 = input_gps{4};
Af_ibb_mps2_est_apriori(:,1)         = input_gps{5}; % f_ibb_mps2 estimated (a priori)
Af_ibb_mps2_est_apriori(:,2)         = input_gps{6};
Af_ibb_mps2_est_apriori(:,3)         = input_gps{7};
Af_ibb_mps2_truth(:,1)               = input_gps{8}; % f_ibb_mps2 truth
Af_ibb_mps2_truth(:,2)               = input_gps{9};
Af_ibb_mps2_truth(:,3)               = input_gps{10};
Af_ibb_mps2_sensed(:,1)              = input_gps{11}; % f_ibb_mps2 sensed
Af_ibb_mps2_sensed(:,2)              = input_gps{12};
Af_ibb_mps2_sensed(:,3)              = input_gps{13};
Asigma_f_ibb_mps2_est(:,1)           = input_gps{14}; % f_ibb_mps2 estimated error std (a posteriori)
Asigma_f_ibb_mps2_est(:,2)           = input_gps{15};
Asigma_f_ibb_mps2_est(:,3)           = input_gps{16};
Asigma_f_ibb_mps2_est_apriori(:,1)   = input_gps{17}; % f_ibb_mps2 estimated error std (a priori)
Asigma_f_ibb_mps2_est_apriori(:,2)   = input_gps{18};
Asigma_f_ibb_mps2_est_apriori(:,3)   = input_gps{19};
Amean_f_ibb_mps2_est(:,1)            = input_gps{20}; % f_ibb_mps2 estimated error mean (a posteriori)
Amean_f_ibb_mps2_est(:,2)            = input_gps{21};
Amean_f_ibb_mps2_est(:,3)            = input_gps{22};
Amean_f_ibb_mps2_est_apriori(:,1)    = input_gps{23}; % f_ibb_mps2b estimated error mean (a priori)
Amean_f_ibb_mps2_est_apriori(:,2)    = input_gps{24};
Amean_f_ibb_mps2_est_apriori(:,3)    = input_gps{25};
clear input_gps;

fileID_pos = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_pos_fibb.txt']);
if (fileID_pos >= 0)
    fgetl(fileID_pos);
    input_pos = textscan(fileID_pos, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
    fclose(fileID_pos);

    Bt_sec                               = input_pos{1}; % time
    Bf_ibb_mps2_est(:,1)                 = input_pos{2}; % f_ibb_mps2 estimated (a posteriori)
    Bf_ibb_mps2_est(:,2)                 = input_pos{3};
    Bf_ibb_mps2_est(:,3)                 = input_pos{4};
    Bf_ibb_mps2_est_apriori(:,1)         = input_pos{5}; % f_ibb_mps2 estimated (a priori)
    Bf_ibb_mps2_est_apriori(:,2)         = input_pos{6};
    Bf_ibb_mps2_est_apriori(:,3)         = input_pos{7};
    Bf_ibb_mps2_truth(:,1)               = input_pos{8}; % f_ibb_mps2 truth
    Bf_ibb_mps2_truth(:,2)               = input_pos{9};
    Bf_ibb_mps2_truth(:,3)               = input_pos{10};
    Bf_ibb_mps2_sensed(:,1)              = input_pos{11}; % f_ibb_mps2 sensed
    Bf_ibb_mps2_sensed(:,2)              = input_pos{12};
    Bf_ibb_mps2_sensed(:,3)              = input_pos{13};
    Bsigma_f_ibb_mps2_est(:,1)           = input_pos{14}; % f_ibb_mps2 estimated error std (a posteriori)
    Bsigma_f_ibb_mps2_est(:,2)           = input_pos{15};
    Bsigma_f_ibb_mps2_est(:,3)           = input_pos{16};
    Bsigma_f_ibb_mps2_est_apriori(:,1)   = input_pos{17}; % f_ibb_mps2 estimated error std (a priori)
    Bsigma_f_ibb_mps2_est_apriori(:,2)   = input_pos{18};
    Bsigma_f_ibb_mps2_est_apriori(:,3)   = input_pos{19};
    Bmean_f_ibb_mps2_est(:,1)            = input_pos{20}; % f_ibb_mps2 estimated error mean (a posteriori)
    Bmean_f_ibb_mps2_est(:,2)            = input_pos{21};
    Bmean_f_ibb_mps2_est(:,3)            = input_pos{22};
    Bmean_f_ibb_mps2_est_apriori(:,1)    = input_pos{23}; % f_ibb_mps2b estimated error mean (a priori)
    Bmean_f_ibb_mps2_est_apriori(:,2)    = input_pos{24};
    Bmean_f_ibb_mps2_est_apriori(:,3)    = input_pos{25};
    clear input_pos;

    t_sec                         = [At_sec;                         Bt_sec(2:end)];
    f_ibb_mps2_est                = [Af_ibb_mps2_est;                Bf_ibb_mps2_est(2:end,:)];               clear Af_ibb_mps2_est;                 clear Bf_ibb_mps2_est;
    f_ibb_mps2_est_apriori        = [Af_ibb_mps2_est_apriori;        Bf_ibb_mps2_est_apriori(2:end,:)];       clear Af_ibb_mps2_est_apriori;         clear Bf_ibb_mps2_est_apriori;
    f_ibb_mps2_truth              = [Af_ibb_mps2_truth;              Bf_ibb_mps2_truth(2:end,:)];             clear Af_ibb_mps2_truth;               clear Bf_ibb_mps2_truth;
    f_ibb_mps2_sensed             = [Af_ibb_mps2_sensed;             Bf_ibb_mps2_sensed(2:end,:)];            clear Af_ibb_mps2_sensed;              clear Bf_ibb_mps2_sensed;
    sigma_f_ibb_mps2_est          = [Asigma_f_ibb_mps2_est;          Bsigma_f_ibb_mps2_est(2:end,:)];         clear Asigma_f_ibb_mps2_est;           clear Bsigma_f_ibb_mps2_est;
    sigma_f_ibb_mps2_est_apriori  = [Asigma_f_ibb_mps2_est_apriori;  Bsigma_f_ibb_mps2_est_apriori(2:end,:)]; clear Asigma_f_ibb_mps2_est_apriori;   clear Bsigma_f_ibb_mps2_est_apriori;
    mean_f_ibb_mps2_est           = [Amean_f_ibb_mps2_est;           Bmean_f_ibb_mps2_est(2:end,:)];          clear Amean_f_ibb_mps2_est;            clear Bmean_f_ibb_mps2_est;
    mean_f_ibb_mps2_est_apriori   = [Amean_f_ibb_mps2_est_apriori;   Bmean_f_ibb_mps2_est_apriori(2:end,:)];  clear Amean_f_ibb_mps2_est_apriori;    clear Bmean_f_ibb_mps2_est_apriori;

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
    t_sec                           = t_sec(I);
    f_ibb_mps2_est                  = f_ibb_mps2_est(I,:);
    f_ibb_mps2_est_apriori          = f_ibb_mps2_est_apriori(I,:);
    f_ibb_mps2_truth                = f_ibb_mps2_truth(I,:);
    f_ibb_mps2_sensed               = f_ibb_mps2_sensed(I,:);
    sigma_f_ibb_mps2_est            = sigma_f_ibb_mps2_est(I,:);
    sigma_f_ibb_mps2_est_apriori    = sigma_f_ibb_mps2_est_apriori(I,:);
    mean_f_ibb_mps2_est             = mean_f_ibb_mps2_est(I,:);
    mean_f_ibb_mps2_est_apriori     = mean_f_ibb_mps2_est_apriori(I,:);

    figure('units','normalized','position',[0 0.15 1.0 0.85]);

    ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
    %ax1 = subplot(3,2,1);
    l = plot(t_sec, f_ibb_mps2_sensed(:,1), ...
             t_sec, f_ibb_mps2_est_apriori(:,1), ...
             t_sec, f_ibb_mps2_est(:,1), ...
             t_sec, f_ibb_mps2_truth(:,1)); hold on;
    xlabel('t [sec]'); 
    ylabel('f_{IB,1}^{B} [mps2]');
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
    l = plot(t_sec, f_ibb_mps2_sensed(:,2), ...
             t_sec, f_ibb_mps2_est_apriori(:,2), ...
             t_sec, f_ibb_mps2_est(:,2), ...
             t_sec, f_ibb_mps2_truth(:,2)); hold on;
    xlabel('t [sec]'); 
    ylabel('f_{IB,2}^{B} [mps2]');
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
    l = plot(t_sec, f_ibb_mps2_sensed(:,3), ...
             t_sec, f_ibb_mps2_est_apriori(:,3), ...
             t_sec, f_ibb_mps2_est(:,3), ...
             t_sec, f_ibb_mps2_truth(:,3)); hold on;
    xlabel('t [sec]'); 
    ylabel('f_{IB,3}^{B} [mps2]');
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
    l = plot(t_sec, f_ibb_mps2_sensed(:,1) - f_ibb_mps2_truth(:,1), ...
             t_sec, f_ibb_mps2_est_apriori(:,1) - f_ibb_mps2_truth(:,1), ...
             t_sec, - sigma_f_ibb_mps2_est_apriori(:,1), ...
             t_sec, sigma_f_ibb_mps2_est_apriori(:,1), ...
             t_sec, mean_f_ibb_mps2_est_apriori(:,1), ...
             t_sec, f_ibb_mps2_est(:,1) - f_ibb_mps2_truth(:,1), ...
             t_sec, - sigma_f_ibb_mps2_est(:,1), ...
             t_sec, sigma_f_ibb_mps2_est(:,1), ...    
             t_sec, mean_f_ibb_mps2_est(:,1)); hold on;
    xlabel('t [sec]'); 
    legend('sensed error', ...
           'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');
    ylabel('Error f_{IB,1}^{B} [mps2]');
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
    l = plot(t_sec, f_ibb_mps2_sensed(:,2) - f_ibb_mps2_truth(:,2), ...
             t_sec,  f_ibb_mps2_est_apriori(:,2) - f_ibb_mps2_truth(:,2), ...
             t_sec,  - sigma_f_ibb_mps2_est_apriori(:,2), ...
             t_sec,  sigma_f_ibb_mps2_est_apriori(:,2), ...
             t_sec,  mean_f_ibb_mps2_est_apriori(:,2), ...
             t_sec,  f_ibb_mps2_est(:,2) - f_ibb_mps2_truth(:,2), ...
             t_sec,  - sigma_f_ibb_mps2_est(:,2), ...
             t_sec,  sigma_f_ibb_mps2_est(:,2), ...
             t_sec,  mean_f_ibb_mps2_est(:,2)); hold on;    
    legend('sensed error', ...
           'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');
    xlabel('t [sec]'); 
    ylabel('Error f_{IB,2}^{B} [mps2]');
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
    l = plot(t_sec, f_ibb_mps2_sensed(:,3) - f_ibb_mps2_truth(:,3), ...
             t_sec,  f_ibb_mps2_est_apriori(:,3) - f_ibb_mps2_truth(:,3), ...
             t_sec,  - sigma_f_ibb_mps2_est_apriori(:,3), ...
             t_sec,  sigma_f_ibb_mps2_est_apriori(:,3), ...   
             t_sec,  mean_f_ibb_mps2_est_apriori(:,3), ...
             t_sec,  f_ibb_mps2_est(:,3) - f_ibb_mps2_truth(:,3), ...
             t_sec,  - sigma_f_ibb_mps2_est(:,3), ...
             t_sec,  sigma_f_ibb_mps2_est(:,3), ...    
             t_sec,  mean_f_ibb_mps2_est(:,3)); hold on;    
    legend('sensed error', ...
           'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');     
    xlabel('t [sec]'); 
    ylabel('Error f_{IB,3}^{B} [mps2]');
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
    At_sec                           = At_sec(I);
    Af_ibb_mps2_est                  = Af_ibb_mps2_est(I,:);
    Af_ibb_mps2_est_apriori          = Af_ibb_mps2_est_apriori(I,:);
    Af_ibb_mps2_truth                = Af_ibb_mps2_truth(I,:);
    Af_ibb_mps2_sensed               = Af_ibb_mps2_sensed(I,:);
    Asigma_f_ibb_mps2_est            = Asigma_f_ibb_mps2_est(I,:);
    Asigma_f_ibb_mps2_est_apriori    = Asigma_f_ibb_mps2_est_apriori(I,:);
    Amean_f_ibb_mps2_est             = Amean_f_ibb_mps2_est(I,:);
    Amean_f_ibb_mps2_est_apriori     = Amean_f_ibb_mps2_est_apriori(I,:);

    figure('units','normalized','position',[0 0.15 1.0 0.85]);

    ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
    %ax1 = subplot(3,2,1);
    l = plot(At_sec, Af_ibb_mps2_sensed(:,1), ...
             At_sec, Af_ibb_mps2_est_apriori(:,1), ...
             At_sec, Af_ibb_mps2_est(:,1), ...
             At_sec, Af_ibb_mps2_truth(:,1)); hold on;
    xlabel('t [sec]'); 
    ylabel('f_{IB,1}^{B} [mps2]');
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
    l = plot(At_sec, Af_ibb_mps2_sensed(:,2), ...
             At_sec, Af_ibb_mps2_est_apriori(:,2), ...
             At_sec, Af_ibb_mps2_est(:,2), ...
             At_sec, Af_ibb_mps2_truth(:,2)); hold on;
    xlabel('t [sec]'); 
    ylabel('f_{IB,2}^{B} [mps2]');
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
    l = plot(At_sec, Af_ibb_mps2_sensed(:,3), ...
             At_sec, Af_ibb_mps2_est_apriori(:,3), ...
             At_sec, Af_ibb_mps2_est(:,3), ...
             At_sec, Af_ibb_mps2_truth(:,3)); hold on;
    xlabel('t [sec]'); 
    ylabel('f_{IB,3}^{B} [mps2]');
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
    l = plot(At_sec, Af_ibb_mps2_sensed(:,1) - Af_ibb_mps2_truth(:,1), ...
             At_sec, Af_ibb_mps2_est_apriori(:,1) - Af_ibb_mps2_truth(:,1), ...
             At_sec, - Asigma_f_ibb_mps2_est_apriori(:,1), ...
             At_sec, Asigma_f_ibb_mps2_est_apriori(:,1), ...
             At_sec, Amean_f_ibb_mps2_est_apriori(:,1), ...
             At_sec, Af_ibb_mps2_est(:,1) - Af_ibb_mps2_truth(:,1), ...
             At_sec, - Asigma_f_ibb_mps2_est(:,1), ...
             At_sec, Asigma_f_ibb_mps2_est(:,1), ...    
             At_sec, Amean_f_ibb_mps2_est(:,1)); hold on;
    xlabel('t [sec]'); 
    legend('sensed error', ...
           'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');
    ylabel('Error f_{IB,1}^{B} [mps2]');
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
    l = plot(At_sec, Af_ibb_mps2_sensed(:,2) - Af_ibb_mps2_truth(:,2), ...
             At_sec,  Af_ibb_mps2_est_apriori(:,2) - Af_ibb_mps2_truth(:,2), ...
             At_sec,  - Asigma_f_ibb_mps2_est_apriori(:,2), ...
             At_sec,  Asigma_f_ibb_mps2_est_apriori(:,2), ...
             At_sec,  Amean_f_ibb_mps2_est_apriori(:,2), ...
             At_sec,  Af_ibb_mps2_est(:,2) - Af_ibb_mps2_truth(:,2), ...
             At_sec,  - Asigma_f_ibb_mps2_est(:,2), ...
             At_sec,  Asigma_f_ibb_mps2_est(:,2), ...
             At_sec,  Amean_f_ibb_mps2_est(:,2)); hold on;    
    legend('sensed error', ...
           'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');
    xlabel('t [sec]'); 
    ylabel('Error f_{IB,2}^{B} [mps2]');
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
    l = plot(At_sec, Af_ibb_mps2_sensed(:,3) - Af_ibb_mps2_truth(:,3), ...
             At_sec,  Af_ibb_mps2_est_apriori(:,3) - Af_ibb_mps2_truth(:,3), ...
             At_sec,  - Asigma_f_ibb_mps2_est_apriori(:,3), ...
             At_sec,  Asigma_f_ibb_mps2_est_apriori(:,3), ...   
             At_sec,  Amean_f_ibb_mps2_est_apriori(:,3), ...
             At_sec,  Af_ibb_mps2_est(:,3) - Af_ibb_mps2_truth(:,3), ...
             At_sec,  - Asigma_f_ibb_mps2_est(:,3), ...
             At_sec,  Asigma_f_ibb_mps2_est(:,3), ...    
             At_sec,  Amean_f_ibb_mps2_est(:,3)); hold on;    
    legend('sensed error', ...
           'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');     
    xlabel('t [sec]'); 
    ylabel('Error f_{IB,3}^{B} [mps2]');
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
    






