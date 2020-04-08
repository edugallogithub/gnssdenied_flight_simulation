function display_filter_pos_Eacc(folder_name, varargin)

fileID_gps = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_gps_Eacc.txt']);
fgetl(fileID_gps);
input_gps = textscan(fileID_gps, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID_gps);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_filter_pos_E_acc'];

At_sec                            = input_gps{1}; % time
AEacc_mps2_est(:,1)               = input_gps{2}; % Eacc_mps2 estimated (a posteriori)
AEacc_mps2_est(:,2)               = input_gps{3};
AEacc_mps2_est(:,3)               = input_gps{4};
AEacc_mps2_est_apriori(:,1)       = input_gps{5}; % Eacc_mps2 estimated (a priori)
AEacc_mps2_est_apriori(:,2)       = input_gps{6};
AEacc_mps2_est_apriori(:,3)       = input_gps{7};
AEacc_mps2_truth(:,1)             = input_gps{8}; % Eacc_mps2 truth
AEacc_mps2_truth(:,2)             = input_gps{9};
AEacc_mps2_truth(:,3)             = input_gps{10};
Asigma_Eacc_mps2_est(:,1)         = input_gps{11}; % Eacc_mps2 estimated error std (a posteriori)
Asigma_Eacc_mps2_est(:,2)         = input_gps{12};
Asigma_Eacc_mps2_est(:,3)         = input_gps{13};
Asigma_Eacc_mps2_est_apriori(:,1) = input_gps{14}; % Eacc_mps2 estimated error std (a priori)
Asigma_Eacc_mps2_est_apriori(:,2) = input_gps{15};
Asigma_Eacc_mps2_est_apriori(:,3) = input_gps{16};
Amean_Eacc_mps2_est(:,1)          = input_gps{17}; % Eacc_mps2 estimated error mean (a posteriori)
Amean_Eacc_mps2_est(:,2)          = input_gps{18};
Amean_Eacc_mps2_est(:,3)          = input_gps{19};
Amean_Eacc_mps2_est_apriori(:,1)  = input_gps{20}; % Eacc_mps2 estimated error mean (a priori)
Amean_Eacc_mps2_est_apriori(:,2)  = input_gps{21};
Amean_Eacc_mps2_est_apriori(:,3)  = input_gps{22};
clear input_gps;

fileID_pos = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_pos_Eacc.txt']);
if (fileID_pos >= 0)
    fgetl(fileID_pos);
    input_pos = textscan(fileID_pos, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
    fclose(fileID_pos);

    Bt_sec                            = input_pos{1}; % time
    BEacc_mps2_est(:,1)               = input_pos{2}; % Eacc_mps2 estimated (a posteriori)
    BEacc_mps2_est(:,2)               = input_pos{3};
    BEacc_mps2_est(:,3)               = input_pos{4};
    BEacc_mps2_est_apriori(:,1)       = input_pos{5}; % Eacc_mps2 estimated (a priori)
    BEacc_mps2_est_apriori(:,2)       = input_pos{6};
    BEacc_mps2_est_apriori(:,3)       = input_pos{7};
    BEacc_mps2_truth(:,1)             = input_pos{8}; % Eacc_mps2 truth
    BEacc_mps2_truth(:,2)             = input_pos{9};
    BEacc_mps2_truth(:,3)             = input_pos{10};
    Bsigma_Eacc_mps2_est(:,1)         = input_pos{11}; % Eacc_mps2 estimated error std (a posteriori)
    Bsigma_Eacc_mps2_est(:,2)         = input_pos{12};
    Bsigma_Eacc_mps2_est(:,3)         = input_pos{13};
    Bsigma_Eacc_mps2_est_apriori(:,1) = input_pos{14}; % Eacc_mps2 estimated error std (a priori)
    Bsigma_Eacc_mps2_est_apriori(:,2) = input_pos{15};
    Bsigma_Eacc_mps2_est_apriori(:,3) = input_pos{16};
    Bmean_Eacc_mps2_est(:,1)          = input_pos{17}; % Eacc_mps2 estimated error mean (a posteriori)
    Bmean_Eacc_mps2_est(:,2)          = input_pos{18};
    Bmean_Eacc_mps2_est(:,3)          = input_pos{19};
    Bmean_Eacc_mps2_est_apriori(:,1)  = input_pos{20}; % Eacc_mps2 estimated error mean (a priori)
    Bmean_Eacc_mps2_est_apriori(:,2)  = input_pos{21};
    Bmean_Eacc_mps2_est_apriori(:,3)  = input_pos{22};
    clear input_pos;

    t_sec                        = [At_sec;                             Bt_sec(2:end)];
    Eacc_mps2_est                = [AEacc_mps2_est;                BEacc_mps2_est(2:end,:)];               clear AEacc_mps2_est;                 clear BEacc_mps2_est;
    Eacc_mps2_est_apriori        = [AEacc_mps2_est_apriori;        BEacc_mps2_est_apriori(2:end,:)];       clear AEacc_mps2_est_apriori;         clear BEacc_mps2_est_apriori;
    Eacc_mps2_truth              = [AEacc_mps2_truth;              BEacc_mps2_truth(2:end,:)];             clear AEacc_mps2_truth;               clear BEacc_mps2_truth;
    sigma_Eacc_mps2_est          = [Asigma_Eacc_mps2_est;          Bsigma_Eacc_mps2_est(2:end,:)];         clear Asigma_Eacc_mps2_est;           clear Bsigma_Eacc_mps2_est;
    sigma_Eacc_mps2_est_apriori  = [Asigma_Eacc_mps2_est_apriori;  Bsigma_Eacc_mps2_est_apriori(2:end,:)]; clear Asigma_Eacc_mps2_est_apriori;   clear Bsigma_Eacc_mps2_est_apriori;
    mean_Eacc_mps2_est           = [Amean_Eacc_mps2_est;           Bmean_Eacc_mps2_est(2:end,:)];          clear Amean_Eacc_mps2_est;            clear Bmean_Eacc_mps2_est;
    mean_Eacc_mps2_est_apriori   = [Amean_Eacc_mps2_est_apriori;   Bmean_Eacc_mps2_est_apriori(2:end,:)];  clear Amean_Eacc_mps2_est_apriori;    clear Bmean_Eacc_mps2_est_apriori;

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
    t_sec                                = t_sec(I);
    Eacc_mps2_est                  = Eacc_mps2_est(I,:);
    Eacc_mps2_est_apriori          = Eacc_mps2_est_apriori(I,:);
    Eacc_mps2_truth                = Eacc_mps2_truth(I,:);
    sigma_Eacc_mps2_est            = sigma_Eacc_mps2_est(I,:);
    sigma_Eacc_mps2_est_apriori    = sigma_Eacc_mps2_est_apriori(I,:);
    mean_Eacc_mps2_est             = mean_Eacc_mps2_est(I,:);
    mean_Eacc_mps2_est_apriori     = mean_Eacc_mps2_est_apriori(I,:);

    figure('units','normalized','position',[0 0.15 1.0 0.85]);

    ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
    %ax1 = subplot(3,2,1);
    l = plot(t_sec, Eacc_mps2_est_apriori(:,1), ...
             t_sec, Eacc_mps2_est(:,1), ...
             t_sec, Eacc_mps2_truth(:,1)); hold on;
    xlabel('t [sec]'); 
    ylabel('E_{ACC,1} [mps2]');
    l(1).Color = 'b';
    l(1).LineStyle = '--';
    l(2).Color = 'b';
    l(3).Color = 'k';
    ax1.YColor = 'k';
    ax1.XMinorGrid = 'on';
    ax1.YMinorGrid = 'on';
    legend(l, 'est a priori', 'est a posteriori', 'truth');
    grid on;

    ax3 = subplot('Position',[0.05 0.38 0.43 0.27]);
    %ax3 = subplot(3,2,3);
    l = plot(t_sec, Eacc_mps2_est_apriori(:,2), ...
             t_sec, Eacc_mps2_est(:,2), ...
             t_sec, Eacc_mps2_truth(:,2)); hold on;
    xlabel('t [sec]'); 
    ylabel('E_{ACC,,2} [mps2]');
    l(1).Color = 'b';
    l(1).LineStyle = '--';
    l(2).Color = 'b';
    l(3).Color = 'k';
    ax3.YColor = 'k';
    ax3.XMinorGrid = 'on';
    ax3.YMinorGrid = 'on';
    legend(l, 'est a priori', 'est a posteriori', 'truth');
    grid on;

    ax5 = subplot('Position',[0.05 0.05 0.43 0.27]);
    %ax5 = subplot(3,2,5);
    l = plot(t_sec, Eacc_mps2_est_apriori(:,3), ...
             t_sec, Eacc_mps2_est(:,3), ...
             t_sec, Eacc_mps2_truth(:,3)); hold on;
    xlabel('t [sec]'); 
    ylabel('E_{ACC,,3} [mps2]');
    l(1).Color = 'b';
    l(1).LineStyle = '--';
    l(2).Color = 'b';
    l(3).Color = 'k';
    ax5.YColor = 'k';
    ax5.XMinorGrid = 'on';
    ax5.YMinorGrid = 'on';
    legend(l, 'est a priori', 'est a posteriori', 'truth');
    grid on;

    ax2 = subplot('Position',[0.55 0.71 0.43 0.27]);
    %ax2 = subplot(3,2,2);
    l = plot(t_sec, Eacc_mps2_est_apriori(:,1) - Eacc_mps2_truth(:,1), ...
             t_sec, - sigma_Eacc_mps2_est_apriori(:,1), ...
             t_sec, sigma_Eacc_mps2_est_apriori(:,1), ...
             t_sec, mean_Eacc_mps2_est_apriori(:,1), ...
             t_sec, Eacc_mps2_est(:,1) - Eacc_mps2_truth(:,1), ...
             t_sec, - sigma_Eacc_mps2_est(:,1), ...
             t_sec, sigma_Eacc_mps2_est(:,1), ...    
             t_sec, mean_Eacc_mps2_est(:,1)); hold on;
    xlabel('t [sec]'); 
    legend('priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');
    ylabel('Error E_{ACC,1} [mps2]');
    l(1).Color = 'k';
    l(2).Color = 'k';
    l(2).LineStyle = '--';
    l(3).Color = 'k';
    l(3).LineStyle = '--';
    l(4).Color = 'k';
    l(4).LineStyle = ':';
    l(5).Color = 'b';
    l(6).Color = 'b';
    l(6).LineStyle = '--';
    l(7).Color = 'b';
    l(7).LineStyle = '--';
    l(8).Color = 'b';
    l(8).LineStyle = ':';
    ax2.YColor = 'k';
    ax2.XMinorGrid = 'on';
    ax2.YMinorGrid = 'on';
    grid on;

    ax4 = subplot('Position',[0.55 0.38 0.43 0.27]);
    %ax4 = subplot(3,2,4);
    l = plot(t_sec,  Eacc_mps2_est_apriori(:,2) - Eacc_mps2_truth(:,2), ...
             t_sec,  - sigma_Eacc_mps2_est_apriori(:,2), ...
             t_sec,  sigma_Eacc_mps2_est_apriori(:,2), ...
             t_sec,  mean_Eacc_mps2_est_apriori(:,2), ...
             t_sec,  Eacc_mps2_est(:,2) - Eacc_mps2_truth(:,2), ...
             t_sec,  - sigma_Eacc_mps2_est(:,2), ...
             t_sec,  sigma_Eacc_mps2_est(:,2), ...
             t_sec,  mean_Eacc_mps2_est(:,2)); hold on;    
    legend('priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');
    xlabel('t [sec]'); 
    ylabel('Error E_{ACC,2} [mps2]');
    l(1).Color = 'k';
    l(2).Color = 'k';
    l(2).LineStyle = '--';
    l(3).Color = 'k';
    l(3).LineStyle = '--';
    l(4).Color = 'k';
    l(4).LineStyle = ':';
    l(5).Color = 'b';
    l(6).Color = 'b';
    l(6).LineStyle = '--';
    l(7).Color = 'b';
    l(7).LineStyle = '--';
    l(8).Color = 'b';
    l(8).LineStyle = ':';
    ax4.YColor = 'k';
    ax4.XMinorGrid = 'on';
    ax4.YMinorGrid = 'on';
    grid on;

    ax6 = subplot('Position',[0.55 0.05 0.43 0.27]);
    %ax6 = subplot(3,2,6);
    l = plot(t_sec,  Eacc_mps2_est_apriori(:,3) - Eacc_mps2_truth(:,3), ...
             t_sec,  - sigma_Eacc_mps2_est_apriori(:,3), ...
             t_sec,  sigma_Eacc_mps2_est_apriori(:,3), ...   
             t_sec,  mean_Eacc_mps2_est_apriori(:,3), ...
             t_sec,  Eacc_mps2_est(:,3) - Eacc_mps2_truth(:,3), ...
             t_sec,  - sigma_Eacc_mps2_est(:,3), ...
             t_sec,  sigma_Eacc_mps2_est(:,3), ...    
             t_sec,  mean_Eacc_mps2_est(:,3)); hold on;    
    legend('priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');     
    xlabel('t [sec]'); 
    ylabel('Error E_{ACC,3} [mps2]');
    l(1).Color = 'k';
    l(2).Color = 'k';
    l(2).LineStyle = '--';
    l(3).Color = 'k';
    l(3).LineStyle = '--';
    l(4).Color = 'k';
    l(4).LineStyle = ':';
    l(5).Color = 'b';
    l(6).Color = 'b';
    l(6).LineStyle = '--';
    l(7).Color = 'b';
    l(7).LineStyle = '--';
    l(8).Color = 'b';
    l(8).LineStyle = ':';
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
    At_sec                                = At_sec(I);
    AEacc_mps2_est                  = AEacc_mps2_est(I,:);
    AEacc_mps2_est_apriori          = AEacc_mps2_est_apriori(I,:);
    AEacc_mps2_truth                = AEacc_mps2_truth(I,:);
    Asigma_Eacc_mps2_est            = Asigma_Eacc_mps2_est(I,:);
    Asigma_Eacc_mps2_est_apriori    = Asigma_Eacc_mps2_est_apriori(I,:);
    Amean_Eacc_mps2_est             = Amean_Eacc_mps2_est(I,:);
    Amean_Eacc_mps2_est_apriori     = Amean_Eacc_mps2_est_apriori(I,:);

    figure('units','normalized','position',[0 0.15 1.0 0.85]);

    ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
    %ax1 = subplot(3,2,1);
    l = plot(At_sec, AEacc_mps2_est_apriori(:,1), ...
             At_sec, AEacc_mps2_est(:,1), ...
             At_sec, AEacc_mps2_truth(:,1)); hold on;
    xlabel('t [sec]'); 
    ylabel('E_{ACC,1} [mps2]');
    l(1).Color = 'b';
    l(1).LineStyle = '--';
    l(2).Color = 'b';
    l(3).Color = 'k';
    ax1.YColor = 'k';
    ax1.XMinorGrid = 'on';
    ax1.YMinorGrid = 'on';
    legend(l, 'est a priori', 'est a posteriori', 'truth');
    grid on;

    ax3 = subplot('Position',[0.05 0.38 0.43 0.27]);
    %ax3 = subplot(3,2,3);
    l = plot(At_sec, AEacc_mps2_est_apriori(:,2), ...
             At_sec, AEacc_mps2_est(:,2), ...
             At_sec, AEacc_mps2_truth(:,2)); hold on;
    xlabel('t [sec]'); 
    ylabel('E_{ACC,2} [mps2]');
    l(1).Color = 'b';
    l(1).LineStyle = '--';
    l(2).Color = 'b';
    l(3).Color = 'k';
    ax3.YColor = 'k';
    ax3.XMinorGrid = 'on';
    ax3.YMinorGrid = 'on';
    legend(l, 'est a priori', 'est a posteriori', 'truth');
    grid on;

    ax5 = subplot('Position',[0.05 0.05 0.43 0.27]);
    %ax5 = subplot(3,2,5);
    l = plot(At_sec, AEacc_mps2_est_apriori(:,3), ...
             At_sec, AEacc_mps2_est(:,3), ...
             At_sec, AEacc_mps2_truth(:,3)); hold on;
    xlabel('t [sec]'); 
    ylabel('E_{ACC,3} [mps2]');
    l(1).Color = 'b';
    l(1).LineStyle = '--';
    l(2).Color = 'b';
    l(3).Color = 'k';
    ax5.YColor = 'k';
    ax5.XMinorGrid = 'on';
    ax5.YMinorGrid = 'on';
    legend(l, 'est a priori', 'est a posteriori', 'truth');
    grid on;

    ax2 = subplot('Position',[0.55 0.71 0.43 0.27]);
    %ax2 = subplot(3,2,2);
    l = plot(At_sec, AEacc_mps2_est_apriori(:,1) - AEacc_mps2_truth(:,1), ...
             At_sec, - Asigma_Eacc_mps2_est_apriori(:,1), ...
             At_sec, Asigma_Eacc_mps2_est_apriori(:,1), ...
             At_sec, Amean_Eacc_mps2_est_apriori(:,1), ...
             At_sec, AEacc_mps2_est(:,1) - AEacc_mps2_truth(:,1), ...
             At_sec, - Asigma_Eacc_mps2_est(:,1), ...
             At_sec, Asigma_Eacc_mps2_est(:,1), ...    
             At_sec, Amean_Eacc_mps2_est(:,1)); hold on;
    xlabel('t [sec]'); 
    legend('priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');
    ylabel('Error E_{ACC,1} [mps2]');
    l(1).Color = 'k';
    l(2).Color = 'k';
    l(2).LineStyle = '--';
    l(3).Color = 'k';
    l(3).LineStyle = '--';
    l(4).Color = 'k';
    l(4).LineStyle = ':';
    l(5).Color = 'b';
    l(6).Color = 'b';
    l(6).LineStyle = '--';
    l(7).Color = 'b';
    l(7).LineStyle = '--';
    l(8).Color = 'b';
    l(8).LineStyle = ':';
    ax2.YColor = 'k';
    ax2.XMinorGrid = 'on';
    ax2.YMinorGrid = 'on';
    grid on;

    ax4 = subplot('Position',[0.55 0.38 0.43 0.27]);
    %ax4 = subplot(3,2,4);
    l = plot(At_sec,  AEacc_mps2_est_apriori(:,2) - AEacc_mps2_truth(:,2), ...
             At_sec,  - Asigma_Eacc_mps2_est_apriori(:,2), ...
             At_sec,  Asigma_Eacc_mps2_est_apriori(:,2), ...
             At_sec,  Amean_Eacc_mps2_est_apriori(:,2), ...
             At_sec,  AEacc_mps2_est(:,2) - AEacc_mps2_truth(:,2), ...
             At_sec,  - Asigma_Eacc_mps2_est(:,2), ...
             At_sec,  Asigma_Eacc_mps2_est(:,2), ...
             At_sec,  Amean_Eacc_mps2_est(:,2)); hold on;    
    legend('priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');
    xlabel('t [sec]'); 
    ylabel('Error E_{ACC,2} [mps2]');
    l(1).Color = 'k';
    l(2).Color = 'k';
    l(2).LineStyle = '--';
    l(3).Color = 'k';
    l(3).LineStyle = '--';
    l(4).Color = 'k';
    l(4).LineStyle = ':';
    l(5).Color = 'b';
    l(6).Color = 'b';
    l(6).LineStyle = '--';
    l(7).Color = 'b';
    l(7).LineStyle = '--';
    l(8).Color = 'b';
    l(8).LineStyle = ':';
    ax4.YColor = 'k';
    ax4.XMinorGrid = 'on';
    ax4.YMinorGrid = 'on';
    grid on;

    ax6 = subplot('Position',[0.55 0.05 0.43 0.27]);
    %ax6 = subplot(3,2,6);
    l = plot(At_sec,  AEacc_mps2_est_apriori(:,3) - AEacc_mps2_truth(:,3), ...
             At_sec,  - Asigma_Eacc_mps2_est_apriori(:,3), ...
             At_sec,  Asigma_Eacc_mps2_est_apriori(:,3), ...   
             At_sec,  Amean_Eacc_mps2_est_apriori(:,3), ...
             At_sec,  AEacc_mps2_est(:,3) - AEacc_mps2_truth(:,3), ...
             At_sec,  - Asigma_Eacc_mps2_est(:,3), ...
             At_sec,  Asigma_Eacc_mps2_est(:,3), ...    
             At_sec,  Amean_Eacc_mps2_est(:,3)); hold on;    
    legend('priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean', 'Location', 'northwest');     
    xlabel('t [sec]'); 
    ylabel('Error E_{ACC,3} [mps2]');
    l(1).Color = 'k';
    l(2).Color = 'k';
    l(2).LineStyle = '--';
    l(3).Color = 'k';
    l(3).LineStyle = '--';
    l(4).Color = 'k';
    l(4).LineStyle = ':';
    l(5).Color = 'b';
    l(6).Color = 'b';
    l(6).LineStyle = '--';
    l(7).Color = 'b';
    l(7).LineStyle = '--';
    l(8).Color = 'b';
    l(8).LineStyle = ':';
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
    






