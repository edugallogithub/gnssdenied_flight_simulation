function display_filter_pos_obs_acc(folder_name, varargin)

fileID_gps = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_gps_obs_acc.txt']);
fgetl(fileID_gps);
input_gps = textscan(fileID_gps, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID_gps);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_filter_pos_obs_acc'];

At_sec                       = input_gps{1};
Af_ibb_mps2_truth(:,1)       = input_gps{2}; % f_ibb_mps2 truth - sensor input
Af_ibb_mps2_truth(:,2)       = input_gps{3};
Af_ibb_mps2_truth(:,3)       = input_gps{4};
Af_ibb_mps2_sens(:,1)        = input_gps{5}; % f_ibb_mps2 sensor output - observations y
Af_ibb_mps2_sens(:,2)        = input_gps{6};
Af_ibb_mps2_sens(:,3)        = input_gps{7};
Af_ibb_mps2_innov(:,1)       = input_gps{8}; % f_ibb_mps2 innovations r
Af_ibb_mps2_innov(:,2)       = input_gps{9};
Af_ibb_mps2_innov(:,3)       = input_gps{10};
Af_ibb_mps2_innov_std(:,1)   = input_gps{11}; % f_ibb_mps2 innovations standard deviations
Af_ibb_mps2_innov_std(:,2)   = input_gps{12};
Af_ibb_mps2_innov_std(:,3)   = input_gps{13};
Af_ibb_mps2_innov_mean(:,1)  = input_gps{14}; % f_ibb_mps2 innovations running mean
Af_ibb_mps2_innov_mean(:,2)  = input_gps{15};
Af_ibb_mps2_innov_mean(:,3)  = input_gps{16};
clear input_gps;

fileID_pos = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_pos_obs_acc.txt']);
if (fileID_pos >= 0)
    fgetl(fileID_pos);
    input_pos = textscan(fileID_pos, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
    fclose(fileID_pos);

    Bt_sec                       = input_pos{1};
    Bf_ibb_mps2_truth(:,1)       = input_pos{2}; % f_ibb_mps2 truth - sensor input
    Bf_ibb_mps2_truth(:,2)       = input_pos{3};
    Bf_ibb_mps2_truth(:,3)       = input_pos{4};
    Bf_ibb_mps2_sens(:,1)        = input_pos{5}; % f_ibb_mps2 sensor output - observations y
    Bf_ibb_mps2_sens(:,2)        = input_pos{6};
    Bf_ibb_mps2_sens(:,3)        = input_pos{7};
    Bf_ibb_mps2_innov(:,1)       = input_pos{8}; % f_ibb_mps2 innovations r
    Bf_ibb_mps2_innov(:,2)       = input_pos{9};
    Bf_ibb_mps2_innov(:,3)       = input_pos{10};
    Bf_ibb_mps2_innov_std(:,1)   = input_pos{11}; % f_ibb_mps2 innovations standard deviations
    Bf_ibb_mps2_innov_std(:,2)   = input_pos{12};
    Bf_ibb_mps2_innov_std(:,3)   = input_pos{13};
    Bf_ibb_mps2_innov_mean(:,1)  = input_pos{14}; % f_ibb_mps2 innovations running mean
    Bf_ibb_mps2_innov_mean(:,2)  = input_pos{15};
    Bf_ibb_mps2_innov_mean(:,3)  = input_pos{16};
    clear input_pos;

    t_sec                 = [At_sec;                        Bt_sec(2:end)];
    f_ibb_mps2_innov      = [Af_ibb_mps2_innov;      Bf_ibb_mps2_innov(2:end,:)];      clear Af_ibb_mps2_innov;      clear Bf_ibb_mps2_innov;
    f_ibb_mps2_sens       = [Af_ibb_mps2_sens;       Bf_ibb_mps2_sens(2:end,:)];       clear Af_ibb_mps2_sens;       clear Bf_ibb_mps2_sens;
    f_ibb_mps2_truth      = [Af_ibb_mps2_truth;      Bf_ibb_mps2_truth(2:end,:)];      clear Af_ibb_mps2_truth;      clear Bf_ibb_mps2_truth;
    f_ibb_mps2_innov_std  = [Af_ibb_mps2_innov_std;  Bf_ibb_mps2_innov_std(2:end,:)];  clear Af_ibb_mps2_innov_std;  clear Bf_ibb_mps2_innov_std;
    f_ibb_mps2_innov_mean = [Af_ibb_mps2_innov_mean; Bf_ibb_mps2_innov_mean(2:end,:)]; clear Af_ibb_mps2_innov_mean; clear Bf_ibb_mps2_innov_mean;
    
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
    t_sec                  = t_sec(I);
    f_ibb_mps2_innov       = f_ibb_mps2_innov(I,:);
    f_ibb_mps2_sens        = f_ibb_mps2_sens(I,:);
    f_ibb_mps2_truth       = f_ibb_mps2_truth(I,:);
    f_ibb_mps2_innov_std   = f_ibb_mps2_innov_std(I,:);
    f_ibb_mps2_innov_mean  = f_ibb_mps2_innov_mean(I,:);
    f_ibb_mps2_eval        = f_ibb_mps2_sens - f_ibb_mps2_innov;
    
    figure('units','normalized','position',[0 0.15 1.0 0.85]);

    ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
    l = plot(t_sec, f_ibb_mps2_sens(:,1), ...
             t_sec, f_ibb_mps2_truth(:,1), ...
             t_sec, f_ibb_mps2_eval(:,1)); hold on;    
    xlabel('t [sec]');
    ylabel('f_{IB,1}^{B} [mps2]');
    l(1).Color = 'g';     
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'None'; 
    l(2).Color = 'k';       
    l(3).Color = 'b';       
    ax1.YColor = 'k';
    ax1.XMinorGrid = 'on';
    ax1.YMinorGrid = 'on';
    legend('sensed', 'truth', 'evaluated');
    grid on;   
       
    ax3 = subplot('Position',[0.05 0.38 0.43 0.27]);
    l = plot(t_sec, [f_ibb_mps2_sens(:,2) f_ibb_mps2_truth(:,2) f_ibb_mps2_eval(:,2)]); hold on;
    xlabel('t [sec]');
    ylabel('f_{IB,2}^{B} [mps2]');
    l(1).Color = 'g';     
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'None'; 
    l(2).Color = 'k';       
    l(3).Color = 'b';       
    ax3.YColor = 'k';
    ax3.XMinorGrid = 'on';
    ax3.YMinorGrid = 'on';
    grid on;    
    
    ax5 = subplot('Position',[0.05 0.05 0.43 0.27]);
    l = plot(t_sec, [f_ibb_mps2_sens(:,3) f_ibb_mps2_truth(:,3) f_ibb_mps2_eval(:,3)]); hold on;
    xlabel('t [sec]');
    ylabel('f_{IB,3}^{B} [mps2]');
    l(1).Color = 'g';     
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'None'; 
    l(2).Color = 'k';       
    l(3).Color = 'b';       
    ax5.YColor = 'k';
    ax5.XMinorGrid = 'on';
    ax5.YMinorGrid = 'on';
    grid on;    
    
    ax2 = subplot('Position',[0.55 0.71 0.43 0.27]);
    l = plot(t_sec, [f_ibb_mps2_innov(:,1) (-f_ibb_mps2_innov_std(:,1)) f_ibb_mps2_innov_std(:,1) f_ibb_mps2_innov_mean(:,1)]); hold on;
    xlabel('t [sec]'); 
    legend('innovation', 'std -', 'std +', 'innov mean');
    ylabel('Error f_{IB,1}^{B} [mps2]');
    l(1).Color = 'b';
    l(2).Color = 'r';
    l(2).LineStyle = '--';
    l(2).LineWidth = 3.;
    l(3).Color = 'r';
    l(3).LineStyle = '--';
    l(3).LineWidth = 3.;
    l(4).Color = 'b';
    l(4).LineStyle = ':';
    ax2.YColor = 'k';
    ax2.XMinorGrid = 'on';
    ax2.YMinorGrid = 'on';
    grid on;

    ax4 = subplot('Position',[0.55 0.38 0.43 0.27]);
    l = plot(t_sec, [f_ibb_mps2_innov(:,2) (-f_ibb_mps2_innov_std(:,2)) f_ibb_mps2_innov_std(:,2) f_ibb_mps2_innov_mean(:,2)]); hold on;
    xlabel('t [sec]'); 
    ylabel('Error f_{IB,2}^{B} [mps2]');
    l(1).Color = 'b';
    l(2).Color = 'r';
    l(2).LineStyle = '--';
    l(2).LineWidth = 3.;
    l(3).Color = 'r';
    l(3).LineStyle = '--';
    l(3).LineWidth = 3.;
    l(4).Color = 'b';
    l(4).LineStyle = ':';
    ax4.YColor = 'k';
    ax4.XMinorGrid = 'on';
    ax4.YMinorGrid = 'on';
    grid on;
       
    ax6 = subplot('Position',[0.55 0.05 0.43 0.27]);
    l = plot(t_sec, [f_ibb_mps2_innov(:,3) (-f_ibb_mps2_innov_std(:,3)) f_ibb_mps2_innov_std(:,3) f_ibb_mps2_innov_mean(:,3)]); hold on;
    xlabel('t [sec]'); 
    ylabel('Error f_{IB,3}^{B} [mps2]');
    l(1).Color = 'b';
    l(2).Color = 'r';
    l(2).LineStyle = '--';
    l(2).LineWidth = 3.;
    l(3).Color = 'r';
    l(3).LineStyle = '--';
    l(3).LineWidth = 3.;
    l(4).Color = 'b';
    l(4).LineStyle = ':';
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
    At_sec                         = At_sec(I);
    Af_ibb_mps2_innov       = Af_ibb_mps2_innov(I,:);
    Af_ibb_mps2_sens        = Af_ibb_mps2_sens(I,:);
    Af_ibb_mps2_truth       = Af_ibb_mps2_truth(I,:);
    Af_ibb_mps2_innov_std   = Af_ibb_mps2_innov_std(I,:);
    Af_ibb_mps2_innov_mean  = Af_ibb_mps2_innov_mean(I,:);
    Af_ibb_mps2_eval        = Af_ibb_mps2_sens - Af_ibb_mps2_innov;
    
    figure('units','normalized','position',[0 0.15 1.0 0.85]);

    ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
    l = plot(At_sec, Af_ibb_mps2_sens(:,1), ...
             At_sec, Af_ibb_mps2_truth(:,1), ...
             At_sec, Af_ibb_mps2_eval(:,1)); hold on;    
    xlabel('t [sec]');
    ylabel('f_{IB,1}^{B} [mps2]');
    l(1).Color = 'g';     
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'None'; 
    l(2).Color = 'k';       
    l(3).Color = 'b';       
    ax1.YColor = 'k';
    ax1.XMinorGrid = 'on';
    ax1.YMinorGrid = 'on';
    legend('sensed', 'truth', 'evaluated');
    grid on;   
       
    ax3 = subplot('Position',[0.05 0.38 0.43 0.27]);
    l = plot(At_sec, [Af_ibb_mps2_sens(:,2) Af_ibb_mps2_truth(:,2) Af_ibb_mps2_eval(:,2)]); hold on;
    xlabel('t [sec]');
    ylabel('f_{IB,2}^{B} [mps2]');
    l(1).Color = 'g';     
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'None'; 
    l(2).Color = 'k';       
    l(3).Color = 'b';       
    ax3.YColor = 'k';
    ax3.XMinorGrid = 'on';
    ax3.YMinorGrid = 'on';
    grid on;    
    
    ax5 = subplot('Position',[0.05 0.05 0.43 0.27]);
    l = plot(At_sec, [Af_ibb_mps2_sens(:,3) Af_ibb_mps2_truth(:,3) Af_ibb_mps2_eval(:,3)]); hold on;
    xlabel('t [sec]');
    ylabel('f_{IB,3}^{B} [mps2]');
    l(1).Color = 'g';     
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'None'; 
    l(2).Color = 'k';       
    l(3).Color = 'b';       
    ax5.YColor = 'k';
    ax5.XMinorGrid = 'on';
    ax5.YMinorGrid = 'on';
    grid on;    
    
    ax2 = subplot('Position',[0.55 0.71 0.43 0.27]);
    l = plot(At_sec, [Af_ibb_mps2_innov(:,1) (-Af_ibb_mps2_innov_std(:,1)) Af_ibb_mps2_innov_std(:,1) Af_ibb_mps2_innov_mean(:,1)]); hold on;
    xlabel('t [sec]'); 
    legend('innovation', 'std -', 'std +', 'innov mean');
    ylabel('Error f_{IB,1}^{B} [mps2]');
    l(1).Color = 'b';
    l(2).Color = 'r';
    l(2).LineStyle = '--';
    l(2).LineWidth = 3.;
    l(3).Color = 'r';
    l(3).LineStyle = '--';
    l(3).LineWidth = 3.;
    l(4).Color = 'b';
    l(4).LineStyle = ':';
    ax2.YColor = 'k';
    ax2.XMinorGrid = 'on';
    ax2.YMinorGrid = 'on';
    grid on;

    ax4 = subplot('Position',[0.55 0.38 0.43 0.27]);
    l = plot(At_sec, [Af_ibb_mps2_innov(:,2) (-Af_ibb_mps2_innov_std(:,2)) Af_ibb_mps2_innov_std(:,2) Af_ibb_mps2_innov_mean(:,2)]); hold on;
    xlabel('t [sec]'); 
    ylabel('Error f_{IB,2}^{B} [mps2]');
    l(1).Color = 'b';
    l(2).Color = 'r';
    l(2).LineStyle = '--';
    l(2).LineWidth = 3.;
    l(3).Color = 'r';
    l(3).LineStyle = '--';
    l(3).LineWidth = 3.;
    l(4).Color = 'b';
    l(4).LineStyle = ':';
    ax4.YColor = 'k';
    ax4.XMinorGrid = 'on';
    ax4.YMinorGrid = 'on';
    grid on;
       
    ax6 = subplot('Position',[0.55 0.05 0.43 0.27]);
    l = plot(At_sec, [Af_ibb_mps2_innov(:,3) (-Af_ibb_mps2_innov_std(:,3)) Af_ibb_mps2_innov_std(:,3) Af_ibb_mps2_innov_mean(:,3)]); hold on;
    xlabel('t [sec]'); 
    ylabel('Error f_{IB,3}^{B} [mps2]');
    l(1).Color = 'b';
    l(2).Color = 'r';
    l(2).LineStyle = '--';
    l(2).LineWidth = 3.;
    l(3).Color = 'r';
    l(3).LineStyle = '--';
    l(3).LineWidth = 3.;
    l(4).Color = 'b';
    l(4).LineStyle = ':';
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
    






