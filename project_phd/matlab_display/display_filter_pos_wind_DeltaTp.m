function display_filter_pos_wind_DeltaTp(folder_name, varargin)

fileID_gps = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_gps_wind_DeltaTp.txt']);
fgetl(fileID_gps);
input_gps = textscan(fileID_gps, '%f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID_gps);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_filter_pos_wind_DeltaTp'];

At_sec                       = input_gps{1}; % time
Avlf_n_mps_est(:,1)          = input_gps{2}; % vlf_n_mps estimated
Avlf_n_mps_est(:,2)          = input_gps{3};
Avlf_n_mps_est(:,3)          = input_gps{4};
Avlf_n_mps_truth(:,1)        = input_gps{5}; % vlf_n_mps truth
Avlf_n_mps_truth(:,2)        = input_gps{6};
Avlf_n_mps_truth(:,3)        = input_gps{7};
ADeltaT_degK_est(:,1)        = input_gps{8};  % DeltaT_degK estimated
ADeltaT_degK_truth(:,1)      = input_gps{9};  % DeltaT_degK truth
ADeltap_pa_est(:,1)          = input_gps{10}; % Deltap_pa estimated
ADeltap_pa_truth(:,1)        = input_gps{11}; % Deltap_pa truth
clear input_gps;

fileID_pos = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_pos_wind_DeltaTp.txt']);
if (fileID_pos >= 0)
    fgetl(fileID_pos);
    input_pos = textscan(fileID_pos, '%f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
    fclose(fileID_pos);

    Bt_sec                       = input_pos{1}; % time
    Bvlf_n_mps_est(:,1)          = input_pos{2}; % vlf_n_mps estimated
    Bvlf_n_mps_est(:,2)          = input_pos{3};
    Bvlf_n_mps_est(:,3)          = input_pos{4};
    Bvlf_n_mps_truth(:,1)        = input_pos{5}; % vlf_n_mps truth
    Bvlf_n_mps_truth(:,2)        = input_pos{6};
    Bvlf_n_mps_truth(:,3)        = input_pos{7};
    BDeltaT_degK_est(:,1)        = input_pos{8};  % DeltaT_degK estimated
    BDeltaT_degK_truth(:,1)      = input_pos{9};  % DeltaT_degK truth
    BDeltap_pa_est(:,1)          = input_pos{10}; % Deltap_pa estimated
    BDeltap_pa_truth(:,1)        = input_pos{11}; % Deltap_pa truth
    clear input_pos;

    t_sec             = [At_sec;              Bt_sec(2:end)];
    vlf_n_mps_est     = [Avlf_n_mps_est;      Bvlf_n_mps_est(2:end,:)];      clear Avlf_n_mps_est;     clear Bvlf_n_mps_est;
    vlf_n_mps_truth   = [Avlf_n_mps_truth;    Bvlf_n_mps_truth(2:end,:)];    clear Avlf_n_mps_truth;   clear Bvlf_n_mps_truth;
    DeltaT_degK_est   = [ADeltaT_degK_est;    BDeltaT_degK_est(2:end,:)];    clear ADeltaT_degK_est;   clear BDeltaT_degK_est;
    DeltaT_degK_truth = [ADeltaT_degK_truth;  BDeltaT_degK_truth(2:end,:)];  clear ADeltaT_degK_truth; clear BDeltaT_degK_truth;
    Deltap_pa_est     = [ADeltap_pa_est;      BDeltap_pa_est(2:end,:)];      clear ADeltap_pa_est;     clear BDeltap_pa_est;
    Deltap_pa_truth   = [ADeltap_pa_truth;    BDeltap_pa_truth(2:end,:)];    clear ADeltap_pa_truth;   clear BDeltap_pa_truth;

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
    t_sec             = t_sec(I);
    vlf_n_mps_est     = vlf_n_mps_est(I,:);
    vlf_n_mps_truth   = vlf_n_mps_truth(I,:);
    DeltaT_degK_est   = DeltaT_degK_est(I,:);
    DeltaT_degK_truth = DeltaT_degK_truth(I,:);
    Deltap_pa_est     = Deltap_pa_est(I,:);
    Deltap_pa_truth   = Deltap_pa_truth(I,:);

    figure('units','normalized','position',[0 0.15 1.0 0.85]);

    ax1 = subplot('Position',[0.05 0.82 0.90 0.15]);
    l = plot(t_sec, vlf_n_mps_est(:,1), ...
             t_sec, vlf_n_mps_truth(:,1)); hold on;
    ylabel('v_{LF,1}^{N} [mps]');
    l(1).Color = 'b';
    l(2).Color = 'k';
    ax1.YColor = 'k';
    ax1.XMinorGrid = 'on';
    ax1.YMinorGrid = 'on';
    legend(l, 'est', 'truth');
    grid on;

    ax2 = subplot('Position',[0.05 0.63 0.90 0.15]);
    l = plot(t_sec, vlf_n_mps_est(:,2), ...
             t_sec, vlf_n_mps_truth(:,2)); hold on;
    ylabel('v_{LF,2}^{N} [mps]');
    l(1).Color = 'b';
    l(2).Color = 'k';
    ax2.YColor = 'k';
    ax2.XMinorGrid = 'on';
    ax2.YMinorGrid = 'on';
    legend(l, 'est', 'truth');
    grid on;
        
    ax3 = subplot('Position',[0.05 0.44 0.90 0.15]);
    l = plot(t_sec, vlf_n_mps_est(:,3), ...
             t_sec, vlf_n_mps_truth(:,3)); hold on;
    ylabel('v_{LF,3}^{N} [mps]');
    l(1).Color = 'b';
    l(2).Color = 'k';
    ax3.YColor = 'k';
    ax3.XMinorGrid = 'on';
    ax3.YMinorGrid = 'on';
    legend(l, 'est', 'truth');
    grid on;
    
    ax4 = subplot('Position',[0.05 0.25 0.90 0.15]);
    l = plot(t_sec, Deltap_pa_est, ...
             t_sec, Deltap_pa_truth); hold on;
    ylabel('\Delta p [pa]');
    l(1).Color = 'b';
    l(2).Color = 'k';
    ax4.YColor = 'k';
    ax4.XMinorGrid = 'on';
    ax4.YMinorGrid = 'on';
    legend(l, 'est', 'truth');
    grid on;
    
    ax5 = subplot('Position',[0.05 0.05 0.90 0.15]);
    l = plot(t_sec, DeltaT_degK_est, ...
             t_sec, DeltaT_degK_truth); hold on;
    xlabel('t [sec]'); 
    ylabel('\Delta T [degK]');
    l(1).Color = 'b';
    l(2).Color = 'k';
    ax4.YColor = 'k';
    ax4.XMinorGrid = 'on';
    ax4.YMinorGrid = 'on';
    legend(l, 'est', 'truth');
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
    At_sec              = At_sec(I);
    Avlf_n_mps_est      = Avlf_n_mps_est(I,:);
    Avlf_n_mps_truth    = Avlf_n_mps_truth(I,:);
    ADeltaT_degK_est    = ADeltaT_degK_est(I,:);
    ADeltaT_degK_truth  = ADeltaT_degK_truth(I,:);
    ADeltap_pa_est      = ADeltap_pa_est(I,:);
    ADeltap_pa_truth    = ADeltap_pa_truth(I,:);
    
    figure('units','normalized','position',[0 0.15 1.0 0.85]);
    
    ax1 = subplot('Position',[0.05 0.82 0.90 0.15]);
    l = plot(At_sec, Avlf_n_mps_est(:,1), ...
             At_sec, Avlf_n_mps_truth(:,1)); hold on;
    ylabel('v_{LF,1}^{N} [mps]');
    l(1).Color = 'b';
    l(2).Color = 'k';
    ax1.YColor = 'k';
    ax1.XMinorGrid = 'on';
    ax1.YMinorGrid = 'on';
    legend(l, 'est', 'truth');
    grid on;

    ax2 = subplot('Position',[0.05 0.63 0.90 0.15]);
    l = plot(At_sec, Avlf_n_mps_est(:,2), ...
             At_sec, Avlf_n_mps_truth(:,2)); hold on;
    ylabel('v_{LF,2}^{N} [mps]');
    l(1).Color = 'b';
    l(2).Color = 'k';
    ax2.YColor = 'k';
    ax2.XMinorGrid = 'on';
    ax2.YMinorGrid = 'on';
    legend(l, 'est', 'truth');
    grid on;

    ax3 = subplot('Position',[0.05 0.44 0.90 0.15]);
    l = plot(At_sec, Avlf_n_mps_est(:,3), ...
             At_sec, Avlf_n_mps_truth(:,3)); hold on;
    ylabel('v_{LF,3}^{N} [mps]');
    l(1).Color = 'b';
    l(2).Color = 'k';
    ax3.YColor = 'k';
    ax3.XMinorGrid = 'on';
    ax3.YMinorGrid = 'on';
    legend(l, 'est', 'truth');
    grid on;
    
    ax4 = subplot('Position',[0.05 0.25 0.90 0.15]);
    l = plot(At_sec, ADeltap_pa_est, ...
             At_sec, ADeltap_pa_truth); hold on;
    ylabel('\Delta p [pa]');
    l(1).Color = 'b';
    l(2).Color = 'k';
    ax4.YColor = 'k';
    ax4.XMinorGrid = 'on';
    ax4.YMinorGrid = 'on';
    legend(l, 'est', 'truth');
    grid on;    
    orGrid = 'on';
    ax2.YMinorGrid = 'on';
    grid on;      
    
    ax5 = subplot('Position',[0.05 0.05 0.90 0.15]);
    l = plot(At_sec, ADeltaT_degK_est, ...
             At_sec, ADeltaT_degK_truth); hold on;
    xlabel('t [sec]'); 
    ylabel('\Delta T [degK]');
    l(1).Color = 'b';
    l(2).Color = 'k';
    ax4.YColor = 'k';
    ax4.XMinorGrid = 'on';
    ax4.YMinorGrid = 'on';
    legend(l, 'est', 'truth');
    grid on;    
    orGrid = 'on';
    ax2.YMinorGrid = 'on';
    grid on;  
end

%savefig([plot_name '.fig']);

%Image = getframe(gcf);
%imwrite(Image.cdata, [plot_name '.jpg']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    






