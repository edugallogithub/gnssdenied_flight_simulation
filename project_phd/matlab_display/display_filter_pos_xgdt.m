function display_filter_pos_xgdt(folder_name, varargin)

fileID_gps = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_gps_xgdt.txt']);
fgetl(fileID_gps);
input_gps = textscan(fileID_gps, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID_gps);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_filter_pos_xgdt'];

At_sec                        = input_gps{1};  % time
Alambda_deg_est               = input_gps{2};  % lambda_deg estimated (a posteriori)
Aphi_deg_est                  = input_gps{3};  % phi_deg    estimated (a posteriori)
Ah_m_est                      = input_gps{4};  % h_m        estimated (a posteriori)
Alambda_deg_est_apriori       = input_gps{5};  % lambda_deg estimated (a priori)
Aphi_deg_est_apriori          = input_gps{6};  % phi_deg    estimated (a priori)
Ah_m_est_apriori              = input_gps{7};  % h_m        estimated (a priori)
Alambda_deg_truth             = input_gps{8};  % lambda_deg truth
Aphi_deg_truth                = input_gps{9};  % phi_deg    truth
Ah_m_truth                    = input_gps{10}; % h_m        truth
Alambda_deg_sensed            = input_gps{11}; % lambda_deg sensed
Aphi_deg_sensed               = input_gps{12}; % phi_deg    sensed
Ah_m_sensed                   = input_gps{13}; % h_m        sensed
Asigma_lambda_deg_est         = input_gps{14}; % lambda_deg estimated error std (a posteriori)
Asigma_phi_deg_est            = input_gps{15}; % phi_deg    estimated error std (a posteriori)
Asigma_h_m_est                = input_gps{16}; % h_m        estimated error std (a posteriori)
Asigma_lambda_deg_est_apriori = input_gps{17}; % lambda_deg estimated error std (a priori)
Asigma_phi_deg_est_apriori    = input_gps{18}; % phi_deg    estimated error std (a priori)
Asigma_h_m_est_apriori        = input_gps{19}; % h_m        estimated error std (a priori)
Amean_lambda_deg_est          = input_gps{20}; % lambda_deg estimated error mean (a posteriori)
Amean_phi_deg_est             = input_gps{21}; % phi_deg    estimated error mean (a posteriori)
Amean_h_m_est                 = input_gps{22}; % h_m        estimated error mean (a posteriori)
Amean_lambda_deg_est_apriori  = input_gps{23}; % lambda_deg estimated error mean (a priori)
Amean_phi_deg_est_apriori     = input_gps{24}; % phi_deg    estimated error mean (a priori)
Amean_h_m_est_apriori         = input_gps{25}; % h_m        estimated error mean (a priori)
clear input_gps;

fileID_pos = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/filter_pos_xgdt.txt']);
if (fileID_pos >= 0)
    fgetl(fileID_pos);
    input_pos = textscan(fileID_pos, '%f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
    fclose(fileID_pos);

    Bt_sec                        = input_pos{1};  % time
    Blambda_deg_est               = input_pos{2};  % lambda_deg integrated
    Bphi_deg_est                  = input_pos{3};  % phi_deg    integrated
    Bh_m_est                      = input_pos{4};  % h_m        integrated
    Blambda_deg_truth             = input_pos{5};  % lambda_deg truth
    Bphi_deg_truth                = input_pos{6};  % phi_deg    truth
    Bh_m_truth                    = input_pos{7};  % h_m        truth
    clear input_pos;

    t_sec                        = [At_sec;             Bt_sec(2:end)];
    lambda_deg_est               = [Alambda_deg_est;    Blambda_deg_est(2:end)];
    phi_deg_est                  = [Aphi_deg_est;       Bphi_deg_est(2:end)];
    h_m_est                      = [Ah_m_est;           Bh_m_est(2:end)];
    lambda_deg_est_apriori       = Alambda_deg_est_apriori;
    phi_deg_est_apriori          = Aphi_deg_est_apriori;
    h_m_est_apriori              = Ah_m_est_apriori;
    lambda_deg_truth             = [Alambda_deg_truth;  Blambda_deg_truth(2:end)];
    phi_deg_truth                = [Aphi_deg_truth;     Bphi_deg_truth(2:end)];
    h_m_truth                    = [Ah_m_truth;         Bh_m_truth(2:end)];
    sigma_lambda_deg_est         = Asigma_lambda_deg_est;
    sigma_phi_deg_est            = Asigma_phi_deg_est;
    sigma_h_m_est                = Asigma_h_m_est;
    sigma_lambda_deg_est_apriori = Asigma_lambda_deg_est_apriori;
    sigma_phi_deg_est_apriori    = Asigma_phi_deg_est_apriori;
    sigma_h_m_est_apriori        = Asigma_h_m_est_apriori;
    mean_lambda_deg_est          = Amean_lambda_deg_est;
    mean_phi_deg_est             = Amean_phi_deg_est;
    mean_h_m_est                 = Amean_h_m_est;
    mean_lambda_deg_est_apriori  = Amean_lambda_deg_est_apriori;
    mean_phi_deg_est_apriori     = Amean_phi_deg_est_apriori;
    mean_h_m_est_apriori         = Amean_h_m_est_apriori;

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
    t_sec              = t_sec(I);
    lambda_deg_est     = lambda_deg_est(I);
    phi_deg_est        = phi_deg_est(I);
    h_m_est            = h_m_est(I);
    lambda_deg_truth   = lambda_deg_truth(I);
    phi_deg_truth      = phi_deg_truth(I);
    h_m_truth          = h_m_truth(I);

    AI = find((At_sec >= tmin_sec) & (At_sec <= tmax_sec));
    At_sec                          = At_sec(AI);
    Alambda_deg_est                 = Alambda_deg_est(AI);
    Aphi_deg_est                    = Aphi_deg_est(AI);
    Ah_m_est                        = Ah_m_est(AI);
    Alambda_deg_est_apriori         = Alambda_deg_est_apriori(AI);
    Aphi_deg_est_apriori            = Aphi_deg_est_apriori(AI);
    Ah_m_est_apriori                = Ah_m_est_apriori(AI);    
    Alambda_deg_truth               = Alambda_deg_truth(AI);
    Aphi_deg_truth                  = Aphi_deg_truth(AI);
    Ah_m_truth                      = Ah_m_truth(AI);
    Alambda_deg_sensed              = Alambda_deg_sensed(AI);
    Aphi_deg_sensed                 = Aphi_deg_sensed(AI);
    Ah_m_sensed                     = Ah_m_sensed(AI);
    Asigma_lambda_deg_est           = Asigma_lambda_deg_est(AI);
    Asigma_phi_deg_est              = Asigma_phi_deg_est(AI);
    Asigma_h_m_est                  = Asigma_h_m_est(AI);
    Asigma_lambda_deg_est_apriori   = Asigma_lambda_deg_est_apriori(AI);
    Asigma_phi_deg_est_apriori      = Asigma_phi_deg_est_apriori(AI);
    Asigma_h_m_est_apriori          = Asigma_h_m_est_apriori(AI);
    Amean_lambda_deg_est            = Amean_lambda_deg_est(AI);
    Amean_phi_deg_est               = Amean_phi_deg_est(AI);
    Amean_h_m_est                   = Amean_h_m_est(AI);
    Amean_lambda_deg_est_apriori    = Amean_lambda_deg_est_apriori(AI);
    Amean_phi_deg_est_apriori       = Amean_phi_deg_est_apriori(AI);
    Amean_h_m_est_apriori           = Amean_h_m_est_apriori(AI);
        
    figure('units','normalized','position',[0 0.15 1.0 0.85]);

    ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
    l = plot(At_sec, Alambda_deg_sensed, ...
             At_sec, Alambda_deg_est_apriori, ...
             t_sec,  lambda_deg_est, ...
             t_sec,  lambda_deg_truth); hold on;
    xlabel('t [sec]'); 
    ylabel('\lambda [deg]');
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
    l = plot(At_sec, Aphi_deg_sensed, ...
             At_sec, Aphi_deg_est_apriori, ...
             t_sec,  phi_deg_est, ...
             t_sec,  phi_deg_truth); hold on;
    xlabel('t [sec]'); 
    ylabel('\phi [deg]');
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
    l = plot(At_sec, Ah_m_sensed, ...
             At_sec, Ah_m_est_apriori, ...
             t_sec,  h_m_est, ...
             t_sec,  h_m_truth); hold on;
    xlabel('t [sec]'); 
    ylabel('h [m]');
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
    l = plot(At_sec, Alambda_deg_sensed - Alambda_deg_truth, ...
             At_sec, Alambda_deg_est_apriori - Alambda_deg_truth, ...
             At_sec, Amean_lambda_deg_est_apriori, ...
             t_sec,  lambda_deg_est - lambda_deg_truth); hold on;    

    xlabel('t [sec]'); 
    legend('sensed error', ...
           'priori error', 'priori error mean', ...
           'poster & integr error');
    ylabel('Error \lambda [deg]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'k';
    l(3).Color = 'k';
    l(3).LineStyle = ':';
    l(4).Color = 'b';
    ax2.YColor = 'k';
    ax2.XMinorGrid = 'on';
    ax2.YMinorGrid = 'on';
    grid on;

    ax4 = subplot('Position',[0.55 0.38 0.43 0.27]);
    l = plot(At_sec, Aphi_deg_sensed - Aphi_deg_truth, ...
             At_sec, Aphi_deg_est_apriori - Aphi_deg_truth, ...
             At_sec, Amean_phi_deg_est_apriori, ...
             t_sec,  phi_deg_est - phi_deg_truth); hold on;    
    legend('sensed error', ...
           'priori error', 'priori error mean', ...
           'poster & integr error');
    xlabel('t [sec]'); 
    ylabel('Error \phi [deg]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'k';
    l(3).Color = 'k';
    l(3).LineStyle = ':';
    l(4).Color = 'b';
    ax4.YColor = 'k';
    ax4.XMinorGrid = 'on';
    ax4.YMinorGrid = 'on';
    grid on;

    ax6 = subplot('Position',[0.55 0.05 0.43 0.27]);
    l = plot(At_sec, Ah_m_sensed - Ah_m_truth, ...
             At_sec, Ah_m_est_apriori - Ah_m_truth, ...
             At_sec,  - Asigma_h_m_est_apriori, ...
             At_sec,  Asigma_h_m_est_apriori, ...
             At_sec,  Amean_h_m_est_apriori, ...
             t_sec,  h_m_est - h_m_truth, ...
             At_sec,  - Asigma_h_m_est, ...
             At_sec,  Asigma_h_m_est, ...
             At_sec,  Amean_h_m_est); hold on;
    legend('sensed error', ...
           'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster & integr error', 'poster std -', 'poster std +', 'poster error mean');     
    xlabel('t [sec]'); 
    ylabel('Error h [m]');
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
    Alambda_deg_est                 = Alambda_deg_est(I);
    Aphi_deg_est                    = Aphi_deg_est(I);
    Ah_m_est                        = Ah_m_est(I);
    Alambda_deg_est_apriori         = Alambda_deg_est_apriori(I);
    Aphi_deg_est_apriori            = Aphi_deg_est_apriori(I);
    Ah_m_est_apriori                = Ah_m_est_apriori(I);
    Alambda_deg_truth               = Alambda_deg_truth(I);
    Aphi_deg_truth                  = Aphi_deg_truth(I);
    Ah_m_truth                      = Ah_m_truth(I);
    Alambda_deg_sensed              = Alambda_deg_sensed(I);
    Aphi_deg_sensed                 = Aphi_deg_sensed(I);
    Ah_m_sensed                     = Ah_m_sensed(I);
    Asigma_lambda_deg_est           = Asigma_lambda_deg_est(I);
    Asigma_phi_deg_est              = Asigma_phi_deg_est(I);
    Asigma_h_m_est                  = Asigma_h_m_est(I);
    Asigma_lambda_deg_est_apriori   = Asigma_lambda_deg_est_apriori(I);
    Asigma_phi_deg_est_apriori      = Asigma_phi_deg_est_apriori(I);
    Asigma_h_m_est_apriori          = Asigma_h_m_est_apriori(I);
    Amean_lambda_deg_est            = Amean_lambda_deg_est(I);
    Amean_phi_deg_est               = Amean_phi_deg_est(I);
    Amean_h_m_est                   = Amean_h_m_est(I);
    Amean_lambda_deg_est_apriori    = Amean_lambda_deg_est_apriori(I);
    Amean_phi_deg_est_apriori       = Amean_phi_deg_est_apriori(I);
    Amean_h_m_est_apriori           = Amean_h_m_est_apriori(I);

    figure('units','normalized','position',[0 0.15 1.0 0.85]);

    ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
    %ax1 = subplot(3,2,1);
    l = plot(At_sec, Alambda_deg_sensed, ...
             At_sec, Alambda_deg_est_apriori, ...
             At_sec, Alambda_deg_est, ...
             At_sec, Alambda_deg_truth); hold on;
    xlabel('t [sec]'); 
    ylabel('\lambda [deg]');
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
    l = plot(At_sec, Aphi_deg_sensed, ...
             At_sec, Aphi_deg_est_apriori, ...
             At_sec, Aphi_deg_est, ...
             At_sec, Aphi_deg_truth); hold on;
    xlabel('t [sec]'); 
    ylabel('\phi [deg]');
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
    l = plot(At_sec, Ah_m_sensed, ...
             At_sec, Ah_m_est_apriori, ...
             At_sec, Ah_m_est, ...
             At_sec, Ah_m_truth); hold on;
    xlabel('t [sec]'); 
    ylabel('h [m]');
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
    l = plot(At_sec, Alambda_deg_sensed - Alambda_deg_truth, ...
             At_sec, Alambda_deg_est_apriori - Alambda_deg_truth, ...
             At_sec, Amean_lambda_deg_est_apriori, ...
             At_sec, Alambda_deg_est - Alambda_deg_truth, ...
             At_sec, Amean_lambda_deg_est); hold on;    
            %t_sec,  - sigma_lambda_deg_est_apriori, ...
             %t_sec,  sigma_lambda_deg_est_apriori, ...
             %t_sec,  - sigma_lambda_deg_est, ...
             %t_sec,  sigma_lambda_deg_est, ...

    xlabel('t [sec]'); 
    legend('sensed error', ...
           'priori error', 'priori error mean', ...
           'poster error', 'poster error mean', 'Location', 'northwest');
    %   legend('sensed error', ...
    %       'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
    %       'poster error', 'poster std -', 'poster std +', 'poster error mean');
    ylabel('Error \lambda [deg]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'k';
    %l(3).Color = 'k';
    %l(3).LineStyle = '--';
    %l(4).Color = 'k';
    %l(4).LineStyle = '--';
    l(3).Color = 'k';
    l(3).LineStyle = ':';
    l(4).Color = 'b';
    %l(7).Color = 'b';
    %l(7).LineStyle = '--';
    %l(8).Color = 'b';
    %l(8).LineStyle = '--';
    l(5).Color = 'b';
    l(5).LineStyle = ':';
    ax2.YColor = 'k';
    ax2.XMinorGrid = 'on';
    ax2.YMinorGrid = 'on';
    grid on;

    ax4 = subplot('Position',[0.55 0.38 0.43 0.27]);
    %ax4 = subplot(3,2,4);
    l = plot(At_sec, Aphi_deg_sensed - Aphi_deg_truth, ...
             At_sec, Aphi_deg_est_apriori - Aphi_deg_truth, ...
             At_sec, Amean_phi_deg_est_apriori, ...
             At_sec, Aphi_deg_est - Aphi_deg_truth, ...
             At_sec, Amean_phi_deg_est); hold on;    
         % At_sec, - Asigma_phi_deg_est_apriori, ...
            % At_sec, Asigma_phi_deg_est_apriori, ...
            % At_sec, - Asigma_phi_deg_est, ...
            % At_sec, Asigma_phi_deg_est, ...
    legend('sensed error', ...
           'priori error', 'priori error mean', ...
           'poster error', 'poster error mean', 'Location', 'northwest');

    %legend('sensed error', ...
    %       'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
    %       'poster error', 'poster std -', 'poster std +', 'poster error mean');
    xlabel('t [sec]'); 
    ylabel('Error \phi [deg]');
    l(1).Color = 'g';
    l(1).Marker = 'x'; 
    l(1).LineStyle = 'none'; 
    l(1).MarkerSize = 6;  
    l(2).Color = 'k';
    %l(3).Color = 'k';
    %l(3).LineStyle = '--';
    %l(4).Color = 'k';
    %l(4).LineStyle = '--';
    l(3).Color = 'k';
    l(3).LineStyle = ':';
    l(4).Color = 'b';
    %l(7).Color = 'b';
    %l(7).LineStyle = '--';
    %l(8).Color = 'b';
    %l(8).LineStyle = '--';
    l(5).Color = 'b';
    l(5).LineStyle = ':';
    ax4.YColor = 'k';
    ax4.XMinorGrid = 'on';
    ax4.YMinorGrid = 'on';
    grid on;

    ax6 = subplot('Position',[0.55 0.05 0.43 0.27]);
    %ax6 = subplot(3,2,6);
    l = plot(At_sec, Ah_m_sensed - Ah_m_truth, ...
             At_sec, Ah_m_est_apriori - Ah_m_truth, ...
             At_sec, - Asigma_h_m_est_apriori, ...
             At_sec, Asigma_h_m_est_apriori, ...
             At_sec, Amean_h_m_est_apriori, ...
             At_sec, Ah_m_est - Ah_m_truth, ...
             At_sec, - Asigma_h_m_est, ...
             At_sec, Asigma_h_m_est, ...
             At_sec, Amean_h_m_est); hold on;
    legend('sensed error', ...
           'priori error', 'priori std -', 'priori std +', 'priori error mean', ...
           'poster error', 'poster std -', 'poster std +', 'poster error mean');     
    xlabel('t [sec]'); 
    ylabel('Error h [m]');
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
    






