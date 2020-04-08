function display_debug()

fileID = fopen('/home/egallo/utm/trunk/phd/outputs/flight_test/debug.txt');
uavTruth = textscan(fileID, '%f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
%plot_name = 'plot_uavTruth_theta_vtas';

t_sec            = uavTruth{1};
vtas_err_mps     = uavTruth{2};
vtas_err_lpf_mps = uavTruth{3};
clear uavTruth;

h = figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot(2,2,1);
l = plot(t_sec, [vtas_err_mps vtas_err_lpf_mps]); hold on;
xlabel('t [sec]'); 
%ylabel(hAx(1),'\theta [deg]');
%ylabel(hAx(2),'\delta_{E} [deg]');
l(1).Color = 'k';
%hLine1(1).LineWidth = 1.5;
l(2).Color = 'b';
%hLine1(2).LineStyle = '-.';
%hLine1(2).LineWidth = 1.5;
%hLine2.Color = 'b';
%hLine2.LineWidth = 1.5;
%hAx(1).YColor = 'k';
%hAx(2).YColor = 'b';
%hAx(1).XMinorGrid = 'on';
%hAx(1).YMinorGrid = 'on';
%hAx(2).YMinorGrid = 'on';
grid on;

ax2 = subplot(2,2,2);
grid on;

ax4 = subplot(2,2,3);
grid on;

ax3 = subplot(2,2,4);
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    