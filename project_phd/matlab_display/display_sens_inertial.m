function display_sens_inertial(folder_name, varargin)

fileID = fopen(['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/sens_inertial.txt']);
fgetl(fileID);
uavSensors = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','MultipleDelimsAsOne',1);
fclose(fileID);
plot_name = ['/data/PHD/outputs/nav/' folder_name(1:2) '/' folder_name '/plot_sens_inertial'];

t_sec               = uavSensors{1};
w_ibb_dps_in(:,1)   = uavSensors{2};
w_ibb_dps_in(:,2)   = uavSensors{3};
w_ibb_dps_in(:,3)   = uavSensors{4};
w_ibb_dps_out(:,1)  = uavSensors{5};
w_ibb_dps_out(:,2)  = uavSensors{6};
w_ibb_dps_out(:,3)  = uavSensors{7};
E_gyr_dps(:,1)      = uavSensors{8};
E_gyr_dps(:,2)      = uavSensors{9};
E_gyr_dps(:,3)      = uavSensors{10};
f_ibb_mps2_in(:,1)  = uavSensors{11};
f_ibb_mps2_in(:,2)  = uavSensors{12};
f_ibb_mps2_in(:,3)  = uavSensors{13};
f_ibb_mps2_out(:,1) = uavSensors{14};
f_ibb_mps2_out(:,2) = uavSensors{15};
f_ibb_mps2_out(:,3) = uavSensors{16};
E_acc_mps2(:,1)     = uavSensors{17};
E_acc_mps2(:,2)     = uavSensors{18};
E_acc_mps2(:,3)     = uavSensors{19};
clear uavSensors;

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
t_sec          = t_sec(I);
w_ibb_dps_in   = w_ibb_dps_in(I,:);
w_ibb_dps_out  = w_ibb_dps_out(I,:);
E_gyr_dps      = E_gyr_dps(I,:);
f_ibb_mps2_in  = f_ibb_mps2_in(I,:); 
f_ibb_mps2_out = f_ibb_mps2_out(I,:);
E_acc_mps2     = E_acc_mps2(I,:);

h = figure('units','normalized','position',[0 0.15 1.0 0.85]);

ax1 = subplot('Position',[0.05 0.71 0.43 0.27]);
[hAx,hLine1,hLine2] = plotyy(t_sec, [w_ibb_dps_out(:,1) w_ibb_dps_in(:,1)], ...
                             t_sec, [(w_ibb_dps_out(:,1) - w_ibb_dps_in(:,1)) E_gyr_dps(:,1)]); hold on;
xlabel('t [sec]');
ylabel(hAx(1),'\omega_{IB,1}^{B} [dps]');
ylabel(hAx(2),'Error \omega_{IB,1}^{B} [dps]');

hLine1(1).Color = 'g';       
hLine1(1).Marker = 'x'; 
hLine1(1).LineStyle = 'none'; 
hLine1(1).MarkerSize = 6;  

hLine1(2).Color = 'k';       
hLine1(2).LineStyle = '-'; 
hLine1(2).LineWidth = 2.;  

hLine2(1).Color = 'r';     
hLine2(1).Marker = '.'; 
hLine2(1).LineStyle = 'none'; 
hLine2(1).MarkerSize = 1; 

hLine2(2).Color = 'b';
hLine2(2).LineStyle = '-'; 
hLine2(2).LineWidth = 2.;  

hAx(1).YColor = 'k';
hAx(2).YColor = 'r';
hAx(1).XMinorGrid = 'on';
hAx(1).YMinorGrid = 'on';
%hAx(2).YMinorGrid = 'on';
legend(hAx(1), 'Sensed', 'Truth', 'Error (sens - truth)', 'Error w/o noise');
grid on;    

ax2 = subplot('Position',[0.05 0.38 0.43 0.27]);
[jAx,jLine1,jLine2] = plotyy(t_sec, [w_ibb_dps_out(:,2) w_ibb_dps_in(:,2)], ...
                             t_sec, [(w_ibb_dps_out(:,2) - w_ibb_dps_in(:,2)) E_gyr_dps(:,2)]); hold on;
xlabel('t [sec]');
ylabel(jAx(1),'\omega_{IB,2}^{B} [dps]');
ylabel(jAx(2),'Error \omega_{IB,2}^{B} [dps]');

jLine1(1).Color = 'g';       
jLine1(1).Marker = 'x'; 
jLine1(1).LineStyle = 'none'; 
jLine1(1).MarkerSize = 6;  

jLine1(2).Color = 'k';       
jLine1(2).LineStyle = '-'; 
jLine1(2).LineWidth = 2.;  

jLine2(1).Color = 'r';     
jLine2(1).Marker = '.'; 
jLine2(1).LineStyle = 'none'; 
jLine2(1).MarkerSize = 1; 

jLine2(2).Color = 'b';
jLine2(2).LineStyle = '-'; 
jLine2(2).LineWidth = 2.;  

jAx(1).YColor = 'k';
jAx(2).YColor = 'r';
jAx(1).XMinorGrid = 'on';
jAx(1).YMinorGrid = 'on';
%jAx(2).YMinorGrid = 'on';
grid on;    

ax3 = subplot('Position',[0.05 0.05 0.43 0.27]);
[kAx,kLine1,kLine2] = plotyy(t_sec, [w_ibb_dps_out(:,3) w_ibb_dps_in(:,3)], ...
                             t_sec, [(w_ibb_dps_out(:,3) - w_ibb_dps_in(:,3)) E_gyr_dps(:,3)]); hold on;
xlabel('t [sec]');
ylabel(kAx(1),'\omega_{IB,3}^{B} [dps]');
ylabel(kAx(2),'Error \omega_{IB,3}^{B} [dps]');

kLine1(1).Color = 'g';       
kLine1(1).Marker = 'x'; 
kLine1(1).LineStyle = 'none'; 
kLine1(1).MarkerSize = 6;  

kLine1(2).Color = 'k';       
kLine1(2).LineStyle = '-'; 
kLine1(2).LineWidth = 2.;  

kLine2(1).Color = 'r';     
kLine2(1).Marker = '.'; 
kLine2(1).LineStyle = 'none'; 
kLine2(1).MarkerSize = 1; 

kLine2(2).Color = 'b';
kLine2(2).LineStyle = '-'; 
kLine2(2).LineWidth = 2.;  

kAx(1).YColor = 'k';
kAx(2).YColor = 'r';
kAx(1).XMinorGrid = 'on';
kAx(1).YMinorGrid = 'on';
%kAx(2).YMinorGrid = 'on';
grid on;    

Aax1 = subplot('Position',[0.55 0.71 0.43 0.27]);
[AhAx,AhLine1,AhLine2] = plotyy(t_sec, [f_ibb_mps2_out(:,1) f_ibb_mps2_in(:,1)], ...
                                t_sec, [(f_ibb_mps2_out(:,1) - f_ibb_mps2_in(:,1)) E_acc_mps2(:,1)]); hold on;
xlabel('t [sec]');
ylabel(AhAx(1),'f_{IB,1}^{B} [mps2]');
ylabel(AhAx(2),'Error f_{IB,1}^{B} [mps2]');

AhLine1(1).Color = 'g';       
AhLine1(1).Marker = 'x'; 
AhLine1(1).LineStyle = 'none'; 
AhLine1(1).MarkerSize = 6;  

AhLine1(2).Color = 'k';       
AhLine1(2).LineStyle = '-'; 
AhLine1(2).LineWidth = 2.;  

AhLine2(1).Color = 'r';     
AhLine2(1).Marker = '.'; 
AhLine2(1).LineStyle = 'none'; 
AhLine2(1).MarkerSize = 1; 

AhLine2(2).Color = 'b';
AhLine2(2).LineStyle = '-'; 
AhLine2(2).LineWidth = 2.;  

AhAx(1).YColor = 'k';
AhAx(2).YColor = 'r';
AhAx(1).XMinorGrid = 'on';
AhAx(1).YMinorGrid = 'on';
%AhAx(2).YMinorGrid = 'on';
grid on;    

Aax2 = subplot('Position',[0.55 0.38 0.43 0.27]);
[AjAx,AjLine1,AjLine2] = plotyy(t_sec, [f_ibb_mps2_out(:,2) f_ibb_mps2_in(:,2)], ...
                                t_sec, [(f_ibb_mps2_out(:,2) - f_ibb_mps2_in(:,2)) E_acc_mps2(:,2)]); hold on;
xlabel('t [sec]');
ylabel(AjAx(1),'f_{IB,2}^{B} [mps2]');
ylabel(AjAx(2),'Error f_{IB,2}^{B} [mps2]');

AjLine1(1).Color = 'g';       
AjLine1(1).Marker = 'x'; 
AjLine1(1).LineStyle = 'none'; 
AjLine1(1).MarkerSize = 6;  

AjLine1(2).Color = 'k';       
AjLine1(2).LineStyle = '-'; 
AjLine1(2).LineWidth = 2.;  

AjLine2(1).Color = 'r';     
AjLine2(1).Marker = '.'; 
AjLine2(1).LineStyle = 'none'; 
AjLine2(1).MarkerSize = 1; 

AjLine2(2).Color = 'b';
AjLine2(2).LineStyle = '-'; 
AjLine2(2).LineWidth = 2.;  

AjAx(1).YColor = 'k';
AjAx(2).YColor = 'r';
AjAx(1).XMinorGrid = 'on';
AjAx(1).YMinorGrid = 'on';
%AjAx(2).YMinorGrid = 'on';
grid on;   


Aax3 = subplot('Position',[0.55 0.05 0.43 0.27]);
[AkAx,AkLine1,AkLine2] = plotyy(t_sec, [f_ibb_mps2_out(:,3) f_ibb_mps2_in(:,3)], ...
                                t_sec, [(f_ibb_mps2_out(:,3) - f_ibb_mps2_in(:,3)) E_acc_mps2(:,3)]); hold on;
xlabel('t [sec]');
ylabel(AkAx(1),'f_{IB,3}^{B} [mps2]');
ylabel(AkAx(2),'Error f_{IB,3}^{B} [mps2]');

AkLine1(1).Color = 'g';       
AkLine1(1).Marker = 'x'; 
AkLine1(1).LineStyle = 'none'; 
AkLine1(1).MarkerSize = 6;  

AkLine1(2).Color = 'k';       
AkLine1(2).LineStyle = '-'; 
AkLine1(2).LineWidth = 2.;  

AkLine2(1).Color = 'r';     
AkLine2(1).Marker = '.'; 
AkLine2(1).LineStyle = 'none'; 
AkLine2(1).MarkerSize = 1; 

AkLine2(2).Color = 'b';
AkLine2(2).LineStyle = '-'; 
AkLine2(2).LineWidth = 2.;  

AkAx(1).YColor = 'k';
AkAx(2).YColor = 'r';
AkAx(1).XMinorGrid = 'on';
AkAx(1).YMinorGrid = 'on';
%AkAx(2).YMinorGrid = 'on';
grid on;   

%savefig([plot_name '.fig']);

%Image = getframe(gcf);
%imwrite(Image.cdata, [plot_name '.jpg']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    