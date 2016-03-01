function data = UM_agility(IMU,CAL)
% This function was written by Ryan McGinnis (ryanmcg@umich.edu) with input
% from Stephen Cain, Steven Davidson, Rachel Vitali, Scott McLean and Noel Perkins as a 
% deliverable for contract W911QY-13-C-0011.
% UM_agility is used to analyze data from a subject negotiating balance beam obstacle
% collected with ADPM Opal sensors as described in the Final Report and 
% User Manual submitted as part of the contract.
% Function inputs and outputs are described in the User Manual.

%{
Last Update by Chika Eke: 22 Feb 2016

Update1: Updated length of path from ~5m to ~2m,Added mlROM, apROM, and resultsAgility
array near line 203. **Commented Out! 
Update2: set 'dev' = sacrum  & did find/replace so that all 'sacrum' results became 
% 'sacrum'for M/L ROM and A/P ROM calcs. ***CHANGED BACK!!
Update 3:Commented out the section of the
code checking to see if subject was standing still when calculating ML ROM using side bend
(since did straight walking right before side bend):lines 88-157. **CHANGED
BACK!
Update 4: added 2 lines for segCoord function. **Commented out!
Update 5: Change xs input from graphical to input number.
%}

% Clear screen
clc

if exist('IMU','var') && exist('CAL','var')
else
    %Load mat file containing IMU structure
    %Expects: IMU.sacrum and IMU.trigger
    [filename,filepath] = uigetfile('*.mat','Select data file');
    load(strcat(filepath,filename));

    %Load mat file containing calibration matrices
    filenameCAL = [filename(1:(end-4)), '_CAL.mat'];
    load(strcat(filepath,filenameCAL));
end

%Filter kinematic data and assign to variables expected below
fs = 1/mean(diff(IMU.time)); % sampling frequency
filter_order = 4;   % 2nd order filter
cut_off = 15; % cutoff frequency (Hz)
[num,den] = butter(filter_order,cut_off/(fs/2),'low');
            
b.sacrum = IMU.trigger.button;
t.sacrum = IMU.time;
a.sacrum = filtfilt(num,den,IMU.sacrum.a * CAL.sacrum.R.'); %m/s^2
w.sacrum = filtfilt(num,den,IMU.sacrum.w * CAL.sacrum.R.'); %rad/s

%Find upslope of button pushes to distinguish start and end of obstacles
ind_b = diff(b.sacrum) == 1; ind_b = [ind_b; false];
t_b = t.sacrum(ind_b);
indd_tap = diff(t_b)<0.25; indd_tap = [false; indd_tap];
t_b = t_b(~indd_tap);
 
figure;
subplot(211)
hold on;
plot(t.sacrum,sqrt(sum(a.sacrum.^2,2)));
plot(t.sacrum,b.sacrum*max(sqrt(sum(a.sacrum.^2,2))),'k');
xlabel('time'); ylabel('accel (m/s^2)');

subplot(212)
hold on;
plot(t.sacrum,sqrt(sum(w.sacrum.^2,2)));
plot(t.sacrum,b.sacrum*max(sqrt(sum(w.sacrum.^2,2))),'k');
xlabel('time'); ylabel('ang vel (rad/s)');
%-------------------------------------------------------------------------%


%Select agility run to process--------------------------------------------%
%This section will eventually be replaced by the code that David is creating
%Zoom to and select agility run section 
%set(gcf,'name','zoom to agility run and hit a key');
%pause;
%[xs,~] = ginput(1);

xs = 732; %manually define start time

overhang = 2;
A(1) = t_b(find(t_b<xs,1,'last'))-overhang;
A(2) = t_b(find(t_b>xs,1,'first'));

obstacle_time = [A(1)+overhang, A(2)]

close(gcf);

%added by Chika for segment coordination calcs
%[turnLatency1] = segCoord('Pilot_F2.mat',A(1)+overhang,A(2),'head','sacrum');
%[turnLatency2] = segCoord('Pilot_F2.mat',A(1)+overhang,A(2),'sacrum','sacrum');
%-------------------------------------------------------------------------%

%Define device we're focusing on and define index array to truncate datafile
dev = 'sacrum';
ind = t.(dev)>A(1) & t.(dev)<A(2);

% %Find still section at least 10 pts long closest to start of time record
amag = sqrt(sum(a.(dev)(ind,:).^2,2));
wmag = sqrt(sum(w.(dev)(ind,:).^2,2));
t_j = t.(dev)(ind);

%establish bounds for still sensor data
a_lb = -3*CAL.sacrum.a_noise; %-3 * range of noise
a_ub = -a_lb; %3 * range of noise
w_ub = (CAL.sacrum.w0_mag + 3 * CAL.sacrum.w_noise); %3 * noise range above mean

%define index array for still points
ind_still = amag < a_ub+CAL.sacrum.g_mag & amag > a_lb+CAL.sacrum.g_mag & wmag < w_ub;

%establish initial end point of still section (before subject told to 'go')
i2 = find(ind_still==1 & t_j<=obstacle_time(1),1,'last');
if isempty(i2)
    error('no still section before agility run!\nTry increasing overhang variable');
end

%establish initial start point of still section
i1 = find(ind_still==0 & t_j<t_j(i2),1,'last')+1;
if isempty(i1)
    i1 = 1;
end

%if necessary, search for still section long enough for initialization
if i2-i1>10 %first still section is long enough
    xs = [t_j(i1), t_j(i2)];
else %need to find a longer still section
    try
        while i2-i1<10 %keep looking for still sections closer to the start of the trial 
            i2 = find(ind_still==1 & t_j<t_j(i1),1,'last');
            i1 = find(ind_still==0 & t_j<t_j(i2),1,'last')+1;
        end
        xs = [t_j(i1), t_j(i2)];
    catch
        figure;
        hold on;
        plot(t_j,amag);
        plot(t_j(ind_still),amag(ind_still),'.k');
        plot([t_j(1),t_j(end)],a_lb*[1,1]+CAL.sacrum.g_mag,'-k');
        plot([t_j(1),t_j(end)],a_ub*[1,1]+CAL.sacrum.g_mag,'-k');
        xlabel('time'); ylabel('amag');
        set(gcf,'name','zoom to still section at start of trial and hit a key');
        
        pause;
        [xs,~] = ginput(2);
        close(gcf);
    end
end

% %plot results so that still section can be confirmed visually
figure;
set(gcf,'name','check still section ID at start of trial');
subplot(211)
hold on;
plot(t_j,amag);
plot(t_j(ind_still),amag(ind_still),'.k');
plot(t_j(t_j>=xs(1)&t_j<=xs(2)),amag(t_j>=xs(1)&t_j<=xs(2)),'.g');
plot([t_j(1),t_j(end)],a_lb*[1,1]+CAL.sacrum.g_mag,'-k');
plot([t_j(1),t_j(end)],a_ub*[1,1]+CAL.sacrum.g_mag,'-k');
xlabel('time'); ylabel('amag');

subplot(212) 
hold on;
plot(t_j,wmag);
plot(t_j(ind_still),wmag(ind_still),'.k');
plot(t_j(t_j>=xs(1)&t_j<=xs(2)),wmag(t_j>=xs(1)&t_j<=xs(2)),'.g');
plot([t_j(1),t_j(end)],w_ub*[1,1],'-k');
xlabel('time'); ylabel('wmag');
   
%Calculate orientation
ind = t.(dev)>xs(1) & t.(dev)<A(2)
params.a_noise = CAL.sacrum.a_noise;
params.w_noise = CAL.sacrum.w_noise;
params.g_mag = CAL.sacrum.g_mag;
params.w0_mag = CAL.sacrum.w0_mag;
params.indg = t.(dev)(ind) < xs(2);
[t_j,q,~,~,params] = orientation(t.(dev)(ind),a.(dev)(ind,:),w.(dev)(ind,:)*180/pi,params);

%Resolve kinematic variables in world frame
a_j = quaternRot(q,a.(dev)(ind,:)) - (t_j.^0)*[0,0,params.g_mag];
w_j = quaternRot(q,w.(dev)(ind,:));

%Calculate quantities needed to define performance metrics
%sensor-fixed vertical and A-P directions in world frame
vert = quaternRot(q, [0,0,1]); %+z is up when standing still
ant = quaternRot(q,[0,1,0]); %+y points forward when standing still
        
%anterior-posterior (pelvis) accel
ant_h = [ant(:,1)./sqrt(sum(ant(:,1:2).^2,2)), ant(:,2)./sqrt(sum(ant(:,1:2).^2,2))];
a_apT = dot(a_j(:,1:2),ant_h,2); %a-p acceleration

%medio-lateral (pelvis) accel
ml_h = cross(ant,(ant(:,1).^0)*[0,0,1]);
a_mlT = dot(a_j(:,1:2),ml_h(:,1:2),2); %m-l acceleration

%tilt angle - angle from vertical
tilt = real(acosd(dot(vert,(vert(:,1).^0)*[0,0,1],2)));

%resolve tilt angle into AP and ML components
th = vert - dot(vert,(vert(:,1).^0)*[0,0,1],2)*[0,0,1]; %horizontal projection of sensor z axis
tap = real(atand(dot(th(:,1:2),ant_h,2)./dot(vert,(vert(:,1).^0)*[0,0,1],2))); %a-p tilt
tml = real(atand(dot(th,ml_h,2)./dot(vert,(vert(:,1).^0)*[0,0,1],2))); %m-l tilt

%tilt rom
rom = max(tilt)-min(tilt);

%azimuthal angle
aang = ang_scan(real(atan2d(ant_h(:,1),ant_h(:,2)))); 
aang = aang - aang(1);

%filter out noise in kinematic data due to foot falls
t_total = obstacle_time(2)-obstacle_time(1);
fs = 1/mean(diff(t_j)); % sampling frequency
filter_order = 2;   % 4th order filter
cut_off = 1.5*(5/t_total); % cutoff frequency (Hz) - defined based on number of turns and time to complete run
[num,den] = butter(filter_order,cut_off/(fs/2),'low');

a_mlF = filtfilt(num,den,a_mlT); %m-l acceleration
t_mlF = filtfilt(num,den,tml); %m-l tilt
a_apF = filtfilt(num,den,a_apT); %a-p acceleration
t_apF = filtfilt(num,den,tap); %a-p tilt
w_zF = filtfilt(num,den,w_j(:,3)); %angular velocity

%Added by Chika for data analysis
% mlROM = max(t_mlF)- min(t_mlF); %MlROM is basically max lean (difference is max lean - zero)
% apROM = max(t_apF)- min(t_apF); 
% resultsAgility = [t_total mlROM apROM]';

%Find turns based on ML acceleration and ML jerk
%first turn (L): a_mlF has a neg peak, jerk (-)->(+)
%second turn (R): a_mlF has a pos peak, jerk (+)->(-)
%third turn (L): a_mlF has a neg peak, jerk (-)->(+)
%fourth turn (R): a_mlF has a pos peak, jerk (+)->(-)
%fifth turn (L): a_mlF has a neg peak, jerk (-)->(+)

%center ml acceleration
c = [t_j.^0, t_j] \ a_mlF;
fit_c = [t_j.^0, t_j]*c;
a_mlFc = a_mlF-fit_c;

%define jerk and jerk sign variables
j_mlF = [0; diff(a_mlFc)./diff(t_j)];
sj_mlF = [0; diff(sign(j_mlF))]; %change in sign of jerk

%find turns - adjusting threshold if not enough turns found
a_range = max(a_mlFc)-min(a_mlFc);
lefts = 0;
rights = 0;
j=0;
while length(lefts)<3 || length(rights)<2
    thresh = a_range/(j+4);
    lefts = find(sj_mlF==2 & a_mlFc<-thresh & t_j>overhang+0.5);
    rights = find(sj_mlF==-2 & a_mlFc>thresh & t_j>overhang+0.5);
    j=j+1;
end

figure;
set(gcf,'name','check turn ID: left = blue, right = red');
hold on;
plotyy(t_j,a_mlF,t_j,j_mlF);
plot(t_j(lefts),a_mlF(lefts),'ob','MarkerFaceColor','b')
plot(t_j(rights),a_mlF(rights),'or','MarkerFaceColor','r')
xlabel('time'); ylabel('a');

t1 = lefts(1);
t2 = rights(find(rights>t1 & t_j(rights)>0.5+t_j(t1),1,'first'));
t3 = lefts(find(lefts>t2 & t_j(lefts)>0.5+t_j(t2),1,'first'));
t4 = rights(find(rights>t3 & t_j(rights)>0.5+t_j(t3),1,'first'));
t5 = lefts(find(lefts>t4 & t_j(lefts)>0.5+t_j(t4),1,'first'));

ind_turn = [t1; t2; t3; t4; t5];

%Calculate corrected velocity and displacement of sacrum using cone
%locations to define drift correction

%uncorrect velocity and displacement
v_j = cumtrapz(t_j,a_j); %m/s
d_j = cumtrapz(t_j,v_j); %m

%define cone locations 
c = zeros(7,3);
seg = 2; %15 ft in m
course_dir = -1; %-1 if 2nd cone is left of 1st cone
c(2,:) = [0,seg,0];
c(3,:) = [course_dir*sqrt(seg^2 - (seg/2)^2),1.5*seg,0];
c(4,:) = [0,2*seg,0];
c(5,:) = [course_dir*sqrt(seg^2 - (seg/2)^2),2.5*seg,0];
c(6,:) = [0,3*seg,0];
c(7,:) = [0,4*seg,0];

%shortest path through obstacle
dc = interp1([t_j(1); t_j(ind_turn); t_j(end)],c,t_j,'linear');

%define initial, nominal heading angle
a0 = mean(a_j(1:round(0.5*ind_turn(1)),1:2));
a0 = a0./norm(a0);
theta0 = -atan2(a0(1),a0(2));

%define and apply drift correction finding optimal heading correction
ind_still = find(params.indg);
f = @(x)vd_drift_correct(x,t_j,v_j,d_j,ind_turn,ind_still,c);

x0 = theta0;
options = optimset('maxfunevals',10000,'maxiter',1000,'tolfun',1e-8,...
    'tolx',1e-8,'plotfcns',{@optimplotx,@optimplotresnorm});
lb = theta0-pi/2;
ub = theta0+pi/2;

theta = lsqnonlin(f,x0,lb,ub,options);

[vrc,drc] = apply_drift_correct(theta,t_j,v_j,d_j,ind_turn,ind_still,c);

%Rotate other data 
R = [cos(theta), sin(theta), 0;
    -sin(theta), cos(theta), 0;
    0,      0, 1];

dr = d_j * R.';
vr = v_j * R.';
ar = a_j * R.';
vertr = vert * R.';
de = dr - dc;
arf = filtfilt(num,den,ar);

%plot results as a sanity check
figure;
set(gcf,'name','filtered accel in lab frame');
hold on;
plot(t_j,arf(:,1:2))
plot(t_j(ind_turn),arf(ind_turn,1:2),'ok','markerfacecolor','k');
xlabel('time (s)','fontsize',16);
ylabel('acceleration (m/s^2)','fontsize',16);

figure;
set(gcf,'name','velocity drift error trajectories');
hold on;
plot(t_j(ind_turn),vr(ind_turn,1:2)-vrc(ind_turn,1:2),'ok','markerfacecolor','k');
plot(t_j,vr(:,1:2)-vrc(:,1:2));
xlabel('time (s)','fontsize',16);
ylabel('velocity drift (m/s)','fontsize',16);
set(gca,'fontsize',16);

figure;
set(gcf,'name','corrected velocity trajectories');
hold on;
plot(t_j,vrc(:,1:2))
plot(t_j(ind_turn),vrc(ind_turn,1:2),'ok','markerfacecolor','k');
xlabel('time (s)','fontsize',16);
ylabel('velocity (m/s)','fontsize',16);
set(gca,'fontsize',16);

figure;
set(gcf,'name','corrected displacement trajectories');
hold on;
% plot(dc);
plot([t_j(ind_turn); t_j(end)],[dc(ind_turn,1:2); dc(end,1:2)],'ok','markerfacecolor','k');
plot(t_j,drc(:,1:2))
xlabel('time (s)','fontsize',16);
ylabel('displacement(m)','fontsize',16);
set(gca,'fontsize',16);

figure;
set(gcf,'name','displacement drift error trajectories');
hold on;
plot([t_j(ind_turn); t_j(end)],[de(ind_turn,1:2); de(end,1:2)],'ok','markerfacecolor','k');
plot(t_j,dr(:,1:2)-drc(:,1:2));
xlabel('time (s)','fontsize',16);
ylabel('displacement drift (m)','fontsize',16);
set(gca,'fontsize',16);

figure;
set(gcf,'name','Corrected path');
hold on; grid on;
plot3(c(:,1),c(:,2),c(:,3),'ok');
plot3(dc(:,1),dc(:,2),dc(:,3),'--b');
% plot3(dr(:,1),dr(:,2),dr(:,3),'m','linewidth',2);
% plot3(dr(ind_turn,1),dr(ind_turn,2),dr(ind_turn,3),'ok','markerfacecolor','k');
plot3(drc(:,1),drc(:,2),drc(:,3),'r','linewidth',2);
plot3(drc(ind_turn,1),drc(ind_turn,2),drc(ind_turn,3),'ok','markerfacecolor','k');
xlabel('X (m)','fontsize',16); ylabel('Y (m)','fontsize',16);
set(gca,'fontsize',16);

%Calculate metrics based on sacral trajectory
%speed
speed = sqrt(sum(vrc(:,1:2).^2,2)); %horizontal speed
speed_turn = speed(ind_turn); %horizontal

%tangential and normal accelerations and radius of curvature
ind_mov = speed > 0.05;
tang = [vrc(:,1)./speed, ...
    vrc(:,2)./speed, ...
    zeros(size(vrc(:,3)))];
tang_mov = tang(ind_mov,:);
ind_turn_mov = ind_turn-(find(ind_mov==1,1,'first')-1);

a_tang = dot(arf(ind_mov,:),tang_mov,2);
a_resid = [arf(ind_mov,1:2), zeros(size(arf(ind_mov,3)))]-...
    [a_tang.*tang_mov(:,1), a_tang.*tang_mov(:,2), a_tang.*tang_mov(:,3)];
norm_dir = [a_resid(:,1)./sqrt(sum(a_resid.^2,2)), ...
    a_resid(:,2)./sqrt(sum(a_resid.^2,2)),...
    a_resid(:,3)./sqrt(sum(a_resid.^2,2))];
a_norm = dot(arf(ind_mov,:),norm_dir,2);
rho = speed(ind_mov).^2 ./ a_norm;

a_tang_turn = a_tang(ind_turn_mov); %m/s^2
a_norm_turn = a_norm(ind_turn_mov); %m/s^2
rho_turn = rho(ind_turn_mov); %m

%tangential and normal tilt angle
vert_horiz = vertr(ind_mov,:) - dot(vertr(ind_mov,:),(vertr(ind_mov,1).^0)*[0,0,1],2)*[0,0,1];
t_norm = real(atand(dot(vert_horiz,norm_dir,2)./vertr(ind_mov,3)));
t_tang = real(atand(dot(vert_horiz,tang_mov,2)./vertr(ind_mov,3)));

%ap and path azimuthal angles 
ap_dir = [ant_h, zeros(size(ant_h(:,1)))] * R.';
ang_ap = acosd(dot(ap_dir(ind_mov,:),(ap_dir(ind_mov,1).^0)*[1,0,0],2));
ang_tang = acosd(dot(tang_mov,(tang_mov(:,1).^0)*[1,0,0],2));

[fit_pX, gof_pX] = fit(ang_tang,ang_ap,'poly1');
[c,lags]=xcorr(ang_ap-mean(ang_ap),...
               ang_tang-mean(ang_tang), round(1.5/fs));
[~,indL] = max(c); dt = lags(indL) * (1/fs); %t2_new = t2 + dt; -> if dt>0 ang_tang lags ang_ap
                                             %                  -> if dt<0 ang_tang leads ang_ap
 
fit_ang = [fit_pX.p1, fit_pX.p2, gof_pX.rsquare, gof_pX.rmse];                                     
ang_fit_slope = fit_pX.p1;
ang_fit_intercept = fit_pX.p2;
ang_fit_r2 = gof_pX.rsquare;
ang_fit_rmse = gof_pX.rmse;
ang_tang_lag = dt;
ang_tang_rom = max(ang_tang)-min(ang_tang)
ang_ap_rom = max(ang_ap)-min(ang_ap)

%turn angles
if length(1:ind_turn_mov(1)) == 1
    v1 = tang_mov(1:ind_turn_mov(1),:);
else
    v1 = mean(tang_mov(1:ind_turn_mov(1),:));
end    
v1 = v1./norm(v1);
v2 = mean(tang_mov(ind_turn_mov(1):ind_turn_mov(2),:)); v2 = v2./norm(v2);
v3 = mean(tang_mov(ind_turn_mov(2):ind_turn_mov(3),:)); v3 = v3./norm(v3);
v4 = mean(tang_mov(ind_turn_mov(3):ind_turn_mov(4),:)); v4 = v4./norm(v4);
v5 = mean(tang_mov(ind_turn_mov(4):ind_turn_mov(5),:)); v5 = v5./norm(v5);
v6 = mean(tang_mov(ind_turn_mov(5):end,:)); v6 = v6./norm(v6);

ang_t1 = real(acosd(dot(v1,v2)));
ang_t2 = real(acosd(dot(v2,v3)));
ang_t3 = real(acosd(dot(v3,v4)));
ang_t4 = real(acosd(dot(v4,v5)));
ang_t5 = real(acosd(dot(v5,v6)));

turn_angle = [ang_t1, ang_t2, ang_t3, ang_t4, ang_t5];


figure;
set(gcf,'name','heading angle of ap (blue) and tang (red) direction trajectories');
hold on;
plot(t_j(ind_mov),ang_ap,'b');
plot(t_j(ind_mov),ang_tang,'r');
plot(t_j(ind_turn),ang_ap(ind_turn_mov),'ok','markerfacecolor','k');
xlabel('time (s)','fontsize',16);
ylabel('heading angle (deg)','fontsize',16);
set(gca,'fontsize',16);

figure;
set(gcf,'name','ap heading vs tang heading');
axis equal; grid on; hold on;
plot(ang_tang,ang_ap,'.b');
plot([min(ang_tang); max(ang_tang)],[min(ang_tang), 1; max(ang_tang), 1]*fit_ang(1:2).','k','linewidth',2);
plot([min(ang_tang); max(ang_tang)],[min(ang_tang), 1; max(ang_tang), 1]*fit_ang(1:2).'-fit_ang(4)*[1;1],'--k','linewidth',2);
plot([min(ang_tang); max(ang_tang)],[min(ang_tang), 1; max(ang_tang), 1]*fit_ang(1:2).'+fit_ang(4)*[1;1],'--k','linewidth',2);
xlabel('tangent heading (deg)','fontsize',16);
ylabel('A-P heading (deg)','fontsize',16);
set(gca,'fontsize',16);

figure;
set(gcf,'name','ap and tangential directions');
hold on;
plot(t_j(ind_mov),ap_dir(ind_mov,1:2))
plot(t_j(ind_mov),tang(ind_mov,1:2),'--')
plot(t_j(ind_turn),ap_dir(ind_turn,1:2),'ok','markerfacecolor','k');
xlabel('time (s)','fontsize',16);
ylabel('direction','fontsize',16);
set(gca,'fontsize',16);

figure;
set(gcf,'name','normal and tangential accelerations');
hold on; grid on;
plot(t_j(ind_mov),a_tang)
plot(t_j(ind_mov),a_norm,'r')
legend('tangential','normal');
plot(t_j(ind_turn),a_tang_turn,'ok','markerfacecolor','k');
plot(t_j(ind_turn),a_norm_turn,'ok','markerfacecolor','k');
xlabel('Time (s)','fontsize',16);
ylabel('Acceleration (m/s^2)','fontsize',16);
set(gca,'fontsize',16);

figure;
set(gcf,'name','horizontal speed');
hold on;
plot(t_j,speed)
plot(t_j(ind_turn),speed(ind_turn),'ok','markerfacecolor','k');
xlabel('time (s)','fontsize',16);
ylabel('horizontal speed (m/s)','fontsize',16);
set(gca,'fontsize',16);

%Save selected metrics
data.obstacle_time = A(2)-(A(1)+overhang);

data.a_ml_turn = a_mlF(ind_turn);
data.a_ap_turn = a_apF(ind_turn);
data.t_ap_turn = t_apF(ind_turn);
data.t_ml_turn = t_mlF(ind_turn);
data.w_z_turn = w_zF(ind_turn);

data.speed_turn = speed(ind_turn); %horizontal

data.a_tang_turn = a_tang(ind_turn_mov); %m/s^2
data.a_norm_turn = a_norm(ind_turn_mov); %m/s^2
data.t_norm_turn = t_norm(ind_turn_mov);
data.t_tang_turn = t_tang(ind_turn_mov);
data.rho_turn = rho(ind_turn_mov); %m

data.ang_fit_slope = fit_pX.p1; %ang_tang = x, ang_ap = y
data.ang_fit_intercept = fit_pX.p2;
data.ang_fit_r2 = gof_pX.rsquare;
data.ang_fit_rmse = gof_pX.rmse;
data.ang_tang_lag = dt;
data.ang_tang_rom = max(ang_tang)-min(ang_tang);
data.ang_ap_rom = max(ang_ap)-min(ang_ap);

data.turn_angle = [ang_t1, ang_t2, ang_t3, ang_t4, ang_t5];

%save selected trajectories 
data.raw.arf = arf;
data.raw.vrc = vrc;
data.raw.drc = drc;
data.raw.a_mlF = a_mlF;
data.raw.t_mlF = t_mlF;
data.raw.a_apF = a_apF;
data.raw.t_apF = t_apF;
data.raw.w_zF = w_zF;
data.raw.ang_tang = ang_tang;
data.raw.ang_ap = ang_ap;
data.raw.a_tang = a_tang;
data.raw.a_norm = a_norm;
data.raw.speed = speed;
data.raw.rho_turn = rho_turn;
data.raw.t_norm = t_norm;
data.raw.t_tang = t_tang;
data.raw.time = t_j;
data.raw.time_mov = t_j(ind_mov);

end


function qConj = quaternConj(q)
%Function to calculate conjugate of quaternion
%Inputs:
%1. q - input quaternion (nx4)

%Outputs:
%1. qConj - quaternion conjugate of q

qConj = [q(:,1) -q(:,2) -q(:,3) -q(:,4)];

end

function ab = quaternProd(a,b)
%Function to take quaternion product of a x b
%Inputs:
%1. a - first quaternion (nx4)
%2. b - second quaternion (nx4)

%Outputs:
%1. ab - quanternion product of a x b (nx4)

ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);

end

function xp = quaternRot(q,x)
%Function to apply quaternion rotation
%Inputs:
%1. q - quaternion (nx4) defining rotation to be applied
%2. x - 3-element vector (nx3) to be transformed by quaternion

%Outputs:
%1. xp - transformed vector (nx3)

%Pad x with column of zeros (quaternion format)
x = [zeros(size(x,1),1), x];

%Account for case where x=(1x3), q=(nx4)
if size(x,1)==1 && size(q,1)~=1
    x = ones(size(q,1),1) * x;
end

%Apply rotation
xt = quaternProd(q, quaternProd(x, quaternConj(q)));

%Extract rotated vector
xp = xt(:,2:4);
end

function xnew = ang_scan(x)
%Function to fix jumps in angle data, expects angle data in degrees

%Calc change in angle from sample to sample
dx = diff(x);

%Identify if there are jumps in angle data greater than 180 deg
jps = find(abs(dx)>180);

if isempty(jps) %No jumps
    xnew = x;
else %Jumps
    xnew = x;
    jp_vals = dx(jps);
    i=1;
    jump_cum = 0;
    while i<=length(jp_vals)
        jump_start = jps(i)+1;
        jump_dir = sign(jp_vals(i));
        try
            jump_end = jps(i+1);
            xnew(jump_start:jump_end) = x(jump_start:jump_end)-jump_dir*360+jump_cum;
            i = i + 1;
            jump_cum = jump_cum-jump_dir*360;
        catch
            jump_end = length(x);
            xnew(jump_start:jump_end) = x(jump_start:jump_end)-jump_dir*360+jump_cum;
            i = length(jp_vals)+1;
            jump_cum = jump_cum-jump_dir*360;
            xnew(jump_end:end) = x(jump_end:end)+jump_cum;
        end
    end
end



end

function [time,q,a,w,params] = orientation(time,a,w,params)
%Function to determine orientation of IMU.  Does so in two steps: 1) Define
%initial orientation of device based on direction of gravity and 2) Define
%orientation thereafter by fusing acceleration and angular velocity
%estimates
%
%Inputs:
%1. acceleration (a (g), nx3)
%2. angular velocity (w (deg/s), nx3)
%3. time (time, nx1)
%
%Outputs (m<n):
%1. time (time, mx1)
%2. DCM describing IMU orientation (R, mx4)
%3. acceleration (a (g), mx3)
%4. angular velocity (w (deg/s), mx3)



%1. Define initial orientation of device ---------------------------------%

%Define noise characteristics of signals
ind = params.indg;
amag = sqrt(sum(a.^2,2));
wmag = sqrt(sum(w.^2,2));

try
    a_noise = params.a_noise;
    w_noise = params.w_noise*180/pi;
    params.w0_mag = params.w0_mag*180/pi; %deg/s
catch
    a_noise = std(amag(ind));
    w_noise = std(wmag(ind)); %deg/s
    params.a_noise = a_noise;
    params.w_noise = w_noise;
    params.g_mag = mean(amag(ind));
    params.w0_mag = mean(wmag(ind)); %deg/s
end
a_lb = -3*a_noise; %-3 * sd of noise
a_ub = -a_lb; %3 * sd of noise
w_ub = (params.w0_mag + 3 * w_noise)*pi/180; %3 * sd above mean

params.ind_still = amag < a_ub+params.g_mag & amag > a_lb+params.g_mag & wmag < w_ub*180/pi;

%Define initial Z direction (gravity)
Z = mean(a(ind,:)) ./ norm(mean(a(ind,:)));

%Define remaining axes 
%X in terms of (i,j,k) according to X = j x Z
X = cross([0,0,1],Z); X = X./norm(X);

%Re-define Y in terms of (i,j,k) to ensure orthogonality (Y = Z x X)
Y = cross(Z,X); Y = Y./norm(Y);


%Define initial DCM    
R = [X; Y; Z];


%2. Determine orientation as a function of time and report as DCM --------%
%   at each time step.

%Define nominal filter gains
Kp0 = 2;
Ki0 = 1;

%Initialize variables
Rinf = zeros(3,3,length(time)); Rinf(:,:,1) = R;
dc = zeros(size(time,1),3);

%Convert angular velocity to rad/s and remove bias
% w = (w-(w(:,1).^0)*mean(w(ind,:))) * pi/180;
w = w * pi/180;
wh = w;

%Define orientation throughout trial
tm = 0;
t0 = 1;
motion = zeros(size(w(:,1)));
X0 = X;
for t=2:length(time)
    
    %Propegate orientation
    dt = time(t) - time(t-1);
    wt = w(t,:) - dc(t-1,:);
    tt = 0.5 * dt * (wh(t-1,:) + wt);
    Rt = Rinf(:,:,t-1)*(eye(3)+(2/(1+0.5*(tt*tt.')))*...
        (0.5*skew(tt) + 0.25*skew(tt)^2));
    
    %Assign Gain Values
    amag = norm(a(t,:));
    wmag = norm(wh(t,:));
    if amag < a_ub+params.g_mag && amag > a_lb+params.g_mag && wmag < w_ub
        Ki = Ki0;
        Kp = Kp0;
        tm = 0;
    elseif amag < a_ub+params.g_mag && amag > a_lb+params.g_mag && wmag < 5*w_ub
        Ki = Ki0/10;
        Kp = Kp0/10;
        tm = 0;
    else
        Ki = 0;
        Kp = 0;
        tm = tm + 1;
    end
    motion(t) = tm;
       
    
    %Estimate error relative to gravity and azimuth
    if motion(t)-motion(t-1)<0 %return to still section
        X0 = Rt(1,:); t0 = t;
    else
        X0 = ((t-1-t0)/(t-t0))*X0 + (1/(t-t0))*Rinf(1,:,t-1); %weighted average of previous still points
    end
        
    at = a(t,:)./norm(a(t,:));
    we = cross(at, Rt(3,:)) + cross(X0, Rt(1,:)); %new
%     we = cross(at, Rt(3,:)); %old
  
    %Adjust angular velocity
    dc(t,:) = dc(t-1, :) + dt * (-Ki * we);
    wh(t,:) = w(t,:) - dc(t,:) + Kp * we;
    
    %Calculate corrected orientation
    theta = 0.5 * dt * (wh(t-1,:) + wh(t,:));
    Rinf(:,:,t) = Rinf(:,:,t-1)*(eye(3)+(2/(1+0.5*(theta*theta.')))*...
                      (0.5*skew(theta) + 0.25*skew(theta)^2));
end
q = dcm2quatern(permute(Rinf, [2,1,3]));
w = w * 180/pi;

end

function scm = skew(x)

    scm = [0     -x(3)    x(2);
           x(3)    0     -x(1);
          -x(2)   x(1)     0 ];

end

function q = dcm2quatern(R)
%Function to extract a quaternion from a DCM of size (3,3,n).  
%Method from Bar-Itzhack 2000
    numR = size(R,3);
    q = zeros(numR, 4);
    K = zeros(4,4);
    for i = 1:numR
        K(1,1) = (1/3) * (R(1,1,i) - R(2,2,i) - R(3,3,i));
        K(1,2) = (1/3) * (R(2,1,i) + R(1,2,i));
        K(1,3) = (1/3) * (R(3,1,i) + R(1,3,i));
        K(1,4) = (1/3) * (R(2,3,i) - R(3,2,i));
        K(2,1) = (1/3) * (R(2,1,i) + R(1,2,i));
        K(2,2) = (1/3) * (R(2,2,i) - R(1,1,i) - R(3,3,i));
        K(2,3) = (1/3) * (R(3,2,i) + R(2,3,i));
        K(2,4) = (1/3) * (R(3,1,i) - R(1,3,i));
        K(3,1) = (1/3) * (R(3,1,i) + R(1,3,i));
        K(3,2) = (1/3) * (R(3,2,i) + R(2,3,i));
        K(3,3) = (1/3) * (R(3,3,i) - R(1,1,i) - R(2,2,i));
        K(3,4) = (1/3) * (R(1,2,i) - R(2,1,i));
        K(4,1) = (1/3) * (R(2,3,i) - R(3,2,i));
        K(4,2) = (1/3) * (R(3,1,i) - R(1,3,i));
        K(4,3) = (1/3) * (R(1,2,i) - R(2,1,i));
        K(4,4) = (1/3) * (R(1,1,i) + R(2,2,i) + R(3,3,i));
        [V,~] = eig(K);
        q(i,:) = V(:,4)';
        q(i,:) = [q(i,4) q(i,1) q(i,2) q(i,3)] ./ norm(q(i,:));
        
        %Check for continuity
        if numR>1 && i>1
            dq = sum(abs(q(i,:) - q(i-1,:)));
            dqp = sum(abs(-q(i,:) - q(i-1,:)));
            
            if dq > dqp
                q(i,:) = -q(i,:);
            end
        end
    end
end

function e = vd_drift_correct(x,t,v,d,ind,ind_still,c)

[~,dc] = apply_drift_correct(x,t,v,d,ind,ind_still,c);

%Minimize distance from cones
e = [c(2:6,1) - dc(ind,1); c(2:6,2) - dc(ind,2)];

end

function [vc,dc] = apply_drift_correct(x,t_j,v,d,ind,ind_still,c)

%Apply rotation correction
R = [cos(x), sin(x), 0;
    -sin(x), cos(x), 0;
          0,      0, 1];

vr = v * R.';
dr = d * R.';

%Define and apply correction
t=t_j-t_j(1);
wi = [1; 1; 1; 1; 1]; %equation weights
ti = t(ind);
di = dr(ind,:)-c(2:6,:);
t0 = t(1); t6 = t(end);
d0 = dr(1,:); d6 = dr(end,:);
v0 = vr(1,:);

%Define drift correction using initial zero velocity constraint, adjusted t0, waypoints, cubic drift model
%y = T * coe
i0 = ind_still(find(ind_still<ind(1),1,'last'));
t0 = t(i0); d0 = dr(i0,:); v0 = vr(i0,:);

T = [1, t0, t0^2, t0^3, 0, 0, 0;
    1, t6, t6^2, t6^3, 0, 0, 0;
    0, 1, 2*t0, 3*t0^2, 0, 0, 0;
    2*wi.'*ti.^0, 2*wi.'*ti.^1, 2*wi.'*ti.^2, 2*wi.'*ti.^3, -1, -1, 0;
    2*wi.'*ti.^1, 2*wi.'*ti.^2, 2*wi.'*ti.^3, 2*wi.'*ti.^4, -t0, -t6, -1;
    2*wi.'*ti.^2, 2*wi.'*ti.^3, 2*wi.'*ti.^4, 2*wi.'*ti.^5, -t0^2, -t6^2, -2*t0;
    2*wi.'*ti.^3, 2*wi.'*ti.^4, 2*wi.'*ti.^5, 2*wi.'*ti.^6, -t0^3, -t6^3, -3*t0^2];

y(:,1) = [d0(1);
    d6(1)-c(end,1);
    v0(1);
    2*wi.'*di(:,1);
    2*(wi.*ti).'*di(:,1);
    2*(wi.*ti.^2).'*di(:,1);
    2*(wi.*ti.^3).'*di(:,1)];
y(:,2) = [d0(2);
    d6(2)-c(end,2);
    v0(2);
    2*wi.'*di(:,2);
    2*(wi.*ti).'*di(:,2);
    2*(wi.*ti.^2).'*di(:,2);
    2*(wi.*ti.^3).'*di(:,2)];
y(:,3) = [d0(3);
    d6(3)-c(end,3);
    v0(3);
    2*wi.'*di(:,3);
    2*(wi.*ti).'*di(:,3);
    2*(wi.*ti.^2).'*di(:,3);
    2*(wi.*ti.^3).'*di(:,3)];

coe = T \ y;

%define corrected velocity and displacement until last still point at start of trial
ind_s = ind_still(ind_still<=i0);
vc(1:i0,:) = vr(1:i0,:) - interp1(t(ind_s),vr(ind_s,:),t(1:i0),'linear');
dc(1:i0,:) = dr(1:i0,:) - interp1(t(ind_s),dr(ind_s,:),t(1:i0),'linear');

%define corrected velocity and displacement after last still point using cubic error model
t = t(i0:end);
len = length(dr);
dc(i0:len,:) = dr(i0:end,:) - [t.^0, t, t.^2, t.^3]*coe(1:4,:) + (t.^0)*dc(i0,:);
vc(i0:len,:) = vr(i0:end,:) - [t.^0, 2*t, 3*t.^2]*coe(2:4,:);

end

function [xCurrent,Resnorm,FVAL,EXITFLAG,OUTPUT,LAMBDA,JACOB] = lsqnonlin(FUN,xCurrent,LB,UB,options,varargin)
%LSQNONLIN solves non-linear least squares problems.
%   LSQNONLIN attempts to solve problems of the form:
%   min  sum {FUN(X).^2}    where X and the values returned by FUN can be   
%    X                      vectors or matrices.
%
%   LSQNONLIN implements two different algorithms: trust region reflective
%   and Levenberg-Marquardt. Choose one via the option Algorithm: for
%   instance, to choose Levenberg-Marquardt, set 
%   OPTIONS = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt'), 
%   and then pass OPTIONS to LSQNONLIN.
%    
%   X = LSQNONLIN(FUN,X0) starts at the matrix X0 and finds a minimum X to 
%   the sum of squares of the functions in FUN. FUN accepts input X 
%   and returns a vector (or matrix) of function values F evaluated
%   at X. NOTE: FUN should return FUN(X) and not the sum-of-squares 
%   sum(FUN(X).^2)). (FUN(X) is summed and squared implicitly in the
%   algorithm.) 
%
%   X = LSQNONLIN(FUN,X0,LB,UB) defines a set of lower and upper bounds on
%   the design variables, X, so that the solution is in the range LB <= X
%   <= UB. Use empty matrices for LB and UB if no bounds exist. Set LB(i)
%   = -Inf if X(i) is unbounded below; set UB(i) = Inf if X(i) is
%   unbounded above.
%
%   X = LSQNONLIN(FUN,X0,LB,UB,OPTIONS) minimizes with the default
%   optimization parameters replaced by values in OPTIONS, an argument
%   created with the OPTIMOPTIONS function. See OPTIMOPTIONS for details.
%   Use the Jacobian option to specify that FUN also returns a second
%   output argument J that is the Jacobian matrix at the point X. If FUN
%   returns a vector F of m components when X has length n, then J is an
%   m-by-n matrix where J(i,j) is the partial derivative of F(i) with
%   respect to x(j). (Note that the Jacobian J is the transpose of the
%   gradient of F.)
%
%   X = LSQNONLIN(PROBLEM) solves the non-linear least squares problem 
%   defined in PROBLEM. PROBLEM is a structure with the function FUN in 
%   PROBLEM.objective, the start point in PROBLEM.x0, the lower bounds in 
%   PROBLEM.lb, the upper bounds in PROBLEM.ub, the options structure in 
%   PROBLEM.options, and solver name 'lsqnonlin' in PROBLEM.solver. Use 
%   this syntax to solve at the command line a problem exported from 
%   OPTIMTOOL. The structure PROBLEM must have all the fields. 
%
%   [X,RESNORM] = LSQNONLIN(FUN,X0,...) returns 
%   the value of the squared 2-norm of the residual at X: sum(FUN(X).^2). 
%
%   [X,RESNORM,RESIDUAL] = LSQNONLIN(FUN,X0,...) returns the value of the 
%   residual at the solution X: RESIDUAL = FUN(X).
%
%   [X,RESNORM,RESIDUAL,EXITFLAG] = LSQNONLIN(FUN,X0,...) returns an 
%   EXITFLAG that describes the exit condition of LSQNONLIN. Possible 
%   values of EXITFLAG and the corresponding exit conditions are listed
%   below. See the documentation for a complete description.
%
%     1  LSQNONLIN converged to a solution.
%     2  Change in X too small.
%     3  Change in RESNORM too small.
%     4  Computed search direction too small.
%     0  Too many function evaluations or iterations.
%    -1  Stopped by output/plot function.
%    -2  Bounds are inconsistent.
%
%   [X,RESNORM,RESIDUAL,EXITFLAG,OUTPUT] = LSQNONLIN(FUN,X0,...) returns a 
%   structure OUTPUT with the number of iterations taken in
%   OUTPUT.iterations, the number of function evaluations in
%   OUTPUT.funcCount, the algorithm used in OUTPUT.algorithm, the number
%   of CG iterations (if used) in OUTPUT.cgiterations, the first-order
%   optimality (if used) in OUTPUT.firstorderopt, and the exit message in
%   OUTPUT.message.
%
%   [X,RESNORM,RESIDUAL,EXITFLAG,OUTPUT,LAMBDA] = LSQNONLIN(FUN,X0,...) 
%   returns the set of Lagrangian multipliers, LAMBDA, at the solution: 
%   LAMBDA.lower for LB and LAMBDA.upper for UB.
%
%   [X,RESNORM,RESIDUAL,EXITFLAG,OUTPUT,LAMBDA,JACOBIAN] = LSQNONLIN(FUN,
%   X0,...) returns the Jacobian of FUN at X.   
%
%   Examples
%     FUN can be specified using @:
%        x = lsqnonlin(@myfun,[2 3 4])
%
%   where myfun is a MATLAB function such as:
%
%       function F = myfun(x)
%       F = sin(x);
%
%   FUN can also be an anonymous function:
%
%       x = lsqnonlin(@(x) sin(3*x),[1 4])
%
%   If FUN is parameterized, you can use anonymous functions to capture the 
%   problem-dependent parameters. Suppose you want to solve the non-linear 
%   least squares problem given in the function myfun, which is 
%   parameterized by its second argument c. Here myfun is a MATLAB file 
%   function such as
%
%       function F = myfun(x,c)
%       F = [ 2*x(1) - exp(c*x(1))
%             -x(1) - exp(c*x(2))
%             x(1) - x(2) ];
%
%   To solve the least squares problem for a specific value of c, first 
%   assign the value to c. Then create a one-argument anonymous function 
%   that captures that value of c and calls myfun with two arguments. 
%   Finally, pass this anonymous function to LSQNONLIN:
%
%       c = -1; % define parameter first
%       x = lsqnonlin(@(x) myfun(x,c),[1;1])
%
%   See also OPTIMOPTIONS, LSQCURVEFIT, FSOLVE, @, INLINE.

%   Copyright 1990-2013 The MathWorks, Inc.

% ------------Initialization----------------
defaultopt = struct(...
    'Algorithm','trust-region-reflective',...
    'DerivativeCheck','off',...
    'Diagnostics','off',...
    'DiffMaxChange',Inf,...
    'DiffMinChange',0,...
    'Display','final',...
    'FinDiffRelStep', [], ...
    'FinDiffType','forward',...
    'FunValCheck','off',...
    'InitDamping', 0.01, ...
    'Jacobian','off',...
    'JacobMult',[],... 
    'JacobPattern','sparse(ones(Jrows,Jcols))',...
    'MaxFunEvals',[],...
    'MaxIter',400,...
    'MaxPCGIter','max(1,floor(numberOfVariables/2))',...
    'OutputFcn',[],...
    'PlotFcns',[],...
    'PrecondBandWidth',Inf,...
    'ScaleProblem','none',...
    'TolFun',1e-6,...
    'TolPCG',0.1,...
    'TolX',1e-6,...
    'TypicalX','ones(numberOfVariables,1)');

% If just 'defaults' passed in, return the default options in X
if nargin==1 && nargout <= 1 && isequal(FUN,'defaults')
   xCurrent = defaultopt;
   return
end

if nargin < 5
    options = [];
    if nargin < 4
        UB = [];
        if nargin < 3
            LB = [];
        end
    end
end

problemInput = false;
if nargin == 1
    if isa(FUN,'struct')
        problemInput = true;
        [FUN,xCurrent,LB,UB,options] = separateOptimStruct(FUN);
    else % Single input and non-structure.
        error(message('optim:lsqnonlin:InputArg'));
    end
end

% Prepare the options for the solver
[options, optionFeedback] = prepareOptionsForSolver(options, 'lsqnonlin');

if nargin < 2 && ~problemInput
  error(message('optim:lsqnonlin:NotEnoughInputs'))
end

% Check for non-double inputs
msg = isoptimargdbl('LSQNONLIN', {'X0','LB','UB'}, ...
                               xCurrent,LB,  UB);
if ~isempty(msg)
    error('optim:lsqnonlin:NonDoubleInput',msg);
end

caller = 'lsqnonlin'; 
[funfcn,mtxmpy,flags,sizes,funValCheck,xstart,lb,ub,EXITFLAG,Resnorm,FVAL,LAMBDA, ...
    JACOB,OUTPUT,earlyTermination] = lsqnsetup(FUN,xCurrent,LB,UB,options,defaultopt, ...
    caller,nargout,length(varargin));
if earlyTermination
    return % premature return because of problem detected in lsqnsetup()
end

xCurrent(:) = xstart; % reshape back to user shape before evaluation
% Catch any error in user objective during initial evaluation only
switch funfcn{1}
    case 'fun'
        try
            initVals.F = feval(funfcn{3},xCurrent,varargin{:});
        catch userFcn_ME
            optim_ME = MException('optim:lsqnonlin:InvalidFUN', ...
                getString(message('optim:lsqnonlin:InvalidFUN')));
            userFcn_ME = addCause(userFcn_ME,optim_ME);
            rethrow(userFcn_ME)
        end
        initVals.J = [];
    case 'fungrad'
        try
            [initVals.F,initVals.J] = feval(funfcn{3},xCurrent,varargin{:});
        catch userFcn_ME
            optim_ME = MException('optim:lsqnonlin:InvalidFUN', ...
                getString(message('optim:lsqnonlin:InvalidFUN')));
            userFcn_ME = addCause(userFcn_ME,optim_ME);
            rethrow(userFcn_ME)
        end
    case 'fun_then_grad'
        try
            initVals.F = feval(funfcn{3},xCurrent,varargin{:});
        catch userFcn_ME
            optim_ME = MException('optim:lsqnonlin:InvalidFUN', ...
                getString(message('optim:lsqnonlin:InvalidFUN')));
            userFcn_ME = addCause(userFcn_ME,optim_ME);
            rethrow(userFcn_ME)
        end
        try    
            initVals.J = feval(funfcn{4},xCurrent,varargin{:});
        catch userFcn_ME
            optim_ME = MException('optim:lsqnonlin:InvalidFUN', ...
                getString(message('optim:lsqnonlin:InvalidJacobFun')));
            userFcn_ME = addCause(userFcn_ME,optim_ME);
            rethrow(userFcn_ME)
        end
    otherwise
        error(message('optim:lsqnonlin:UndefCallType'))
end

% Check for non-double data typed values returned by user functions 
if ~isempty( isoptimargdbl('LSQNONLIN', {'F','J'}, initVals.F, initVals.J) )
    error('optim:lsqnonlin:NonDoubleFunVal',getString(message('optimlib:commonMsgs:NonDoubleFunVal','LSQNONLIN')));
end

[xCurrent,Resnorm,FVAL,EXITFLAG,OUTPUT,LAMBDA,JACOB] = ...
   lsqncommon(funfcn,xCurrent,lb,ub,options,defaultopt,caller,...
              initVals,sizes,flags,mtxmpy,optionFeedback,varargin{:});
end