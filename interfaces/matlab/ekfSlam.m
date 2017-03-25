% /*******************************************************************************************************************************
%  * ekf_slam - a work-in-progress ROS node for an Extended Kalman Filter SLAM (Simultaneous Localization and Mapping) algorithm *
%  * on a Pioneer 3-DX                                                                                                           *
%  *   Copyright (C) 2017  TheSmallHill                                                                                          *
%  *                                                                                                                             *
%  *   This program is free software: you can redistribute it and/or modify                                                      *
%  *   it under the terms of the GNU General Public License as published by                                                      *
%  *   the Free Software Foundation, either version 3 of the License, or                                                         *
%  *   (at your option) any later version.                                                                                       *
%  *                                                                                                                             *
%  *   This program is distributed in the hope that it will be useful,                                                           *
%  *   but WITHOUT ANY WARRANTY; without even the implied warranty of                                                            *
%  *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                                                             *
%  *   GNU General Public License for more details.                                                                              *
%  *                                                                                                                             *
%  *   You should have received a copy of the GNU General Public License                                                         *
%  *   along with this program.  If not, see <http://www.gnu.org/licenses/>.                                                     *
%  *******************************************************************************************************************************/

function ekfSlam
% Runs the EKF-SLAM algorithm on the remote node, ekf_slam
% Adapted from code on openslam.org

%% Initialize MATLAB
clear
close all
clc

pause('on')

cleanupObj = onCleanup(@cleanMeUp);

%% Connect to ROS
setenv('ROS_MASTER_URI', 'http://192.168.1.3:11311')
setenv('ROS_IP','192.168.1.124');
rosCoreIP = '192.168.1.3';
rosinit;
rosNodeName = 'ekfSlamControl';
node = robotics.ros.Node(rosNodeName,rosCoreIP);

% remote node
robotName = 'RosAria';

% Create publishers and subscribers
% Estimation subsystem
observeRequest_pub = robotics.ros.Publisher(node, 'observeRequest', 'std_msgs/Bool');
observeResponse_sub = robotics.ros.Subscriber(node, 'observeResponse', 'ekf_slam/RadioScan');

% Pioneer
pose_sub = robotics.ros.Subscriber(node, strcat('/', robotName, '/pose'), 'nav_msgs/Odometry');
cmdVel_pub = robotics.ros.Publisher(node, strcat('/', robotName, '/cmd_vel'), 'geometry_msgs/Twist');
setPose_pub = robotics.ros.Publisher(node, strcat('/', robotName, '/setpose'), 'nav_msgs/Odometry');

% Temporary variables 
% Estimation subsystem
observe_msg = rosmessage(observeResponse_sub);
observe_req = rosmessage(observeRequest_pub);

% Rosaria
pose = rosmessage(pose_sub);
cmdVel = rosmessage(cmdVel_pub);

% set pose before continuing
pose.Pose.Pose.Position.X = 1.5;
pose.Pose.Pose.Position.Y = 1;
pose.Pose.Pose.Orientation.W = pi/2;
send(setPose_pub, pose);

%% Algorithm parameters
% general
numFeatures = 3;                                    % @TODO: make dynamic (no hardcoded values since number of landmarks is unknown)

% timing
T = .25;                                            % [sec], time step
dtsum = 0;                                          % [sec], time since last observation

% data association
ftag = 1:numFeatures;                               % feature identifier
da_table = zeros(1,numFeatures);                    % data association table

% navigation
wp_idx = 1;                                         % waypoint index
wpIsFinal = false;                                  % set if at final waypoint 
wp = [1.5;                                          % [m], x-coordinates of waypoints
      2];                                         % [m], y-coordinates of waypoints

% control
V = 0;                                              % [m/s], constant linear velocity, @TODO: convert to fuzzy logic
G = 0;                                              % [rad], initial steering angle
G_MAX = deg2rad(180);                               % [rad], max steering angle

% pioneer
V_MAX = 1.2;                                        % [m/s], max linear velocity
W_MAX = deg2rad(300);                               % [rad/s], max angular velocity
WHEEL_DIAMETER = .185;                              % [m], wheel diameter
WHEEL_RADIUS = WHEEL_DIAMETER/2;                    % [m], wheel radius
TRACK = .381;                                       % [m], distance between wheels
ENCODER_TICKS = 500;                                % number of encoder ticks

% observation
DT_OBSERVE = 50*T;                                 % [sec], time between observations
obs_idx = 0;                                        % index for counting obervations

% rssi to distance
ETA = 2;                                            % propogation constant
P_REF = -29;                                        % [dBm], reference power level

%% Algorithm setup
pose = receive(pose_sub);                           % robot's initial pose, measured by robot
q = [pose.Pose.Pose.Position.X; pose.Pose.Pose.Position.Y; pi_to_pi(pose.Pose.Pose.Orientation.W)];
qTrue = [1.5; 1; pi_to_pi(pi/2)];                        % [m, m, rad], estimate of initial pose

Pcov = diag((qTrue-q(1:3)).^2);                     % initial state covariance matrix

% process noise
SIGMA_D = (1/3)*(pi*WHEEL_RADIUS/ENCODER_TICKS);    % standard deviation of displacement noise
SIGMA_V = sqrt((1/T^2)*2*SIGMA_D^2);                % standard deviation of linear velocity
SIGMA_W = (0.2/3)*(pi/180);                         % standard deviation of angular velocity

Q = diag([SIGMA_V^2 SIGMA_W^2]);                    % process noise covariance matrix

% observation noise, @TODO make dynamic or calculate based on prior data
SIGMA_R = 2;                                        % standard deviation of range measurement
SIGMA_B = deg2rad(18);                              % standard deviation of bearing measurement

R = diag([SIGMA_R^2 SIGMA_B^2]);                    % observation noise covariance matrix

% other
AT_WAYPOINT = .25;                                  % [m], threshold for robot to be considered as arrived at waypoint
i = 1;                                              % starting at first iteration
i_max = 1e3;                                        % max number of iteration

%% data saving
qTrueDT(:,i) = qTrue;
qDT(:,i) = q(1:3);
PcovDT{1,i} = Pcov;

%% recording setup
fig = figure;
MINUTES = 1;                                        % Length of video in minutes
VIDEO_LENGTH = 60*MINUTES;                          % Length of video in seconds
vid = VideoWriter('OUT/trajectory.avi');            % Name of the video file
vid.Quality = 100;                                  % all the quality
vid.FrameRate = i_max/VIDEO_LENGTH;                  % Frames per second needed for desired video length and max iterations
open(vid);                                          % Open the video for writing

tic

%% Algorithm running
while(1)
    
%     % initial plotting
%     clf                                             % clear the current figure
% %     lmActual = plot(lm(1,:),lm(2,:),'b*'); % plot actual landmark locations, maybe unknown 
%     hold on 
%     wpActual = plot(wp(1,:),wp(2,:), 'gd-');        % plot the waypoints with lines in between to show the path
%     xlim([-2 5]), ylim([-2 5]);                       % Limit the size of the simulation plot
%     xlabel('X [m]'), ylabel('Y [m]');               % Label the axes
%     grid on
    
    i = i + 1;
   
    % compute steering angle
    cwp = wp(:,wp_idx);
    dist = sqrt((cwp(1) - qTrue(1))^2 + (cwp(2)-qTrue(2))^2);
    if (dist^2 < AT_WAYPOINT^2)
        wp_idx = wp_idx + 1;
        if (wp_idx > size(wp,2))
            wpIsFinal = true;
        else
            cwp = wp(:,wp_idx);
        end
    end
   
    % change in steering angle
    deltaG = pi_to_pi(atan2(cwp(2) - qTrue(2), cwp(1)-qTrue(1))-qTrue(3)-G);

    % limit steering rate
%     maxDelta = RATE_G*T;
%     if abs(deltaG) > maxDelta
%         deltaG = sign(deltaG)*maxDelta;
%     end

    % limit steering angle
    G = G + deltaG;
    if abs(G) > G_MAX
        G = sign(G)*G_MAX;
    end
    
    % convert to our model
    W = V*sin(G)/TRACK;
    
    % this will cause the pioneer to just keep driving 
    if wpIsFinal == true
        wp_idx = 1;
        wpIsFinal = false;
        break;
    end
    
    % limit velocities
    if abs(V) > V_MAX
         V = sign(V)*V_MAX;
     end
     
     if abs(W) > W_MAX
         W = sign(W)*W_MAX;
     end
    
    % estimate noisy velocities using process noise covariance matrix
    if (V ~= 0)
        Vn = V + randn(1)*sqrt(Q(1,1));
    else
        Vn = V;
    end
    
    if (W ~= 0)
        Wn = W + randn(1)*sqrt(Q(2,2));
    else
        Wn = W;
    end
       
%     if i > 1
    if toc < T
        pause(T-toc);
    end
%     end
    % send the command to the pioneer
    cmdVel.Linear.X = V;
    cmdVel.Angular.Z = W;
    send(cmdVel_pub, cmdVel);
%     pause(T);
    tic

    % get the pose after moving for T seconds
    oldPose = q(1:3);
    pose = receive(pose_sub);
    q(1:3) = [pose.Pose.Pose.Position.X; pose.Pose.Pose.Position.Y; pi_to_pi(pose.Pose.Pose.Orientation.W)];
    
    % estimate pose
    if V ~= 0
        qTrue(1:2) = qTrue(1:2) + Vn*T*((q(1:2) - oldPose(1:2))/norm(q(1:2) - oldPose(1:2)));
    end
        
        
    % predict step
    s = sin(G + q(3));
    c = cos(G + q(3));
    
    vts = Vn*T*s;
    vtc = Vn*T*c;
    
    Fk = [1 0 -vts;
          0 1 vtc;
          0 0 1];
      
    Lk = [T*c -vts;
          T*s vtc;
          T*sin(G)/TRACK Vn*T*cos(G)/TRACK];
    
    % predict state covariance  
    Pcov(1:3,1:3) = Fk*Pcov(1:3,1:3)*Fk' + Lk*Q*Lk';
    if size(Pcov,1) > 3
        Pcov(1:3,4:end) = Fk*Pcov(1:3,4:end);
        Pcov(4:end,1:3) = Pcov(1:3,4:end)';
    end  
    
    % observate steps every DT_OBSERVE
    dtsum = dtsum + T;
    if dtsum >= DT_OBSERVE
        dtsum = 0;
        
        pause(T-toc);
        
        % stop pioneer for observations
        cmdVel.Linear.X = 0;
        cmdVel.Angular.Z = 0;
        send(cmdVel_pub, cmdVel);
                
        % get observation
        observe_req.Data = true;
        send(observeRequest_pub, observe_req);
        observe_msg = receive(observeResponse_sub);
        
        % find range and bearing for each beacon (process, reorganize, convert to meters)
        [z, ftag_temp] = processObservations(observe_msg, ETA, P_REF);
        
        % direct data associate
        [zf, idf, zn, da_table] = data_associate_known(q, z, ftag_temp, da_table);
        
        % update step
        [q, Pcov] = update(q, Pcov, zf, R, idf);
        
        % augment step
        [q, Pcov] = augment(q, Pcov, zn, R);
        
        % save estimated beacon states
%         qbeacons{obs_idx} = q(4:end);

        clc
        ftag_temp
        z
        q

    end
    
        % initial plotting
    clf                                             % clear the current figure
%     lmActual = plot(lm(1,:),lm(2,:),'b*'); % plot actual landmark locations, maybe unknown 
    hold on 
    wpActual = plot(wp(1,:),wp(2,:), 'gd-');        % plot the waypoints with lines in between to show the path
    xlim([-2 5]), ylim([-2 5]);                       % Limit the size of the simulation plot
    xlabel('X [m]'), ylabel('Y [m]');               % Label the axes
    grid on
    
    % real time plotting
    [Xa,Ya] = plot_DDMR((q(1:3))',axis(gca));                               % Plot the estimated robot position
    hold on
    [Xd,Yd] = plot_DDMR((qTrue(1:3))',axis(gca));                           % Plot the actual robot position
    qDT(:,i) = q(1:3);                                                      % Save the estimated pose for this time instance
    qTrueDT(:,i) = qTrue;                                                   % Save the actual pose for this time instance
    PcovDT{1,i} = Pcov;
    noisypose = plot(qTrueDT(1,1:i),qTrueDT(2,1:i),'b-');                   % Plot path of the estimated pose up to this point (the line following the moving triangle)
    truepose = plot(qDT(1,1:i),qDT(2,1:i),'r--','linewidth',2);             % Plot path of the actual pose up to this point (the line following the moving triangle)
fill(Xa,Ya,'r');                                                        % Fill in the estimated triangle with red
    fill(Xd,Yd,'b');                                                        % Fill in the actual triangle with blue    
    landmarkEstimate = plot(q(4:2:end), q(5:2:end),'k.','MarkerSize',16);   % Plot the estimated positions of the beacons
    
    % Print the current iteration value in the plot so it is visible in the video
    iter = sprintf('i = %d', i);
    text(.5, .5, iter);
    
%     fill(Xa,Ya,'r');                                                        % Fill in the estimated triangle with red
%     fill(Xd,Yd,'b');                                                        % Fill in the actual triangle with blue    
    
%     if dtsum==0
%         if ~isempty(z)                                                                              % If there are measurements
%             plines = make_laser_lines (z,q(1:3));                                                   % Draw lines between the actual robot and the beacons it is currently using, see function below
%             plot(plines(1,:),plines(2,:),'r-');                                                     % Put the lines in the figure and make them solid red           
%             pcov = make_covariance_ellipses(q,Pcov);                                                   % Make the covariance ellipses, see function below
%             [U1,S1,V1] = svd(Pcov(1:3,1:3));                                                           % Singular value decomposition, where P(1:3,1:3)=U1*S1*V1
%             ellipse(30*S1(1,1),30*S1(2,2),atan2(U1(1,2),U1(1,1)),qTrueDT(1,i),qTrueDT(2,i),'c');    % Use the svd results to make the ellipse around the robot?    
%             %ellipse(S1(1,1),S1(2,2),atan2(U1(1,2),U1(1,1)),QDT(1,i),QDT(2,i),'r');               
%             plot(pcov(1,:), pcov(2,:),'r')                                                          % Landmark pose covariance uncertainty ellipse
%         end
%     end
    drawnow                                                                 % Updates the figure immediately
    F = getframe(fig);                                                      % Get the current frame
    writeVideo(vid,F);                                                      % Write the current frame to the video

    % stop after some iterations
    if (i >= i_max)  % time = DT_CONTROLS*2e3
        break;        
    end
    
    % a debugging breakpoint
    if i == 1e3
        breakpoint = true;
    end
    
end

%% save all data
close(vid)
save('OUT/allData.mat');

%% position figure
% plot of x, y, and theta (3 subplots)
figure(2);
subplot(3,1,1); % x-position subplot
trueq = plot((1:i)*T,qDT(1,:), 'b-');
hold on
noisyq = plot((1:i)*T,qTrueDT(1,:),'r--', 'linewidth',2);
title('Robot pose');
xlabel('Iteration');
ylabel('x-position [m]');
grid on

subplot(3,1,2); % y-position subplot
plot((1:i)*T,qDT(2,:),'b-');
hold on
plot((1:i)*T,qTrueDT(2,:),'r--', 'linewidth', 2);
xlabel('Time [sec]');
ylabel('y-position [m]');
grid on

subplot(3,1,3); % orientation subplot
plot((1:i)*T,qDT(3,:),'b-');
hold on
plot((1:i)*T,qTrueDT(3,:),'r--', 'linewidth', 2);
xlabel('Time [sec]');
ylabel('Orientation [rad]');
grid on

% save figure 2
saveas(gcf, 'OUT/positionComparison','fig');
print('-depsc2','-r300','OUT/positionComparison.eps');

%% robot and beacon error figures
% for ii = 1:size(qbeacons,2)
%     qbeacons_temp(:,ii) = NaN*zeros(2*num_beacons,1);
%     qbeacons_temp(1:size(qbeacons{ii},1),ii) = qbeacons{ii};
%     
% % x values
%     for i = 1:size(qbeacons{ii},1)/2
%         qbeacons_temp{ii}(da_table(i),1) = qbeacons{ii}(i*2-1);
%         qbeacons_temp{ii}(da_table(i),2) = qbeacons{ii}(i*2);
%     end
% y values
%     for i = 2:2:num_beacons
%         qbeacons_temp{ii}(da_table(i),2) = qbeacons{ii}(i);    
%     endd
%     qbeacons_temp{ii}(:,1) = qbeacons{1:2:end};
%     qbeacons_temp{ii}(:,2) = qbeacons{2:2:end};
% end

% for ii = 1:size(qbeacons_temp,2)
%     for jj = da_table(end,:)
%        beac_error(jj,ii) = sqrt((qbeacons_temp{ii}(jj, 1) - lm(1, jj))^2 + (qbeacons_temp{ii}(jj, 2) - lm(2, jj))^2);
%     end
%    beac_error(1,ii) =  sqrt((qbeacons_temp(1,ii) - lm(1,4))^2 + (qbeacons_temp(2,ii) - lm(2,4))^2);
%    beac_error(2,ii) =  sqrt((qbeacons_temp(3,ii) - lm(1,5))^2 + (qbeacons_temp(4,ii) - lm(2,5))^2);
%    beac_error(3,ii) =  sqrt((qbeacons_temp(5,ii) - lm(1,6))^2 + (qbeacons_temp(6,ii) - lm(2,6))^2);
%    beac_error(4,ii) =  sqrt((qbeacons_temp(7,ii) - lm(1,7))^2 + (qbeacons_temp(8,ii) - lm(2,7))^2);
%    beac_error(5,ii) =  sqrt((qbeacons_temp(9,ii) - lm(1,8))^2 + (qbeacons_temp(10,ii) - lm(2,8))^2);
%    beac_error(6,ii) =  sqrt((qbeacons_temp(11,ii) - lm(1,9))^2 + (qbeacons_temp(12,ii) - lm(2,9))^2);
%    beac_error(7,ii) =  sqrt((qbeacons_temp(13,ii) - lm(1,3))^2 + (qbeacons_temp(14,ii) - lm(2,3))^2);
%    beac_error(8,ii) =  sqrt((qbeacons_temp(15,ii) - lm(1,1))^2 + (qbeacons_temp(16,ii) - lm(2,1))^2);
%    beac_error(9,ii) =  sqrt((qbeacons_temp(17,ii) - lm(1,2))^2 + (qbeacons_temp(18,ii) - lm(2,2))^2);
% end

% beac_error(isnan(beac_error))=0;

% for i=1:size(beac_error,1)
%    for j=1:size(beac_error,2)
%       if ISNAN(beac_error(i,j))
%           beac_error(i,j) = 0;
%       end
%    end
% end

% eb = sum(beac_error,1);
% RMSE_beac = sqrt((1/size(eb,2))*sum(eb.^2));

% figure
% plot((1:obs_idx)*DT_OBSERVE*T, eb);
% title('Total Beacon Position error');
% xlabel('Time [sec]');
% ylabel('e_B(t) [m]');
% grid on

% savefilename = 'OUT/beaconerror';
% saveas(gcf, savefilename, 'fig');
% print('-depsc2','-r300',[savefilename, '.eps']);

% figure
% plot((1:i-1)*T, robot_error);
% title('Robot Error');
% xlabel('Time [sec]');
% ylabel('e_P(t) [m]');
% grid on

% savefilename = 'OUT/roboterror';
% saveas(gcf, savefilename, 'fig');
% print('-depsc2','-r300',[savefilename, '.eps']);

% for ii = 1:obs_idx
%     b4_err(ii,1) = sqrt((qbeacons{ii}(:,1)-lm(1,4))^2+(qbeacons{ii}(:,2)-lm(2,4))^2);
% end

%% Save everything again (including plots)
save('OUT/allData.mat');

end

% wrap angle [-pi, pi) 
function angle = pi_to_pi(angle)

angle = mod(angle, 2*pi);

i = find(angle > pi);
angle(i) = angle(i) - 2*pi;

i = find(angle < -pi);
angle(i) = angle(i) + 2*pi;

end

% ekf data association
function [zf,idf,zn, table]= data_associate_known(x,z,idz, table)

zf= []; zn= [];
idf= []; idn= [];

% find associations (zf) and new features (zn)
for i=1:length(idz)
    ii= idz(i);
    if table(ii) == 0 % new feature
        zn= [zn z(:,i)];
        idn= [idn ii];
    else
        zf= [zf z(:,i)];
        idf= [idf table(ii)];
    end
end

% add new feature IDs to lookup table
Nxv= 3; % number of vehicle pose states
Nf= (length(x) - Nxv)/2; % number of features already in map
table(idn)= Nf + (1:size(zn,2)); % add new feature positions to lookup table
end

function [z, H] = observe_model(x, idf)

Nxv = 3; % number of vehicle pose states
fpos = Nxv + idf*2 - 1; % position of xf in state
H = zeros(2, length(x)); % preallocate the observation jacobian

% auxiliary values, pretty sure these for derivatives
dx = x(fpos) - x(1); % change in value of x
dy = x(fpos+1) - x(2); % change in value of y 
d2 = dx^2 + dy^2; % squared distance
d = sqrt(d2); % distance
xd = dx/d; % normalized x distance?
yd = dy/d; % normalized y distance?
xd2 = dx/d2; % normalized x squared distance?
yd2 = dy/d2; % normalized y squared distance?

% predict z
z = [d;
     atan2(dy,dx) - x(3)];

% calculate H, the observation jacobian
H(:,1:3) = [-xd -yd 0; yd2 -xd2 -1];
H(:,fpos:fpos+1) = [ xd  yd;  -yd2  xd2];
 
end

% ekf update
function [x,P] = update(x,P,z,R,idf)

lenz = size(z,2); % number of measurements
lenx = length(x); % number of states for the robot (x,y,theta)
H = zeros(2*lenz, lenx);
v = zeros(2*lenz, 1);
RR = zeros(2*lenz);

% update everything
for i=1:lenz
    ii = 2*i + (-1:0);
    [zp,H(ii,:)] = observe_model(x, idf(i));
    
    v(ii) = [z(1,i)-zp(1);
    pi_to_pi(z(2,i)-zp(2))];
    RR(ii,ii) = R;
end

% use cholesky decomposition to update
[x,P] = KF_cholesky_update(x,P,v,RR,H);

end

function [x,P] = KF_cholesky_update(x,P,v,R,H)

PHt = P*H';
S = H*PHt + R;

S = (S+S')*0.5; % make symmetric
SChol = chol(S);

SCholInv = inv(SChol); % triangular matrix
W1 = PHt * SCholInv;
W = W1 * SCholInv';

x = x + W*v; % update 
P = P - W1*W1';
end

% ekf augment
function [x,P] = augment(x,P,z,R) 

% add new features to state
for i=1:size(z,2)
    [x,P] = add_one_z(x,P,z(:,i),R);
end

end

% add a feature to the list
function [x,P]= add_one_z(x,P,z,R)

len = length(x);
r = z(1); % range
b = z(2); % bearing
s = sin(x(3)+b); % sine portion
c = cos(x(3)+b); % cosine portion

% augment x
x = [x;
     x(1) + r*c;
     x(2) + r*s];

% jacobians
Gv = [1 0 -r*s;
      0 1  r*c];

Gz = [c -r*s;
      s  r*c];
     
% augment P
rng = len+1:len+2;
P(rng,rng) = Gv*P(1:3,1:3)*Gv' + Gz*R*Gz'; % feature cov
P(rng,1:3) = Gv*P(1:3,1:3); % vehicle to feature xcorr
P(1:3,rng) = P(rng,1:3)';

if len>3
    rnm = 4:len;
    P(rng,rnm) = Gv*P(1:3,rnm); % map to feature xcorr
    P(rnm,rng) = P(rng,rnm)';
end

end

function [X,Y] = plot_DDMR(Q,AX)

x     = Q(1);
y     = Q(2);
theta = Q(3);

l1 = 0.02*max([AX(2)-AX(1),AX(4)-AX(3)]);
X = [x,x+l1*cos(theta-2*pi/3),x+l1*cos(theta),x+l1*cos(theta+2*pi/3),x];
Y = [y,y+l1*sin(theta-2*pi/3),y+l1*sin(theta),y+l1*sin(theta+2*pi/3),y];
end

function p = make_laser_lines (rb,xv)

% compute set of line segments for laser range-bearing measurements
if isempty(rb)
    p = []; 
    return;
end

len = size(rb,2);
lnes(1,:) = zeros(1,len)+ xv(1);
lnes(2,:) = zeros(1,len)+ xv(2);
lnes(3:4,:) = TransformToGlobal([rb(1,:).*cos(rb(2,:)); rb(1,:).*sin(rb(2,:))], xv);
p = line_plot_conversion (lnes);

end

function p= make_covariance_ellipses(x,P)

% compute ellipses for plotting state covariances
N = 10;
inc = 2*pi/N;
phi = 0:inc:2*pi;

lenx = length(x);
lenf = (lenx-3)/2;
p = zeros (2,(lenf+1)*(N+2));

ii = 1:N+2;
p(:,ii) = make_ellipse(x(1:2), P(1:2,1:2), 2, phi);

ctr = N+3;
for i=1:lenf
    ii = ctr:(ctr+N+1);
    jj = 2+2*i;
    jj = jj:jj+1;
    
    p(:,ii) = make_ellipse(x(jj), P(jj,jj), 2, phi);
    ctr = ctr+N+2;
end

end

function p = make_ellipse(x,P,s, phi)

% make a single 2-D ellipse of s-sigmas over phi angle intervals 
r = sqrtm(P);
a = s*r*[cos(phi); sin(phi)];
p(2,:) = [a(2,:)+x(2) NaN];
p(1,:) = [a(1,:)+x(1) NaN];

end

function p = TransformToGlobal(p, b)

% rotate
rot = [cos(b(3)) -sin(b(3)); sin(b(3)) cos(b(3))];
p(1:2,:) = rot*p(1:2,:);

% translate
p(1,:) = p(1,:) + b(1);
p(2,:) = p(2,:) + b(2);

% if p is a pose and not a point
if size(p,1)==3
   p(3,:) = pi_to_pi(p(3,:) + b(3));
end

end

function p= line_plot_conversion (lne)

len = size(lne,2)*3 - 1;
p = zeros(2, len);

p(:,1:3:end) = lne(1:2,:);
p(:,2:3:end) = lne(3:4,:);
p(:,3:3:end) = NaN;

end

function h = ellipse(ra,rb,ang,x0,y0,C,Nb)

% Check the number of input arguments 
if nargin < 1
  ra = [];
end;
if nargin < 2
  rb = [];
end;
if nargin < 3
  ang = [];
end;

if nargin < 5
  x0 = [];
  y0 = [];
end;
 
if nargin < 6
  C = [];
end

if nargin < 7
  Nb = [];
end

% set up the default values
if isempty(ra)
    ra=1;
end
if isempty(rb)
    rb=1;
end
if isempty(ang)
    ang=0;
end
if isempty(x0)
    x0=0;
end
if isempty(y0)
    y0=0;
end
if isempty(Nb)
    Nb=300;
end
if isempty(C)
    C = get(gca,'colororder');
end

% work on the variable sizes, transposes matrices
x0 = x0(:);
y0 = y0(:);
ra = ra(:);
rb = rb(:);
ang = ang(:);
Nb = Nb(:);

% if isstr(C)
if ischar(C)
    C = C(:);
end

% some error messages
if length(ra)~=length(rb)
  error('length(ra)~=length(rb)');
end
if length(x0)~=length(y0)
  error('length(x0)~=length(y0)');
end

% how many inscribed elllipses are plotted
if length(ra)~=length(x0)
  maxk=length(ra)*length(x0);
else
  maxk=length(ra);
end

% drawing loop
for k=1:maxk
  
  if length(x0)==1
    xpos = x0;
    ypos = y0;
    radm = ra(k);
    radn = rb(k);
    if length(ang)==1
      an = ang;
    else
      an = ang(k);
    end
  elseif length(ra)==1
    xpos = x0(k);
    ypos = y0(k);
    radm = ra;
    radn = rb;
    an = ang;
  elseif length(x0)==length(ra)
    xpos = x0(k);
    ypos = y0(k);
    radm = ra(k);
    radn = rb(k);
    an = ang(k);
  else
    rada = ra(fix((k-1)/size(x0,1))+1);
    radb = rb(fix((k-1)/size(x0,1))+1);
    an = ang(fix((k-1)/size(x0,1))+1);
    xpos = x0(rem(k-1,size(x0,1))+1);
    ypos = y0(rem(k-1,size(y0,1))+1);
  end

  co = cos(an);
  si = sin(an);
  the = linspace(0,2*pi,Nb(rem(k-1,size(Nb,1))+1,:)+1);
%  x=radm*cos(the)*co-si*radn*sin(the)+xpos;
%  y=radm*cos(the)*si+co*radn*sin(the)+ypos;
  h(k) = line(radm*cos(the)*co-si*radn*sin(the)+xpos,radm*cos(the)*si+co*radn*sin(the)+ypos);
  set(h(k),'color',C(rem(k-1,size(C,1))+1,:));

end
end

function [z, ftag] = processObservations(observe_msg, eta, ref)

%     ftag = 0;
%     eta = 2;
%     ref = -35;
    
    total_beacons = 0;
for i = 1:size(observe_msg.Observations,1) % through the rows
       
       if size(observe_msg.Observations(i).Observation,1) > total_beacons
           total_beacons = size(observe_msg.Observations(i).Observation,1);
       end                 
end

for i = 1:size(observe_msg.Observations,1)
    data{i} = NaN*ones(total_beacons,1);
end

for i = 1:size(observe_msg.Observations,1) % through the rows
    
%     for j=1:total_beacons
%         data{i}(j) = Inf;
%     end
    
    % go through beacons in each row
    for j=1:size(observe_msg.Observations(i).Observation,1)
       
       
       [~,temp] = strtok(observe_msg.Observations(i).Observation(j).Name, 'n');
       temp = str2double(temp(2:end));
       
       % cell array, each row is an angle, beacon number is index of rssi
       % data
       data{i}(temp) = observe_msg.Observations(i).Observation(j).Rssi;
       
       ftag(j) = temp;
       
    end
        
end

for k = 1:total_beacons
max = Inf;
idx = 0;
    for i=1:size(data,2)
   
%         if size(data{i} < total_beacons
        
        if data{i}(k) < max
        
            max = data{i}(k);
            idx = i;
            
        end
        
    end
    
    if (idx ~= 0)
        z(1,k) = max;
        z(2,k) = pi_to_pi(deg2rad(observe_msg.Observations(idx).Angle));
%         if k == 1
%             ftag = data{idx};
%         else    
%             ftag = [ftag; idx];
%         end
    end
    
%     % put ftags in order
%     ftag = sort(ftag);
    
end

    z = rssi2distance(z, eta, ref);

    [ftag, idx] = sort(ftag);
    
%     z(1,:) = z(1,idx);
%     z(2,:) = z(2,idx);

    if ~isequal(ftag, idx)
        z = z(:,ftag)
    else
        breakpoint=1;
    end
    % put ftags in order
%     ftag = sort(ftag);

end

function z = rssi2distance(z, eta, ref)

for i = 1:size(z,2)
   
    z(1,i) = 10^((abs(z(1,i)) - abs(ref))/(10*eta));
    
end

end

% cleanup function
function cleanMeUp()

    rosshutdown;
    disp('Cleanup successful');

end
