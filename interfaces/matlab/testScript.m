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

function testScript
%% Initialize matlab
clear 
close all
clc

cleanupObj = onCleanup(@cleanMeUp);

%% ros setup
setenv('ROS_MASTER_URI', 'http://192.168.1.3:11311')
setenv('ROS_IP','192.168.1.124');
rosinit;

% local node
rosCoreIP = '192.168.1.3';
rosNodeName = 'ekfSlamControl';
node = robotics.ros.Node(rosNodeName,rosCoreIP);

%% Create publishers and subscribers
% Estimation subsystem
observeRequest_pub = robotics.ros.Publisher(node, 'observeRequest','std_msgs/Bool');
observeResponse_sub = robotics.ros.Subscriber(node, 'observeResponse', 'ekf_slam/RadioScan');
shutdownRequest_pub = robotics.ros.Publisher(node, 'shutdownRequest','std_msgs/Bool');

% temporary variables
observe_msg = rosmessage(observeResponse_sub);
bool_msg = rosmessage(observeRequest_pub); % used for both observation request and shutdown request
bool_msg.Data = true;

% send request for observation
send(observeRequest_pub, bool_msg);
observe_msg = receive(observeResponse_sub);

total_beacons = 0;
for i = 1:size(observe_msg.Observations,1) % through the rows
       
       if size(observe_msg.Observations(i).Observation,1) > total_beacons
           total_beacons = size(observe_msg.Observations(i).Observation,1);
       end                 
end

for i = 1:size(observe_msg.Observations,1)
   
    data{i} = NaN*ones(2,1);
    
%     for j=1:total_beacons
%     
%         
%         
%     end
end
%% process things
for i = 1:size(observe_msg.Observations,1) % through the rows
    
    % go through beacons in each row
    for j=1:size(observe_msg.Observations(i).Observation,1)
       
       
       [~,temp] = strtok(observe_msg.Observations(i).Observation(j).Name, 'n');
       temp = str2double(temp(2:end));
       
       % cell array, each row is an angle, beacon number is index of rssi
       % data
       data{i}(temp) = observe_msg.Observations(i).Observation(j).Rssi;
            
    end
        
end

for k = 1:total_beacons
max = Inf;
idx = 0;
    for i=1:size(data,2)
   
        if data{i}(k) < max
        
            max = data{i}(k);
            idx = i;
            
        end
        
    end
    
    z(1,k) = max;
    z(2,k) = observe_msg.Observations(idx).Angle;
    
end
z

%% shutdown
send(shutdownRequest_pub, bool_msg);

save('data.mat');

end

function cleanMeUp()

rosshutdown;
disp('Cleanup successful');

end