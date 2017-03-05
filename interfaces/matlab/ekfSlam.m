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
%% Initialize MATLAB
clear
close all
clc

%% Connect to ROS 
rosCoreIP = '192.168.1.3';
rosNodeName = 'ekfSlamControl';
node = robotics.ros.Node(rosNodeName,rosCoreIP);

%% Create publishers and subscribers
observeRequestPub = robotics.ros.Publisher(node, 'observeRequest','std_msgs/Bool');
observeResponseSub = robotics.ros.Subscriber(node, 'observeResponse', 'xbee_scanner/RadioScan');

% temporary variables
observe_msg = rosmessage(observeResponseSub);
observe_req = rosmessage(observeRequestPub);

% send a request...
observe_req.data = true;
send(observeRequestPub, observe_req);

%...and wait for the response
observe_msg = receive(observe_msg);

% set a breakpoint here so we can read the data
breakpoint = 1;

end