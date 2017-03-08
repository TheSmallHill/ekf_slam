function testScript
clear 
close all
clc

setenv('ROS_MASTER_URI', 'http://192.168.1.3:11311')
setenv('ROS_IP','192.168.1.124');

rosinit;

% local node
rosCoreIP = '192.168.1.3';
rosNodeName = 'ekfSlamControl';
node = robotics.ros.Node(rosNodeName,rosCoreIP);

% Create publishers and subscribers
% Estimation subsystem
observeRequest_pub = robotics.ros.Publisher(node, 'observeRequest','std_msgs/Bool');
observeResponse_sub = robotics.ros.Subscriber(node, 'observeResponse', 'ekf_slam/RadioScan');
shutdownRequest_pub = robotics.ros.Publisher(node, 'shutdownRequest','std_msgs/Bool');

observe_msg = rosmessage(observeResponse_sub);
observe_req = rosmessage(observeRequest_pub);
shutdown_req = rosmessage(shutdownRequest_pub);

observe_req.Data = true;
shutdown_req.Data = true;

send(observeRequest_pub, observe_req);

observe_msg = receive(observeResponse_sub);

send(shutdownRequest_pub, shutdown_req);

rosshutdown;

end