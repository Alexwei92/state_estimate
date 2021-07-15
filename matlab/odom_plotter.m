clc;
clear;
clf;

rotate3d on;
hold on;

%% Intel odometry

bag = rosbag('indoor_data1.bag');

% bag.MessageList
% bag.AvailableTopics

bagselect = select(bag, 'Topic', '/t265/odom/sample');
% msgs = readMessages(bagselect);

ts = timeseries(bagselect, 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y', 'Pose.Pose.Position.Z');
data = ts.Data;

% apply rotation matrix to intel camera (acc to pose)
rot = [-1 0 0
        0 1 0
        0 0 1];

initial_x = data(1, 1);
initial_y = data(1, 2);
initial_z = data(1, 3);

for i=1:length(data)
  x = data(i, 1);
  y = data(i, 2);
  z = data(i, 3);
  
  corrected = rot * [x
                     y
                     z];
                 
  data(i, 1) = corrected(1) - initial_x;
  data(i, 2) = corrected(2) - initial_y;
  data(i, 3) = corrected(3) - initial_z;
end

X = data(:, 1);
Y = data(:, 2);
Z = data(:, 3);

plot3(X, Y, Z, 'b');

%% VINS odometry

bag2 = rosbag('vins_odom.bag');

bagselect2 = select(bag2, 'Topic', '/vins_estimator/odometry');

ts2 = timeseries(bagselect2, 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y', 'Pose.Pose.Position.Z');
data2 = ts2.Data;

initial_x2 = data2(1, 1);
initial_y2 = data2(1, 2);
initial_z2 = data2(1, 3);

for i=1:length(data2)
    data2(i, 1) = data2(i, 1) - initial_x2;
    data2(i, 2) = data2(i, 2) - initial_y2;
    data2(i, 3) = data2(i, 3) - initial_z2;
end

X2 = data2(:, 1);
Y2 = data2(:, 2);
Z2 = data2(:, 3);

plot3(X2, Y2, Z2, 'r');