function set_initialpose(x,y,yaw)

msg = rosmessage('geometry_msgs/PoseWithCovarianceStamped');

msg.Pose.Pose.Position.X = x;
msg.Pose.Pose.Position.Y = y;

q = eul2quat([yaw,0,0]);
msg.Pose.Pose.Orientation.W = q(1);
msg.Pose.Pose.Orientation.X = q(2);
msg.Pose.Pose.Orientation.Y = q(3);
msg.Pose.Pose.Orientation.Z = q(4);

msg.Pose.Covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.07];
topic='/initialpose';
pub = rospublisher(topic,'geometry_msgs/PoseWithCovarianceStamped');
fprintf('Publishing the following message on the channel: %s\n',topic);
showdetails(msg)
send(pub,msg);