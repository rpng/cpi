function tmp = rot2rpy(R)

% takes a rotational matrix R and returns the roll, pitch and yaw angle

pitch = atan2(-R(3,1),sqrt(R(1,1)^2 + R(2,1)^2));

if (abs(cos(pitch))>1.0e-12)
   
   yaw = atan2(R(2,1)/cos(pitch), R(1,1)/cos(pitch));
   
   roll= atan2(R(3,2)/cos(pitch), R(3,3)/cos(pitch));
   
else
   yaw=0;
   
   roll=atan2(R(1,2),R(2,2));
end

tmp = [roll, pitch, yaw]';