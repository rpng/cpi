function [ data_e ] = flipquatorder( data_e )
%FLIPQUATORDER Flips quaternion order [w x y z] => [x y z w]

quatw = data_e.data(:,5);
data_e.data(:,5:7) = data_e.data(:,6:8);
data_e.data(:,8) = quatw;


end

