function [ value ] = getstringvalue( string, key )
%GETSTRINGVALUE Summary of this function goes here
%   Detailed explanation goes here

index = strfind(string, key);
value = sscanf(string(index(1) + length(key):end), '%g', 1);

end

