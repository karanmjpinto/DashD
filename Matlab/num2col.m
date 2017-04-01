%Written by JamieR as an easy way of making things pretty.

function [ RGB ] = num2col(x,min,max)
R = [cos(0:pi/2000:pi/2),zeros(1,1000)];
G = sin(0:pi/2000:pi);
B = sin([zeros(1,1000),0:pi/2000:pi/2]);

fract = (max-x)/(max-min);

i = round(2000*fract);

%Stops Matlab Complaining
if (i == 0)
    i = 1;
end

RGB = [R(i),G(i),B(i)];
end

