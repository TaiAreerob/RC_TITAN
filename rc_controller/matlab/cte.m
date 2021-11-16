function [e,v1,v2] = cte(w1,w2,p)
%w1=ahead waypoint, w2 = behind waypoint, p=current actual point

%Linear line between the 2 waypoints
c(1,1:2) = polyfit([w1(1,1) w2(1,1)],[w1(2,1) w2(2,1)],1); %c(1)x-y+c(2)=0

%Check if w1 is passed
a = (w1-p)/norm(w1-p); %Unit vector from p to w1
b = (w2-w1)/norm(w2-w1); %Unit vector from w1 to w2

%Find angle of the two vectors
theta = rad2deg(acos(a'*b));

if abs(theta) == 0  %a and b are on the same direction
    e = 0;
elseif abs(theta) > 0 & abs(theta) ~= 90
    e = abs(c(1,1)*p(1,1)-p(2,1)+c(1,2))/sqrt(c(1,1)^2+(-1)^2); %minimum distance from point to line
elseif abs(theta) == 90
    e = dis(p,w1); %minimum distance from p to w1
end

if nargout > 1
    dotProd = a'*b;
    v1 = a;
    v2 = b;
end