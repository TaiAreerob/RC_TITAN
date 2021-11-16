function [next,pass,next_index,pass_index] = nextWP(W,p,current_index)

for i = 1:length(W)
    dist(i) = dis(p,W(:,i));
end
d_sort=  sort(dist,'ascend');

index_min = find(dist==min(d_sort(d_sort > 0))); %Find index of dist to the shortest distance
index_min2 = find(dist==min(d_sort(d_sort > min(d_sort(d_sort > 0))))); %Find index of dist to the 2nd shortest distance

l = 30; %look ahead l points to reconsider index_min2
for i = find(d_sort>dist(index_min),l)
    if index_min2 < index_min
        index_min2 = find(dist==d_sort(i));
    elseif index_min2-index_min > 1
        index_min2 = find(dist==d_sort(i));
    end
end



%Create unit vectors
v_1 = (W(:,index_min)-p)/norm(W(:,index_min)-p);
v_2 = (W(:,index_min2)-W(:,index_min))/norm(W(:,index_min2)-W(:,index_min));
%Find angle of the two vectors
theta = rad2deg(acos(v_1'*v_2));
if abs(theta) > 90 %W(index_min) has passed
    next = W(:,index_min2);
    pass = W(:,index_min);
elseif abs(theta) < 90 %W(index_min) has not passed
    next = W(:,index_min);
    pass = W(:,index_min-1);
elseif abs(theta) == 90 %W(index_min) is perpendicular to %W(index_min2)
    next = W(:,index_min);
    pass = W(:,index_min-1);
end

%If there is any output more than 2 variables
if nargout > 2
    next_index = find(W(1,:)==next(1) & W(2,:)==next(2));
    pass_index = find(W(1,:)==pass(1) & W(2,:)==pass(2));
end

%For initial point
if current_index == 1
    next = W(:,current_index+1);
    pass = [0;0];
    next_index = current_index+1;
    pass_index = [1];
end