%this script is for tracking PID
%Oct 8, 2021
%Edit by BP
clear; clc; close all;

%Set all parameters
phi_max = 30*pi/180; %maximum steering angle in radians
L = 108; %Distance between front and rear wheels in cm
w = 80; l=150;
r = L/tan(phi_max); %radius of turning circle in cm
v = 1; %Velocity of 1 cm/s
p = 1700; %time steps in one period

%Create rectangle path (400m x50m)
init = [0 0]; %the initial point
c1 = [20000,0]; c2 = [20000,-5000]; c3 = [-20000,-5000]; c4 = [-20000,0]; %Specify co-ordinate at all corners
np1 = 250; np2 = 350; np3 = 500; np4 = 350; np5 = 250; %Specify number of points in each side
%generate randomly spaced values
random_p(1:np1) = sort(init(1)+rand(1,np1)*(c1(1)-init(1)),'ascend'); %x increases
random_p(np1+1:np1+np2) = sort(c1(2)+rand(1,np2)*(c2(2)-c1(2)),'descend'); %y decreases
random_p(np1+np2+1:np1+np2+np3) = sort(c2(1)+rand(1,np3)*(c3(1)-c2(1)),'descend'); %x decreases
random_p(np1+np2+np3+1:np1+np2+np3+np4) = sort(c3(2)+rand(1,np4)*(c4(2)-c3(2)),'ascend'); %y increases
random_p(np1+np2+np3+np4+1:np1+np2+np3+np4+np5) = sort(c4(1)+rand(1,np5)*(init(1)-c4(1)),'ascend'); %x increases

%Desired path
%Generate x-y co-ordinates in each side 
Desire.x(1) = init(1); Desire.y(1) = init(2);
marks_d = [" "]; %String vector containing number labels
for i=1:p
    if i<=np1 %x increases
        Desire.x(i+1) = random_p(i);
        Desire.y(i+1) = Desire.y(i);
    elseif i>np1 && i<=np1+np2 %y decreases
        Desire.x(i+1) = Desire.x(i);
        Desire.y(i+1) = random_p(i);
    elseif i>np1+np2 && i<=np1+np2+np3 %x decreases
        Desire.x(i+1) = random_p(i);
        Desire.y(i+1) = Desire.y(i);
    elseif i>np1+np2+np3 && i<=np1+np2+np3+np4 %y increases
        Desire.x(i+1) = Desire.x(i);
        Desire.y(i+1) = random_p(i);
    elseif i>np1+np2+np3+np4 %x increases
        Desire.x(i+1) = random_p(i);
        Desire.y(i+1) = Desire.y(i);
    end
    mark = num2str(i); %Store value of i in the string variable
    marks_d = [marks_d, mark]; %Store in the string vector
end

%Draw the desired path
figure; plot(Desire.x,Desire.y,'o');

%generate randomly spaced values
dev = 10; %Deviation 
random_p(1:np1) = sort(init(1)+rand(1,np1)*(c1(1)-init(1)),'ascend'); %x increases
random_p(np1+1:np1+np2) = sort(c1(2)+rand(1,np2)*(c2(2)-c1(2)),'descend'); %y decreases
random_p(np1+np2+1:np1+np2+np3) = sort(c2(1)+rand(1,np3)*(c3(1)-c2(1)),'descend'); %x decreases
random_p(np1+np2+np3+1:np1+np2+np3+np4) = sort(c3(2)+rand(1,np4)*(c4(2)-c3(2)),'ascend'); %y increases
random_p(np1+np2+np3+np4+1:np1+np2+np3+np4+np5) = sort(c4(1)+rand(1,np5)*(init(1)-c4(1)),'ascend'); %x increases
%Actual path
%Generate x-y co-ordinates in each side 
Actual.x(1) = init(1); Actual.y(1) = init(2);
marks_a = [" "]; %String vector containing number labels
for i=1:p
    if i<=np1 %x increases
        Actual.x(i+1) = random_p(i);
        Actual.y(i+1) = Actual.y(i);
    elseif i>np1 && i<=np1+np2 %y decreases
        Actual.x(i+1) = Actual.x(i);
        Actual.y(i+1) = random_p(i);
    elseif i>np1+np2 && i<=np1+np2+np3 %x decreases
        Actual.x(i+1) = random_p(i);
        Actual.y(i+1) = Actual.y(i);
    elseif i>np1+np2+np3 && i<=np1+np2+np3+np4 %y increases
        Actual.x(i+1) = Actual.x(i);
        Actual.y(i+1) = random_p(i);
    elseif i>np1+np2+np3+np4 %x increases
        Actual.x(i+1) = random_p(i);
        Actual.y(i+1) = Actual.y(i);
    end
    mark = num2str(i); %Store value of i in the string variable
    marks_a = [marks_a, mark]; %Store in the string vector
end
%Draw the path
hold on; plot(Actual.x,Actual.y,'r*');

%%
% %Draw the paths from initial point to final point
% init = 10; final = 40;
% figure;
% for i = init:final
%     plot(Desire.x(i),Desire.y(i),'o'); hold on
%     text(Desire.x(i),Desire.y(i),marks_d(i),'VerticalAlignment','bottom','HorizontalAlignment','right');
%     plot(Actual.x(i),Actual.y(i),'r*'); hold on
%     text(Actual.x(i),Actual.y(i),marks_a(i),'VerticalAlignment','bottom','HorizontalAlignment','left');
% end

%%
%Find the shortest distance from a point to a linear line
W = [Desire.x; Desire.y]; %matrix with 2 rows
P = [Actual.x; Actual.y]; %matrix with 2 rows

%%
%This figure; displays the current vehicle's point compared with the passed
%and ahead waypoints

for i = 1:1690%1:length(P)-1
    [next(:,i),pass(:,i),next_index(i),pass_index(i)] = nextWP(W,P(:,i),i);
    %Plot the current point compared with the passed and ahead points
    figure;plot(next(1,i),next(2,i),'o'); hold on;
    text(next(1,i),next(2,i),marks_d(next_index(i)+1),'VerticalAlignment','bottom','HorizontalAlignment','right');
    plot(pass(1,i),pass(2,i),'o'); hold on;
    text(pass(1,i),pass(2,i),marks_d(pass_index(i)+1),'VerticalAlignment','bottom','HorizontalAlignment','right');
    plot(P(1,i),P(2,i),'k*'); hold on;
    text(P(1,i),P(2,i),marks_a(i+1),'VerticalAlignment','bottom','HorizontalAlignment','right'); hold on;
    
    %Calculate CTE
    e(i) = cte(W(:,next_index(i)),W(:,next_index(i)+1),P(:,i));
end
figure; subplot 411; plot(Actual.x); title('x'); subplot 412; plot(Actual.y); title('y');
subplot 413; plot(e); title('CTE'); subplot 414; plot(Desire.x,Desire.y);


% %Vehicle model
% theta = 0; x = init(1); y = init(2);
% phi = -30*pi/180;
% %Set time
% ti = 0; tf = p; ts = 1;
% %initial time = 0s, final time = 1699s, sample time = 1s
% t = ti:ts:tf;
% 
% [T,X] = ode45(@motion,t,[theta x y phi]);
% figure;  plot(X(:,2),X(:,3));
% 
