%% Mutilayer weld design
% Written by: Maggie XU
% Date: 29 oct 2020

% Note:
% this code is built partially based on the reference
% reference: Multi-pass path planning for thick plate by DSAW based on vision sensor, avaliable at sci-hub.st/10.1108/SR-04-2013-649

% This method:
% plans the welding layers and sequence for V-shape groove
% works for V-shape groove only
% welding bead geometry has been approximated into parallelograms or trapezoids
% use side to center, left-to-right approach to plan the beads

%input: 1.thickness           h (mm)
%       2.groove angle        beta (degree)
%       3.assembly clearance  g(mm)
%       4.bead thickness      t(mm)

%output: 1.number of layer
%        2.all weld points with pose in 2d, in proper sequence order

clc
clf
clear

%% start

%% 1.plot the V profile

h = 16;                  % height(thickness), unit: mm
beta = 30;               % angle in degree [0, 90], the total angle will be 2*beta
g = 1;                   % assembly clearance = g*2, unit mm

A = [-g/2, 0];                 % start point left, assume the bottom of V lies in origin
B = [g/2, 0];                  % start point right, assume the bottom of V lies in origin
C = [h*tand(beta)+g/2, h];     % corner point right
D = [-h*tand(beta)-g/2, h];    % corner point left

figure(1)
subplot(2, 3, 1);
title("1.V-shape groove", 'FontSize',15) 
hold on;
plot([A(1) B(1)], [A(2) B(2)], 'k')
plot([A(1) D(1)], [A(2) D(2)], 'k')
plot([B(1) C(1)], [B(2) C(2)], 'k')

%% 2.get the layers for welding

% welding parameter information
aH = 1; % deposition coefficient
v1 = 1; % wire feed rate
d = 1;  % wire diameter
v2 = 1; % welding speed
m = 1;  % number of the weld beads contained in L layer

% derived param for layer planning
S = aH*pi*v1*(d^2)/(4*v2);     % cross-sectional area of the weld bead, S=t^2*tand(beta)+2g
t = sqrt(S-g/tand(beta));      % the thickness of each layer
t = h/7;                       % for simplicity
S = (tand(beta)*(t)+g)*t;      % cross-sectional area of the weld bead
K = floor(h/t);                % number of layer

% plot the layers
subplot(2, 3, 2);
title("2.Plan the layers", 'FontSize',15)
hold on;
plot([A(1) B(1)], [A(2) B(2)], 'k')
plot([A(1) D(1)], [A(2) D(2)], 'k')
plot([B(1) C(1)], [B(2) C(2)], 'k')
for i = 1:1:K;
  y = i*t;
  right_x = tand(beta)*y+g/2;
  left_x = -right_x;
  plot([left_x right_x], [y y], 'k')
end



%% 3.segmentatzatize the layer

L = @(m) 2*tand(beta)*t*m+g;        % layer length
l = S/t;                            %length of the weld bead in parallelograms                 
LN = ceil((L(K)+L(K-1))/(L(1)+g));  % largest number of beads in one layer
vl=[tand(beta)*t -t];               % vector for left point drawing
vr=[-tand(beta)*t -t];              % vector for right point drawing

% plot segmentatzation
subplot(2, 3, 4);
title("3.Segmentatzation", 'FontSize',15)
hold on;
plot([A(1) B(1)], [A(2) B(2)], 'k')
plot([A(1) D(1)], [A(2) D(2)], 'k')
plot([B(1) C(1)], [B(2) C(2)], 'k')
plot([A(1) B(1)], [A(2) B(2)], 'k')
plot([A(1) D(1)], [A(2) D(2)], 'k')
plot([B(1) C(1)], [B(2) C(2)], 'k')
for i = 1:1:K;
  y = i*t;
  right_x = tand(beta)*y+g/2;
  left_x = -right_x;
  plot([left_x right_x], [y y], 'k')
end

% constrcut weld points
for i = 1:1:K
  y = i*t;
  xr = tand(beta)*y+g/2;
  xl = -xr;
%  N = (L(i)+L(i-1))/(L(1)+g)
  N = round((L(i)+L(i-1))/(L(1)+g));
  if N>1
    lp = floor(N/2);
    rp = N-lp-1;
    trap = 1;
    for j = 1:1:lp;
      x = xl+j*l;
      z(i,j,:) = [x y];
%      plot(x, y, 'r*', 'markersize', 10)
      temp = [x+vl(1) y+vl(2)];
      plot([x,temp(1)], [y, temp(2)], 'k')
    endfor
    for j = lp+1:1:N-1;
      x = xr - (N-j)*l;
      z(i,j,:) = [x y];
%      plot(x, y, 'r*', 'markersize', 10)
      temp = [x+vr(1) y+vr(2)];
      plot([x,temp(1)], [y, temp(2)], 'k')
    endfor
  else
    lp = 0; 
    rp=0;
    trap=1;
  endif
endfor



%% 4.plan the sequence for segmentatzation

subplot(2, 3, 5);
title("4.Sequence planning", 'FontSize',15)
hold on;
plot([A(1) B(1)], [A(2) B(2)], 'k')
plot([A(1) D(1)], [A(2) D(2)], 'k')
plot([B(1) C(1)], [B(2) C(2)], 'k')
plot([A(1) B(1)], [A(2) B(2)], 'k')
plot([A(1) D(1)], [A(2) D(2)], 'k')
plot([B(1) C(1)], [B(2) C(2)], 'k')
for i = 1:1:K;
  y = i*t;
  right_x = tand(beta)*y+g/2;
  left_x = -right_x;
  plot([left_x right_x], [y y], 'k')
end

% constrcut points for each segmentation
for i = 1:1:K
  y = i*t;
  xr = tand(beta)*y+g/2;
  xl = -xr;
  N = round((L(i)+L(i-1))/(L(1)+g));
  if N>1
    lp = floor(N/2);
    rp = N-lp-1;
    trap = 1;
    for j = 1:1:lp;
      x = xl+j*l;
      z(i,j,:) = [x y];
%      plot(x, y, 'r*', 'markersize', 10)
      temp = [x+vl(1) y+vl(2)];
      plot([x,temp(1)], [y, temp(2)], 'k')
%      text(x,y,num2str(1), 'FontSize',18, 'HorizontalAlignment','right', 'VerticalAlignment','top')
    endfor
    for j = lp+1:1:N-1;
      x = xr - (N-j)*l;
      z(i,j,:) = [x y];
%      plot(x, y, 'r*', 'markersize', 10)
      temp = [x+vr(1) y+vr(2)];
      plot([x,temp(1)], [y, temp(2)], 'k')
    endfor
  else
    lp = 0; 
    rp=0;
    trap=1;
  endif
endfor



%% 5.get the pose for welding (position and orientation, represneted in vector form)

w = zeros(K,LN,2);                              % set of weld points, ith layer, jth weld segmentation
vl=[tand(beta)*t-l -t];                         % vector for left point projection
vr=[-tand(beta)*t+l -t];                        % vector for right point drawing
length = t/2;                                   % length of arrow to represnt weld in the plot
dl = length*[sind(45-beta/2) cosd(45-beta/2)];  % orientation of weld point for left segmentation
dv = length*[-cosd(45+beta/2) sind(45+beta/2)]; % orientation of weld point for right segmentation
number = 1;                                     % for arranging sequece

for i = 1:1:K
  y = i*t;
  xr = tand(beta)*y+g/2;
  xl = -xr;
  N = round((L(i)+L(i-1))/(L(1)+g));
  k = N-1;
  if N == 1
    lp = 0; 
    rp=0;
    trap=1;
    temp = [0 0];
    w(i,1,:) = [0 0];
    text(0, 0, num2str(number), 'FontSize',10, 'VerticalAlignment','bottom','HorizontalAlignment','center') 
    number = number+1;
  elseif N == 2
    x = xl+l;
    temp = [x+vl(1), y+vl(2)];
    w(i,1,:) = temp;
    text(temp(1), temp(2), num2str(number), 'FontSize',10, 'VerticalAlignment','bottom') 
    number = number+1;
    temp(1) = l/2;
    w(i,2,:) = temp;
    text(temp(1), temp(2), num2str(number), 'FontSize',10, 'VerticalAlignment','bottom') 
    number = number+1;
  elseif N>2
    lp = floor(N/2);
    rp = N-lp-1;
    trap = 1;
    for j = 1:1:lp;
      x = xl+j*l;
      temp = [x+vl(1) y+vl(2)];
      w(i,j,:) = temp;
      text(temp(1), temp(2), num2str(number), 'FontSize',10, 'VerticalAlignment','bottom') 
      number = number+1;
    endfor
    for j = lp+1:1:N-1;
      x = xr - (N-k)*l;
      temp = [x+vr(1) y+vr(2)];
      w(i,j,:) = temp;
      text(temp(1), temp(2), num2str(number), 'FontSize',10, 'VerticalAlignment','bottom', 'HorizontalAlignment','right') 
      number = number+1;
      k = k-1;
    endfor
    for j = N:N
      temp = (w(i,lp,:) + w(i,N-1,:))/2;
      w(i,j,:) = temp;
      text(temp(1), temp(2), num2str(number), 'FontSize',10, 'VerticalAlignment','bottom',  'HorizontalAlignment','center') 
      number = number+1;
     endfor
  endif
endfor
point_number = number-1;

%plot previous plots
subplot(2, 3, 6);
title("5.Weld points pose", 'FontSize',15)
hold on;
plot([A(1) B(1)], [A(2) B(2)], 'k')
plot([A(1) D(1)], [A(2) D(2)], 'k')
plot([B(1) C(1)], [B(2) C(2)], 'k')
plot([A(1) B(1)], [A(2) B(2)], 'k')
plot([A(1) D(1)], [A(2) D(2)], 'k')
plot([B(1) C(1)], [B(2) C(2)], 'k')
for i = 1:1:K;
  y = i*t;
  right_x = tand(beta)*y+g/2;
  left_x = -right_x;
  plot([left_x right_x], [y y], 'k')
end
vl=[tand(beta)*t -t]; % vector for left point drawing
vr=[-tand(beta)*t -t]; % vector for right point drawing
%% constrcut points
for i = 1:1:K
  y = i*t;
  xr = tand(beta)*y+g/2;
  xl = -xr;
  N = round((L(i)+L(i-1))/(L(1)+g));
  if N>1
    lp = floor(N/2);
    rp = N-lp-1;
    trap = 1;
    for j = 1:1:lp;
      x = xl+j*l;
      z(i,j,:) = [x y];
%      plot(x, y, 'r*', 'markersize', 10)
      temp = [x+vl(1) y+vl(2)];
      plot([x,temp(1)], [y, temp(2)], 'k')
%      text(x,y,num2str(1), 'FontSize',18, 'HorizontalAlignment','right', 'VerticalAlignment','top')
    endfor
    for j = lp+1:1:N-1;
      x = xr - (N-j)*l;
      z(i,j,:) = [x y];
%      plot(x, y, 'r*', 'markersize', 10)
      temp = [x+vr(1) y+vr(2)];
      plot([x,temp(1)], [y, temp(2)], 'k')
    endfor
  else
    lp = 0; 
    rp=0;
    trap=1;
  endif
endfor

% plot the arrows
w = zeros(K,LN,2);
welding_points = [];
vl=[tand(beta)*t-l -t]; % vector for left point projection
vr=[-tand(beta)*t+l -t]; % vector for right point drawing
length = l/2;
color = 'c';
dl = length*[sind(45-beta/2) cosd(45-beta/2)];
dv = length*[-cosd(45+beta/2) sind(45+beta/2)];
for i = 1:1:K
  y = i*t;
  xr = tand(beta)*y+g/2;
  xl = -xr;
  N = round((L(i)+L(i-1))/(L(1)+g));
  k = N-1;
  if N == 1
    lp = 0; 
    rp=0;
    trap=1;
    temp = [0 0];
    w(i,1,:) = [0 0];
    plot(0, 0, 'bo', 'markersize', 8) 
    tempy(1) = temp(1)+0;
    tempy(2) = temp(2)+length;
    plot([temp(1) tempy(1)],[temp(2) tempy(2)],'Linewidth',2, 'color','r')
    welding_pt = [temp 0 length];
    welding_points = [welding_points welding_pt];
    number = number+1;
  elseif N == 2
    x = xl+l;
    temp = [x+vl(1), y+vl(2)];
    w(i,1,:) = temp;
    plot(temp(1), temp(2), 'bo', 'markersize', 8)
    number = number+1;
    tempy(1) = temp(1)+dl(1);
    tempy(2) = temp(2)+dl(2);
    plot([temp(1) tempy(1)],[temp(2) tempy(2)],'Linewidth',2, 'color','r')
    welding_pt = [temp dl(1) dl(2)];
    welding_points = [welding_points welding_pt];
    temp(1) = l/2;
    w(i,2,:) = temp;
    plot(temp(1), temp(2), 'bo', 'markersize', 8)
    number = number+1;
    tempy(1) = temp(1)+0;
    tempy(2) = temp(2)+length;
    plot([temp(1) tempy(1)],[temp(2) tempy(2)],'Linewidth',2, 'color','r')
    welding_pt = [temp 0 length];
    welding_points = [welding_points welding_pt];
  elseif N>2
    lp = floor(N/2);
    rp = N-lp-1;
    trap = 1;
    for j = 1:1:lp;
      x = xl+j*l;
      temp = [x+vl(1) y+vl(2)];
      w(i,j,:) = temp;
      plot(temp(1), temp(2), 'bo', 'markersize', 8)
      number = number+1;
      tempy(1) = temp(1)+dl(1);
      tempy(2) = temp(2)+dl(2);
      plot([temp(1) tempy(1)],[temp(2) tempy(2)],'Linewidth',2, 'color','r')
      welding_pt = [temp dl(1) dl(2)];
      welding_points = [welding_points welding_pt];
    endfor
    for j = lp+1:1:N-1;
      x = xr - (N-k)*l;
      temp = [x+vr(1) y+vr(2)];
      w(i,j,:) = temp;
      plot(temp(1), temp(2), 'bo', 'markersize', 8) 
      number = number+1;
      k = k-1;
      tempy(1) = temp(1)+dv(1);
      tempy(2) = temp(2)+dv(2);
      plot([temp(1) tempy(1)],[temp(2) tempy(2)],'Linewidth',2, 'color','r')
      welding_pt = [temp dv(1) dv(2)];
      welding_points = [welding_points welding_pt];
    endfor
    for j = N:N
      temp = (w(i,lp,:) + w(i,N-1,:))/2;
      w(i,j,:) = temp;
      plot(temp(1), temp(2), 'bo', 'markersize', 8) 
      number = number+1;
      tempy(1) = temp(1)+0;
      tempy(2) = temp(2)+length;
      plot([temp(1) tempy(1)],[temp(2) tempy(2)],'Linewidth',2, 'color','r')
      welding_pt = [temp(1) temp(2) 0 length];
      welding_points = [welding_points welding_pt];
    endfor
  endif
endfor

welding_points =  reshape(welding_points,4,point_number);

%information of this plot
txt0 = ['V-shape groove parameter'];
line = ['   '];
txt1 = ['Thickness: ' num2str(h) ' mm'];
txt2 = ['Angle:  ' num2str(beta) ' degree'];
txt3 = ['Assembly clearance: ' num2str(g) ' mm'];
txt4 = ['Bead thickness : ' num2str(t) ' mm']; 
txt5 = ['Number of layer: ' num2str(K)];
txt6 = ['Number of weld points: ' num2str(point_number)];
txt = {txt0, line, txt1, txt2, txt3, txt4, txt5, txt6};
annotation('textbox',[0.75,0.6,0.3,0.3],'String',txt,'FitBoxToText','on','FontSize', 18);
%% end