% Code for making a contour plot for Problem 2.
x1_l = -1.5; x1_h = 6.5;
x2_l = -1.5; x2_h = 6.5;
res = 0.01;
[x1, x2] = meshgrid(x1_l:res:x2_h, x1_l:res:x2_h);
f = -(3-0.4*x1).*x1 - (2-0.2*x2).*x2;
levels = (-12:2:8)';
[C, h] = contour(x1, x2, f, levels, 'Color', .7*[1 1 1]);
set(h, 'ShowText', 'on', 'LabelSpacing', 300); % For text labels
% Changes made to the file qp_prodplan.m
% Quadratic objective (MODIFY THESE)
G = [0.8 0 ;
0 0.4]; % Remember the factor 1/2 in the objective
c = [-3 ; -2];
% Linear constraints (MODIFY THESE)
A = [2 1 ;
1 3];
b = [8 ; 15];

global XIT; % Storing iter  ations in a global variable
global IT;  % Storing number of iterations in a global variable
global x0;  % Used in qp1.m (line 216).

IT=1; XIT=[];

x0 = [0 0]'; % Initial value
vlb = [0 0]; % Lower bound on x
vub = [];    % Upper bound on x

% min 0.5*x'*G*x + x'*c
%  x 
%
% s.t. A*x <= b 


options = optimset('LargeScale','Off');
[x,lambda] = quadprog1(G,c,A,b,[],[],vlb,vub,x0,options);

disp('Iteration sequence:')
disp(XIT');
disp('Solution:')
disp(x);
