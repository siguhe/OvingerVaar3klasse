A = [1  0.5;
     0  1  ];
B = [0.125;  0.5];
Q = diag([2 2]);
R = 2;

[K,S,e]=dlqr(A,B,Q,R,0);

S=1/2*S;

eigStab=eig(A-B*K)

% 
% eigStab =
% 
%    0.6307 + 0.1628i
%    0.6307 - 0.1628i
%    => Stable system