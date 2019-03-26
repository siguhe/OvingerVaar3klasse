k_1=1;
k_2=1;
k_3=1;
T=0.1;
x_0=[5;1];
x_0hat=[6;0];

A=[0 1;
    -k_2 -k_1];
B=[0 k_3]

Ad=[1 T;
    -k_2*T 1-k_1*T];
Bd=[0;k_3*T];
C=[1 0];
D=0;

Q=eye(2)*4;
R=1;

K=dlqr(Ad,Bd,Q,R,[])

eig(Ad-Bd*K)

