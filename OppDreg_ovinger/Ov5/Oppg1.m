r=1;
N=30;
x0=[0;0;1];
A=[0 0 0;
    0 0 1;
    0.1 -0.79 1.78];
B=[1; 0; 0.1];
C=[0 0 1];
Q=[0 0 0;
    0 0 0;
    0 0 2];
R=2
    
nx=size(A,2);
nu=size(B,2);

Aeq1=kron(eye(N/2),[ eye(3) zeros(3); -A eye(3)]);
Aeq2=kron(eye(N),-B);
Aeq=[Aeq1,Aeq2];
Beq=[A*x0; zeros((N-1)*nx,1)];

G=blkdiag(kron(eye(N), Q),kron(eye(N),R));

KKT1=[G -Aeq';
    Aeq zeros(size(Aeq,1))];
KKT2=[zeros(N*(nx+nu),1);Beq];

KKT=KKT1\KKT2;

z = KKT(1:N*(nx+nu));   % Variable vector (KKT_sol includes lambdas)
z=quadprog(G,[],[],[],Aeq,Beq);
y = [x0(3); z(nx:nx:N*nx)]; % y = x3
u = z(N*nx+1:N*nx+N*nu);    % Control
% Time vector
t = 1:N;

% Plot optimal trajectory
figure(1);
subplot(2,1,1);
plot([0,t],y,'-ko'); % Plot on 0 to N
grid('on');
ylabel('y_t')
subplot(2,1,2);
plot(t-1,u,'-ko'); % Plot on 0 to N-1
grid('on');
xlabel('t');
ylabel('u_t');


