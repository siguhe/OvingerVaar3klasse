%% Assignment 7, Problem 3 b)
%  MPC and state feedback
%  Tor Aksel N. Heirung, April 2013

k1 = 1;
k2 = 1;
k3 = 1;
T = 0.1;

% Continuous time:
Ac = [  0    1 
      -k1  -k2 ];
Bc = [0  k3]';

% Discrete time:
A = eye(2) + Ac*T;
B = Bc*T;

N = 10; % Length of time horizon
nx1 = size(A,2); % number of states (equals the number of rows in A)
nu = size(B,2); % number of controls (equals the number of rows in B)

% Cost function
I_N = eye(N);
Qt = diag([4 4]);
Q = kron(I_N, Qt);
Rt = 1;
R = kron(I_N, Rt);
G = blkdiag(Q, R);

% Equality constraint
Aeq_c1 = eye(N*nx1);                         % Component 1 of A_eq
Aeq_c2 = kron(diag(ones(N-1,1),-1), -A);    % Component 2 of A_eq
Aeq_c3 = kron(I_N, -B);                     % Component 3 of A_eq
Aeq = [Aeq_c1 + Aeq_c2, Aeq_c3];

% Inequality constraint
x1_lb = -Inf(N*nx1,1);    % Lower bound on x1
x1_ub =  Inf(N*nx1,1);    % Upper bound on x1
u_lb = -4*ones(N*nu,1); % Lower bound on u
u_ub =  4*ones(N*nu,1); % Upper bound on u
lb = [x1_lb; u_lb];      % Lower bound on z
ub = [x1_ub; u_ub];      % Upper bound on z

%% MPC

opt = optimset('Display','off', 'Diagnostics','off', 'LargeScale','off', 'Algorithm', 'active-set');

tf = 50; % Final time step
x1 = NaN(2,tf+1);
u = NaN(1,tf+1);
y = NaN(1,tf+1);

x10     = [5, 1]'; % Initial state

x1(:,1) = x10;

% Initialize beq
beq = [zeros(nx1,1); zeros((N-1)*nx1,1)];

for t = 1:tf
    
    % Update equality constraint with latest state
    beq(1:nx1) = A*x1(:,t);
    
    % Solve optimization problem
    [z,fval,ex1itflag,output,lambda] = quadprog(G,[],[],[],Aeq,beq,lb,ub,[],opt);
    
    % Ex1tract optimal control from solution z
    u_ol = z(N*nx1+1:N*nx1+N*nu);    % Control, open-loop optimal
    u(t) = u_ol(1); % Only first element is used
    
    % Simulate system one step ahead
    x1(:,t+1) = A*x1(:,t) + B*u(t);
    
end

%% Plot
t_vec = 0:tf; % Time vector

% Plot optimal trajectory
figure(1);
subplot(2,1,1);
plot(t_vec, x1, 'k--', 'linewidth', 2);
plot(t_vec, x, 'k--', 'linewidth', 2);
hleg = legend('x1_1(t)', 'x1_2(t)', 'x1_3(t)', 'x1_4(t)');
grid('on');
box1('on');
ylim([-4, 8]);
ylabel('x1(t)');
subplot(2,1,2);
plot(t_vec,u);
box1('on');
grid('on');
ylim([-4, 4]);
ylabel('u_t');
x1label('t');

