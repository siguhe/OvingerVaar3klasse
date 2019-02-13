%% Implicit Euler
%y(n+1)=y(n)+h(f(n),t(n))+h^2*f(y(n),t(n))/dt+O(h^3)
 x_d = 1.32;
 K = 2.40; 
 g = 9.81;

t=10; h=0.01;    
N = round(t/h,0);
time=0:h:t
y0 = [2;0];
sigma = size(y0) + 1;

f = @(y,t) [ y(2); -g*(1-(x_d/y(1))^K) ];

y = zeros(size(y0,1),size(time,2));
opt = optimset('Display','off','TolFun',1e-8);



y(:,1)=1;


for i = 1:N-1
    r = @(ynext) (y(:,i) + h*feval(f, (ynext+y(:,i))/2, time(i+1)+h/2) - ynext);
    y(:,i+1) = fsolve(r, y(:,i), opt);
      E(i)=(m*g/(K-1)) *(x_d^K/(y(1,i)^(K-1)))+m*g*y(1,i)+1/2*m*y(2,i)^2;
end

hold on;
plot(time,E,'red')
%plot(time,y(1,:))
