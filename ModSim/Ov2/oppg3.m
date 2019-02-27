syms y(t);

ode = int(diff(y,t)^3) == t*y;
ySol(t) = dsolve(ode)