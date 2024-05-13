function dx = physics(X,params,u)

g = params.g;
m = params.m;
M = params.M;
S = sin(-X(3));
C = cos(X(3));
L = params.L;

% Equations of Motion
x_dot = X(2);
v_dot = (u + (m*L*X(4)*S) + (m*g*S*C))/(M + m*(1 + C*C));
q_dot = X(4);
w_dot = -g*S/L - v_dot*C;

% Generating State Space
dx = [x_dot; v_dot; q_dot; w_dot];

end