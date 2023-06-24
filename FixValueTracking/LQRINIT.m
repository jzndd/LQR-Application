% 设置 state - space
A = [0 1;10 0];
B = [0 -1]';
C = [1 0];
D = 0;

% 设置 Q(for x) R(for u)

%% NORMAL
Q0 = [100 0 ;0 1];
R0 = 1;
K0 = lqr(A,B,Q0,R0);

%% LARGE Q
Q1 = [1000 0 ;0 10];
R1 = 1;
K1 = lqr(A,B,Q1,R1);

%% LARGE R
Q2 = [100 0 ;0 1];
R2 = 100;
K2 = lqr(A,B,Q2,R2);