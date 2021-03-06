%-------------------------------------------------------------------------%
%                             Extract Solution                            %
%-------------------------------------------------------------------------%
auxdata.g = 9.81;
auxdata.L1 = 1;
auxdata.L2 = 1;
auxdata.m1 = 1;
auxdata.m2 = 1;

solution = output.result.solution;
time = solution.phase(1).time;
state = solution.phase(1).state;
control = solution.phase(1).control;

theta1 = state(:, 1);
theta2 = state(:, 2);

X1 = auxdata.L1 * cos(theta1);
Y1 = auxdata.L1 * sin(theta1);
X2 = X1 + auxdata.L2 * cos(theta1 + theta2);
Y2 = Y1 + auxdata.L2 * sin(theta1 + theta2);

h = figure;
hold on;
axis(2 * [-1 1 -1 1]);
for i = 1:length(time)
    plot([0 X1(i)], [0 Y1(i)]);
    plot([X1(i) X2(i)], [Y1(i) Y2(i)]);
    pause(0.02);
    cla;
end

