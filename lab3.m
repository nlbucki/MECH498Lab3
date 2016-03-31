%% simulateRR testing
RR = RRInit();
% drawRR([0 pi/2], RR);
simulateRR();

%% robInit testing
rob = robInit();
drawRob([0 0 0],rob);

%% creatRobTrajectory testing
rob = robInit();
via = [500 600 700 500; 0 0 0 0; 2000 1750 1250 1000];
createRobTrajectory(via, rob)