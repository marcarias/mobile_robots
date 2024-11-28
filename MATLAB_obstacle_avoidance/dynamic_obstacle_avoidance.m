clear all;
hold off;
close all;

% 1)
% ===========PLOT ESCENARI INICIAL==============
q_robot = [9,2];
q_goal = [2,9];
q_obstacle = [7,2; 6,2; 5,2; 5,3; 4,3; 4,4; 3,4; 3,5; 4,5; 5,5; 5,4; 6,4; 6,3; 7,3];
q_pers = [3,8];
v_robot = [-1,1];
v_pers = [1,-1];
t=0;
dt=0.5;

figure;
p=patch(q_obstacle(:,1), q_obstacle(:,2), 'black');
hold on;

axis equal;
xlabel('x (m)');
ylabel('y (m)');
xlim([0 9])
ylim([0 11])
grid on;
camroll(-90);
set(gca,'ytick',0:11);
hold on;

plot(q_goal(:,1),q_goal(:,2),'rs',MarkerSize=20,MarkerEdgeColor='red',LineWidth=3);
hold on;

plot(q_robot(:,1),q_robot(:,2),'go',MarkerSize=20,MarkerEdgeColor='black',LineWidth=3,MarkerFaceColor='green');
hold on;

plot(q_pers(1),q_pers(2),'yo',MarkerSize=20,MarkerEdgeColor='black',LineWidth=1,MarkerFaceColor='blue');
hold on;

%===========CALCULS
q_robot = [9,2];
q_goal = [2,9];
q_obstacle = [7,2; 6,2; 5,2; 5,3; 4,3; 4,4; 3,4; 3,5; 4,5; 5,5; 5,4; 6,4; 6,3; 7,3];
q_pers = [3,8];
v_robot = [-1,1];
v_pers = [1,-1];
t=0;
dt=0.2;

k=3;
ao = 1;
bo = 0.5;
do = 1;

ap = 1.5;
dp = 1.5;
bp = 1.3;

F_tot = [0,0];

iter = 0;

while (norm(q_robot - q_goal) > 0.1) && ishghandle(p)

    iter = iter + 1;

    distances = q_robot - q_obstacle; 
    distances_norm = 1e6*ones(14,1);
    for i=1:length(distances_norm)
        if norm(distances(i,:)) == 0
            distances_norm(i,:) = 1e-6;
        else
            distances_norm(i) = norm(distances(i,:));
        end
    end
    [value,index] = min(distances_norm);
    d_ro = distances(index,:);
    norm_d_ro = norm(distances(index,:));
    f_obs = ao*exp((do-norm_d_ro)/bo)*(d_ro/norm_d_ro);
    
    v_goal = ((q_goal-q_robot))/norm((q_goal-q_robot));
    v_robot = v_robot + F_tot*dt;
    f_goal = k*(v_goal - v_robot);
    
    distance_p = q_robot - q_pers;
    distance_p_norm = norm(distance_p);
    f_pers = ap*exp((dp-distance_p_norm)/bp)*(distance_p/distance_p_norm);

    F_tot = f_goal + f_obs + f_pers;

    q_robot = q_robot + v_robot*dt + 0.5*F_tot*(dt^2);
    q_pers = q_pers + v_pers*dt;

    pause(0.25);
    plot(q_robot(1),q_robot(2),'yo',MarkerSize=10,MarkerEdgeColor='black',LineWidth=1,MarkerFaceColor='yellow');
    hold on;
    plot(q_pers(1),q_pers(2),'yo',MarkerSize=10,MarkerEdgeColor='blue',LineWidth=1,MarkerFaceColor='blue');
    hold on;
end