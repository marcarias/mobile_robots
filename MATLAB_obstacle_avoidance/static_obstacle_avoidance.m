clear all;
hold off;
close all;

% 1)
% ===========PLOT ESCENARI INICIAL==============
q_robot = [9,2];
q_goal = [2,9];
q_obstacle = [8 5; 7,5; 6,5; 5,5; 5,6; 5,7; 5,8; 6,8; 7,8; 8,8; 8,7; 8,6];
v_init = [-1,1];

figure;
p=patch(q_obstacle(:,1),q_obstacle(:,2),'black');
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


%===========CALCULS
q_robot = [9,2];
q_goal = [2,9];
q_obstacle = [8 5; 7,5; 6,5; 5,5; 5,6; 5,7; 5,8; 6,8; 7,8; 8,8; 8,7; 8,6];
v_robot = [-1,1];
t=0;
dt=0.5;

k=3;
ao = 1;
bo = 0.5;
do = 1;

F_tot = [0,0];

iter = 0;

while (round(norm(q_robot - q_goal)) > 0) && ishghandle(p)

    iter = iter + 1;

    distances = q_robot - q_obstacle; 
    distances_norm = 1e6*ones(12,1);
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

    F_tot = f_goal + f_obs;

    q_robot = q_robot + v_robot*dt + 0.5*F_tot*(dt^2);

    pause(0.25);

    if iter > 1
        plot(q_robot(1),q_robot(2),'yo',MarkerSize=10,MarkerEdgeColor='black',LineWidth=1,MarkerFaceColor='yellow');
        hold on;
    end
end