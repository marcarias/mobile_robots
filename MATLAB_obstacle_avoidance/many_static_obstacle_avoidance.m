clear all;
hold off;
close all;

% 1)
% ===========PLOT ESCENARI INICIAL==============
q_robot = [9,2];
q_goal = [2,9];
q_obstacle1 = [6,2; 5,2; 5,3; 6,3];
q_obstacle2 = [4,4; 3,4; 3,5; 4,5];
q_obstacle3 = [5,6; 4,6; 4,7; 5,7];
v_init = [-1,1];

figure;
p=patch(q_obstacle1(:,1),q_obstacle1(:,2),'black');
hold on;
patch(q_obstacle2(:,1),q_obstacle2(:,2),'black');
hold on;
patch(q_obstacle3(:,1),q_obstacle3(:,2),'black');
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
q_obstacle1 = [6,2; 5,2; 5,3; 6,3];
q_obstacle2 = [4,4; 3,4; 3,5; 4,5];
q_obstacle3 = [5,6; 4,6; 4,7; 5,7];
v_robot = [-1,1];
t=0;
dt=0.5;

k=3;
ao = 1;
bo = 0.5;
do = 1;

F_tot = [0,0];

iter = 0;

f_obs_store = zeros(6000,6);
d_obs1_store = zeros(6000,2);

while (norm(q_robot - q_goal) > 0.1) && ishghandle(p)
    iter = iter + 1;

    distances1 = - (q_obstacle1 - q_robot); 
    distances_norm1 = 1e6*ones(4,1);
    for i=1:length(distances_norm1)
        if norm(distances1(i,:)) == 0
            distances_norm1(i,:) = 1e-6;
        else
            distances_norm1(i) = norm(distances1(i,:));
        end
    end
    [value,index1] = min(distances_norm1);
    d_ro1 = distances1(index1,:);
    norm_d_ro1 = norm(distances1(index1,:));
    f_obs1 = ao*exp((do-norm_d_ro1)/bo)*(d_ro1/norm_d_ro1);

    distances2 = -(q_obstacle2 - q_robot); 
    distances_norm2 = 1e6*ones(4,1);
    for i=1:length(distances_norm2)
        if norm(distances2(i,:)) == 0
            distances_norm2(i,:) = 1e-6;
        else
            distances_norm2(i) = norm(distances2(i,:));
        end
    end
    [~,index2] = min(distances_norm2);
    d_ro2 = distances2(index2,:);
    norm_d_ro2 = norm(distances2(index2,:));
    f_obs2 = ao*exp((do-norm_d_ro2)/bo)*(d_ro2/norm_d_ro2);
    
    distances3 = -(q_obstacle3 - q_robot); 
    distances_norm3 = 1e6*ones(4,1);
    for i=1:length(distances_norm3)
        if norm(distances3(i,:)) == 0
            distances_norm3(i,:) = 1e-6;
        else
            distances_norm3(i) = norm(distances3(i,:));
        end
    end
    [~,index3] = min(distances_norm3);
    d_ro3 = distances3(index3,:);
    norm_d_ro3 = norm(distances3(index3,:));
    f_obs3 = ao*exp((do-norm_d_ro3)/bo)*(d_ro3/norm_d_ro3);

    v_goal = ((q_goal-q_robot))/norm((q_goal-q_robot));
    v_robot = v_robot + F_tot*dt;
    f_goal = k*(v_goal - v_robot);
    
    f_obs = f_obs1 + f_obs2 + f_obs3;
    f_obs_store(iter,1:2) = f_obs1;
    f_obs_store(iter,3:4) = f_obs2;
    f_obs_store(iter,5:6) = f_obs3;
    d_obs1_store(iter,:) = d_ro1;
    
    F_tot = f_goal + f_obs;
    q_robot = q_robot + v_robot*dt + 0.5*F_tot*(dt^2);

    pause(0.25);
    if iter > 1
        plot(q_robot(1),q_robot(2),'yo',MarkerSize=10,MarkerEdgeColor='black',LineWidth=1,MarkerFaceColor='yellow');
        hold on;
    end
end
