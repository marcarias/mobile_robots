clear
close all

% PATH DEFINITION: First of all, we divide the whole path in discrete
% rectilinear sections in order to build a vector [path_x, path_y,
% path_theta] which contains every point travelled by the robot.
% It is important to consider that it has a velocity of 1m/s and we have to
% consider a time step of 0.1s

% TRAM 1
tram1_y = 8:0.1:15;
tram1_x = ones(1,length(tram1_y));
tram1_theta = ones(1,length(tram1_y)).*(pi/2);

% TRAM 2
tram2_x = 1:0.1:9;
tram2_y = ones(1,length(tram2_x)).*tram1_y(end);
tram2_theta = zeros(1,length(tram2_x));

% TRAM 3
tram3_y = flip(8:0.1:15);
tram3_x = ones(1,length(tram3_y)).*tram2_x(end);
tram3_theta = ones(1,length(tram3_y)).*(-pi/2);

% TRAM 4
tram4_x = 9:0.1:15;
tram4_y = ones(1,length(tram4_x)).*tram3_y(end);
tram4_theta = zeros(1,length(tram4_x));

% TRAM 5
tram5_y = flip(1:0.1:8);
tram5_x = ones(1,length(tram5_y)).*tram4_x(end);
tram5_theta = ones(1,length(tram5_y)).*(-pi/2);

% TRAM 6
tram6_x = flip(8:0.1:15);
tram6_y = ones(1,length(tram6_x)).*tram5_y(end);
tram6_theta = ones(1,length(tram6_x)).*(pi);

% TRAM 7 
tram7_y = 1:0.1:6;
tram7_x = ones(1,length(tram7_y)).*tram6_x(end);
tram7_theta = ones(1,length(tram7_y)).*(pi/2);

% TRAM 8
tram8_x = flip(1:0.1:7);
tram8_y = ones(1,length(tram8_x)).*tram7_y(end);
tram8_theta = ones(1,length(tram8_x)).*(pi);

% TRAM 9
tram9_y = 6:0.1:8;
tram9_x = ones(1,length(tram9_y)).*tram8_x(end);
tram9_theta = ones(1,length(tram9_y)).*(pi/2);

% COMPLETE PATH
path_x = [tram1_x, tram2_x, tram3_x, tram4_x, tram5_x, tram6_x, tram7_x, tram8_x, tram9_x];
path_y = [tram1_y, tram2_y, tram3_y, tram4_y, tram5_y, tram6_y, tram7_y, tram8_y, tram9_y];
path_theta = [tram1_theta, tram2_theta, tram3_theta, tram4_theta, tram5_theta, tram6_theta, tram7_theta, tram8_theta, tram9_theta];

% Once we have the whole path, we define the initial position of the robot
x_0 = 1;
y_0 = 9;
theta_0 = pi/2;
pose_0 = [x_0, y_0, theta_0];

% We define the features according to the lidar
depth_lidar = 5.5;    
angle_lidar = 180;
resolution_lidar = 0.1*pi/angle_lidar;

% We load the image according to the scenario given in the statement
img = imread('img3.jpeg');
grayscale_img = rgb2gray(img);
binary_img = grayscale_img < 0.5;
map = binaryOccupancyMap(binary_img,49);

% We create the scan simulator for the lidar sensor
lidar_sensor = rangeSensor;
lidar_sensor.Range = [0 depth_lidar];
lidar_sensor.HorizontalAngle = [-pi/2 pi/2];
lidar_sensor.HorizontalAngleResolution = resolution_lidar;

% Here we specify the total number of particles
n_particles = 5000;

% We create the Monte Carlo localization object with functions that were
% given by MATLAB
montecarlo_localization = monteCarloLocalization(map);
montecarlo_localization.UseLidarScan = true;
montecarlo_localization.ParticleLimits = [n_particles n_particles];
montecarlo_localization.InitialPose = pose_0;
montecarlo_localization.GlobalLocalization = true;
likelyhood_sensor_model = likelihoodFieldSensorModel;
likelyhood_sensor_model.Map = map;
likelyhood_sensor_model.SensorLimits = [0 depth_lidar];
montecarlo_localization.SensorModel = likelyhood_sensor_model;

% We load the odometry of the model
odom = odometryMotionModel;
montecarlo_localization.MotionModel = odom;

% We make a loop in order to compute the real robot pose, the estimated
% robot pose by the Monte Carlo given particles and also the lidar
% obstacles recognition by its scan
% ADVERTISE: Aproximately 20 different figures will be spawned to visualize
% its evolution in "real time simulation".
i = 0;
f = 0;
n=0;
while (f <= length(path_x))
    i = i + 1;
    
    % Real robot pose computation
    pose = [path_x(i), path_y(i), path_theta(i)];
    
    % Lidar scan information
    [range,angle] = lidar_sensor(pose,map);
    lidar_scan = lidarScan(range,angle);
    valid_scan = removeInvalidData(lidar_scan);
    transformed_scan = transformScan(valid_scan,pose);
    
    % Obtantion of the estimated pose of the robot
    [a, estimated_pose, b] = montecarlo_localization([path_x(i), path_y(i), path_theta(i)],valid_scan);
    estimated_orientation = [estimated_pose(1) + cos(estimated_pose(3)), estimated_pose(2) + sin(estimated_pose(3))];
    [part,w] = getParticles(montecarlo_localization);
    
    % As it is demanded in the statement, we show the localization of the
    % robot every 3 seconds, which means 30 steps with 0.1 of time step
    if (f == 0) || (mod(f, 30) == 0) || (f == (length(path_x)-1))
        n=n+1;
        % Plot figure
        pause(0.01);
        figure;
        show(map);
        hold on;
        title(sprintf('Time %d s', f/10));
        xlabel('X (m)');
        ylabel('Y (m)');

        % Plot LIDAR obstacle detection
        plot(transformed_scan);
        hLine = findobj(gca, 'Type', 'Line');
        set(hLine, 'Color', [0, 45, 255]./255);
        
        % Plot the estimated position of the robot
        plot(estimated_pose(1), estimated_pose(2), 'go');
        
        % Plot the Monte Carlo filtered particles
        scatter(part(:,1),part(:,2), w*1000, 'r', 'SizeData', 2)
        
        % Plot the estimated orientation of the robot
        plot([estimated_pose(1),estimated_orientation(1)], [estimated_pose(2),estimated_orientation(2)], 'Color', [22, 192, 0]./255, MarkerSize=5,MarkerEdgeColor='black',LineWidth=2);
        
        % Plot the real robot position
        plot(path_x(i), path_y(i),'ro',MarkerSize=5,MarkerEdgeColor='black',LineWidth=2);
        
        % Plot the real robot travelled path until that moment
        plot(path_x(1:i), path_y(1:i), 'Color', [255, 170, 0]./255, 'LineStyle', '--', LineWidth=2);
        
        % Specify graph limits to avoid resizing while simulating
        xlim([0 17])
        ylim([0 16])
        %filename = sprintf('%d.jpg', n/10); % Nombre del archivo con un número de iteración
        %saveas(gcf, filename); % Guardar la figura actual en formato PNG
    end
    f = f + 1;
end

