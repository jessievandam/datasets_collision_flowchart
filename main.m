close all;
clear all;

%% choose dataset number
datasetNr = 1;

%% initialization

% add folders to path
addpath('function_files')
addpath('datasets')

% load dataset
load('dataset'+string(datasetNr));

%% parameters time

endInd = size(time,2);    % end index
Ts = time(end)/(endInd);  % sampling time [sec]
timeVec = linspace(0, time(end), endInd); % linearly spaced time vector

%% estimate external torques
% torques calculated with continuous-time momentum-based observer

% estimate torques
torques = momentum_observer(taum, nonlinearTerms, massMatrix, qd, Ts, endInd);

%% estimate external force
% force{1,1} is arm force, force{1,2} is base force

% cut-off frequency LPF force, to be applied during trotting
fc = 1.0; % [Hz] 
                          
% for the datasets for which forceEE is not recorded: set forcEE to empty
if datasetNr ~= 5 && datasetNr ~= 8 && datasetNr ~= 10 && datasetNr ~= 11 && ...
    datasetNr ~= 12 && datasetNr ~= 13 && datasetNr ~= 14 && datasetNr ~= 19
    forceEE = [];
end

% estimate force
[force, forceLPF] = estimate_force_base_arm(torques, jacobiansCollidingLinks,...
                            jacobiansFeet, timeVec, endInd, fc, forceEE);
                        
%% detect collision     

% parameters collision detection flowchart
% T_twopeaks: if the second peak doesn't appear after T_twopeaks sec, the ending of the collision is detected
% T_rippling: if all force components are below the threshold for T_rippling sec after the collision has ended,
%             the collision has disappeared
T_twopeaks = 2.5;  % [sec] 
T_rippling = 0.6;  % [sec] 
                       
% start index to start detecting collision
startInd = 1000;
                       
% cut-off frequencies of band-pass filter
if datasetNr == 18
    % cut-off frequencies during trotting
    cutOffFreqMin = 0.1;  % [Hz] HPF cut-off freq
    cutOffFreqMax = 1.5;  % [Hz] LPF cut-off freq
else
    % cut-off frequencies when not trotting
    cutOffFreqMin = 0.4;  % [Hz] HPF cut-off freq
    cutOffFreqMax = 3.0;  % [Hz] LPF cut-off freq
end

% constant threshold
if datasetNr == 18
    % constant threshold during trotting
    constThresh = [15.8; 15.6; 7.9]; % [N]
elseif datasetNr == 6 || datasetNr == 7 || datasetNr == 14 || datasetNr == 15 || ...
    datasetNr == 17 || datasetNr == 19 
    % constant threshold during arm motion   
    constThresh = [4; 3; 6.5];  % [N]
else
    % constant threshold during stance
    constThresh = [1.8; 1.8; 1.8]; % [N]
end

% detect collision
[collision] = collision_detection(force, timeVec, endInd, Ts, T_twopeaks,...
                   T_rippling, cutOffFreqMin, cutOffFreqMax, constThresh, startInd);
                   
%% estimate disturbances and identify collision force

% cut-off frequency LPF for estimating disturbances [Hz]
fc = 0.5; 

% switch forceEstimated to forceLPF for trotting
if datasetNr == 18
    % during trotting
    forceEstimated = forceLPF;
else
    % when not trotting
    forceEstimated = force;
end

% identify collision
[forceCollision, magEstForceCollision, disturbance] = collision_identification(collision,...
                forceEstimated, endInd, Ts, fc);

%% plot identified vs ground truth collision force

% figure parameters
LW = 1.5;  % line width
FS = 20;   % font size
x00 = 10;
y00 = 10;
width = 1500;
height = 700;

% choose colliding body part to plot
if datasetNr == 3 || datasetNr == 5 || datasetNr == 15 || datasetNr == 17 
    % for base collisions
    jacobian = 2;
else
    % for arm collisions
    jacobian = 1;
end
  
% plot magnitude of collision force and detection bool
figure()
set(gcf,'position',[x00,y00,width,height])
plot(time, magEstForceCollision{1, jacobian}, 'b', 'LineWidth', LW)
hold on
plot(time, 10*collision{1,jacobian}, 'r:', 'LineWidth', 2.0)
if datasetNr ~= 14
    % dataset 14 contains collisions with human arm and does not have a
    % ground truth F/T sensor force
    hold on
    plot(time, magFTForce, 'k--', 'LineWidth', LW)
end
hold off
grid on
ylabel('Force magnitude [N]','Interpreter','latex','Fontsize', FS)
xlabel('Time [sec]','Interpreter','latex','Fontsize', FS)
leg = legend('Estimated force','Collision?', 'Ground truth');
set(leg, 'Location', 'northeast',  'Interpreter', 'latex','Fontsize', FS);
xlim([time(500) time(end)])   
