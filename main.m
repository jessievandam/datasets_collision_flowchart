%% initialization

% add folders to path
addpath('function_files')
addpath('datasets')

% load dataset
load('dataset9');

%% parameters time
endInd = size(time,2);    % end index
Ts = time(end)/(endInd);  % sampling time
timeVec = linspace(0, time(end), endInd); % linearly spaced time vector

%% estimate external torques
% torques calculated with continuous-time momentum-based observer

% estimate torques
torques = momentum_observer(taum, nonlinearTerms, massMatrix, qd, Ts, endInd);

%% estimate external force
% force{1,1} is arm force, force{1,2} is base force

% cut-off frequency LPF force, to be applied during trotting
fc = 1.0; % [Hz] 
 
[force, forceLPF] = estimate_force_base_arm(torques, jacobiansCollidingLinks,...
                            jacobiansFeet, timeVec, endInd, fc);
                        
%% detect collision     

% parameters collision detection flowchart
T_twopeaks = 3;        % if the second peak doesn't appear after T_twopeaks sec, the ending of the collision is detected
T_rippling = 0.8;      % if all force components are below the threshold for T_rippling sec after the collision has ended,
                       % the collision has officially disappeared

% cut-off frequencies of BPF
cutOffFreqMin = 0.4;  % [Hz] HPF cut-off freq
cutOffFreqMax = 3.0;  % [Hz] LPF cut-off freq
% cut-off frequencies of BPF during trotting
% cutOffFreqMin = 0.1;  % [Hz] HPF cut-off freq
% cutOffFreqMax = 1.5;  % [Hz] LPF cut-off freq

% constant threshold
constThresh = [1.8; 1.8; 1.8];    % stance
% constThresh = [4; 3; 6.5];          % arm motion 
% constThresh = [15.8; 15.6; 7.9];  % trotting

[collision] = collision_detection(force, timeVec, endInd, Ts, T_twopeaks,...
                       T_rippling, cutOffFreqMin, cutOffFreqMax, constThresh);
                   
%% estimate disturbances and identify collision force

% cut-off frequency LPF for estimating disturbances [Hz]
fc = 0.5; 

% switch forceEstimated to forceLPF for trotting
forceEstimated = force;

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

% choose colliding body part to plot: 1 = arm, 2 = base
jacobian = 1;
  
% plot magnitude of collision force and detection bool
figure()
set(gcf,'position',[x00,y00,width,height])
plot(time, magEstForceCollision{1, jacobian}, 'b', 'LineWidth', LW)
hold on
plot(time, 10*collision{1,jacobian}, 'r:', 'LineWidth', 2.0)
hold on
plot(time, magFTForce, 'k--', 'LineWidth', LW)
hold off
grid on
ylabel('Magnitude force [N]','Interpreter','latex','Fontsize', FS)
xlabel('Time [sec]','Interpreter','latex','Fontsize', FS)
leg = legend('Estimated force','Collision?', 'Ground truth');
set(leg, 'Location', 'northeast',  'Interpreter', 'latex','Fontsize', FS);
xlim([time(500) time(end)])   
