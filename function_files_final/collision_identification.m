function [forceCollision, magEstForceCollision, disturbance] = collision_identification(collision,...
                forceEstimated, endInd, Ts, fc)

%   Identify the collision force by subtracting the disturbance
   
    %% estimate disturbance
    
    % intialize 
    alpha = exp(-fc*2*pi*Ts);
    disturbance = cell(1,2);
    runOnce = 1;
    offsetFixed = zeros(3,1);
    for h = 1:2
        disturbance{1,h} = zeros(3,endInd);
    end
    
    for h = 1:2
        for i = 2:endInd
            % if no collision: apply low-pass filter 
            if collision{1,h}(i) == 0
                runOnce = 1;
                disturbance{1,h}(:,i) = alpha*disturbance{1,h}(:,i-1) + (1-alpha)*forceEstimated{1,h}(:,i);
            else
            % if collision: freeze offset
                if runOnce == 1 
                    offsetFixed = disturbance{1,h}(:,i-1);
                    runOnce = 0; 
                end
                disturbance{1,h}(:,i) = offsetFixed;
            end
        end
    end
    
    %% subtract disturbance
    
    % initialize
    forceCollision = cell(1,2);
    for h = 1:2
        forceCollision{1,h} = zeros(3,endInd);
    end
   
    % constantly subtract disturbance
    for h = 1:2   
         for i = 1:endInd  
             forceCollision{1,h}(:,i) =  forceEstimated{1,h}(:,i) -  disturbance{1,h}(:,i);
         end
    end
    
    %% compute magnitude of colliding body forces
    
    % force magnitudes
    magEstForceCollision = cell(1,2);
    for h = 1:2
        magEstForceCollision{1,h} = sqrt(forceCollision{1,h}(1,:).^2 + forceCollision{1,h}(2,:).^2 + forceCollision{1,h}(3,:).^2);
    end
    
end

