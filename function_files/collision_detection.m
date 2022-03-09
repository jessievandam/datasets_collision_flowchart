function [collision] = collision_detection(force, timeVec, endInd, Ts, T_twopeaks,...
                       T_rippling, cutOffFreqMin, cutOffFreqMax, constThresh, startInd)

%   Collision detection with BPF and constant threshold
    
    %% add BPF to force, input from Hz to rad/sec
    sysBPF = tf([0.0, cutOffFreqMax*2*pi, 0.0], [1.0, (cutOffFreqMax*2*pi+cutOffFreqMin*2*pi), (cutOffFreqMax*cutOffFreqMin*4*pi*pi)]);
    forceBPF = cell(1,2);
    for h = 1:2 
        for l = 1:3
            forceBPF{1,h}(l,:) = lsim(sysBPF,force{1,h}(l,:),timeVec);
        end   
    end
 
    %% detect collision
  
    % initialize
    collision = cell(1,2);
    for h = 1:2
        collision{1,h} = zeros(1,endInd);
    end
    
    % detect collision
     for h = 1:2      

          % intialization
          signCollision = 0;
          phaseCollision = 0;
          count = 0;

          for i = startInd:endInd

              % PHASE 0: no collision 
              if phaseCollision == 0

                  % if collision is detected
                  if (abs(forceBPF{1,h}(1,i))>constThresh(1) || abs(forceBPF{1,h}(2,i))>constThresh(2) || ...
                          abs(forceBPF{1,h}(3,i))>constThresh(3) )

                      % collision detected
                      collision{1,h}(i) = 1;
                      % move on to next phase of detection flowchart
                      phaseCollision = 1;      

                      % check on which axis and in which direction collision is detected
                      if forceBPF{1,h}(1,i)>constThresh(1)
                          signCollision = 1;
                          axis = 1;
                      elseif forceBPF{1,h}(2,i)>constThresh(2)
                          signCollision = 1;
                          axis = 2;
                      elseif forceBPF{1,h}(3,i)>constThresh(3)
                          signCollision = 1;
                          axis = 3;
                      elseif forceBPF{1,h}(1,i)<-constThresh(1)
                          signCollision = -1;
                          axis = 1;
                      elseif forceBPF{1,h}(2,i)<-constThresh(2)
                          signCollision = -1;
                          axis = 2;
                      else
                          signCollision = -1;
                          axis = 3;
                      end

                  % if no collision is detected
                  else
                      collision{1,h}(i) = 0;
                  end

              % PHASE 1: if collision has started but not ended yet
              elseif phaseCollision == 1

                  % collision still lasting although not detected
                  collision{1,h}(i) = 1; 

                  % if collision was detected positive, wait until it is detected negative
                  if signCollision == 1 
                      if (axis == 1 && forceBPF{1,h}(1,i)<-constThresh(1)) ||...
                              (axis == 2 && forceBPF{1,h}(2,i)<-constThresh(2)) ||...
                              (axis == 3 && forceBPF{1,h}(3,i)<-constThresh(3))
                          collision{1,h}(i) = 0;
                          phaseCollision = 2; % after this, collision has ended
                          count = 0; % make sure phase 2 starts with 0 count
                      end

                  % if collision was detected negative, wait until it is detected positive
                  elseif signCollision == -1 
                      if (axis == 1 && forceBPF{1,h}(1,i)>constThresh(1)) ||...
                              (axis == 2 && forceBPF{1,h}(2,i)>constThresh(2)) ||...
                              (axis == 3 && forceBPF{1,h}(3,i)>constThresh(3))
                          collision{1,h}(i) = 0;
                          phaseCollision = 2; % after this, collision has ended
                          count = 0; % make sure phase 2 starts with 0 count
                      end
                  end   

                  % in case the second peak doesn't appear for T_twopeaks full second after the collision has ended
                  % go straight to phase 0 again and skip phase 2
                  if abs(forceBPF{1,h}(1,i))<constThresh(1) && abs(forceBPF{1,h}(2,i))<constThresh(2) && ...
                      abs(forceBPF{1,h}(3,i))<constThresh(3) 

                        % count how long the peaks are below the threshold
                        count = count + 1;
                        if count > (T_twopeaks/Ts)
                            phaseCollision = 0;
                            count = 0;
                        end 
                   else
                       count = 0;
                   end

              % PHASE 2: if end of collision has been detected, but the force is still crossing the threshold
              elseif phaseCollision == 2

                   collision{1,h}(i) = 0;   

                   % detect when all peaks are below threshold for at least T_rippling sec, to prevent additional 
                   % detected collisions during rippling
                   if abs(forceBPF{1,h}(1,i))<constThresh(1) && abs(forceBPF{1,h}(2,i))<constThresh(2) && ...
                      abs(forceBPF{1,h}(3,i))<constThresh(3) 

                        % count how long the peaks are below the threshold
                        count = count + 1;
                        if count > (T_rippling/Ts)
                            phaseCollision = 0;
                            count = 0;
                        end
                   else
                       count = 0;
                   end
              end  

          end
     end

end

