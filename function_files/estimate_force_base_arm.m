function [force, forceLPF] = estimate_force_base_arm(torques,allJacobians,...
                            jacobiansFeet, timeVec, endInd, fc)

%   Estimation of forces acting on arm links and base
%   This script only uses one arm Jacobian, but it can be adjusted for any Jacobian

    %% estimate force

    % pre allocation
    force = cell(1,2);
    for h = 1:2 % loop over all jacobians
        force{1,h} = zeros(3, endInd);
    end
    stackedFeetJacobian = zeros(12, 24);

    for h = 1:2 % loop over all jacobians
        for i = 1:endInd 

            % stack colliding body jacobian and feet jacobians
            jacobianColBody = allJacobians{1,3*h}(i,:); 
            jacobianFourFeet = jacobiansFeet(i,:); 
            for j = 1:12
                stackedFeetJacobian(j,:) = jacobianFourFeet((24*(j-1)+1):(24*j));
            end
            stackedJacobian = [stackedFeetJacobian; 
                            jacobianColBody(1:24); jacobianColBody(25:48); jacobianColBody(49:72); 
                            jacobianColBody(73:96); jacobianColBody(97:120); jacobianColBody(121:144)];

            % estimate stacked force vector feet and extract force on colliding body
            stackedForces = pinv(stackedJacobian')*torques(i,:)'; 
            force{1,h}(:,i) = stackedForces(13:15); 
        end
    end
    
    %% apply LPF to force during trotting
   
    sysLPF = tf([0.0, fc*2*pi], [1.0, fc*2*pi]);
    forceLPF = cell(1,2);
    for h = 1:2
        forceLPF{1,h}(1,:) = lsim(sysLPF,force{1,h}(1,:),timeVec);
        forceLPF{1,h}(2,:) = lsim(sysLPF,force{1,h}(2,:),timeVec);
        forceLPF{1,h}(3,:) = lsim(sysLPF,force{1,h}(3,:),timeVec);
    end  

end

