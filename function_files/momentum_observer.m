function [torques] = momentum_observer(taum, nonlinearTerms, massMatrix, qd, Ts, endInd)

%   Momentum based observer

    % observer gain matrix
    gains = [15, 15, 3, 15, 3, 15,... % base
            40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,... % legs
            10, 10, 10, 10, 10, 10];  % arm
    K_O = diag(gains);  
    
    % actuator selection matrix
    S_transposed = zeros(24,18);
    S_transposed(7:24,1:18) = eye(18,18);
    
    % initialize
    torques = zeros(endInd,24);
    nonInertialTerms = zeros(24,1);
    
    % calculate torques
    for i = 2:endInd
        % compute non inertia terms
        newNonInertialTerms = S_transposed * taum(i,:)' - nonlinearTerms(i,:)' + torques(i-1,:)'; 
        % integrate
        nonInertialTerms = nonInertialTerms + newNonInertialTerms * Ts;
        % estimated external torques
        torques(i,:) = K_O * (massMatrix{i,1} * qd(i,:)' - nonInertialTerms); 
    end

end