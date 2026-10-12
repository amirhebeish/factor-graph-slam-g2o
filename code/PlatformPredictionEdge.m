classdef PlatformPredictionEdge < g2o.core.BaseBinaryEdge
    % PlatformPredictionEdge summary of PlatformPredictionEdge
    %
    % This class stores the factor representing the process model which
    % transforms the state from timestep k to k+1
    %
    % The process model is as follows.
    %
    % Define the rotation vector
    %
    %   M = dT * [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
    %
    % The new state is predicted from 
    %
    %   x_(k+1) = x_(k) + M * [vx;vy;theta]
    %
    % Note in this case the measurement is actually the mean of the process
    % noise. It has a value of 0. The error vector is given by
    %
    % e(x,z) = inv(M) * (x_(k+1) - x_(k))
    %
    % Note this requires estimates from two vertices - x_(k) and x_(k+1).
    % Therefore, this inherits from a binary edge. We use the convention
    % that vertex slot 1 contains x_(k) and slot 2 contains x_(k+1).
    
    properties(Access = protected)
        % The length of the time step
        dT;
    end
    
    methods(Access = public)
        function obj = PlatformPredictionEdge(dT)
            % PlatformPredictionEdge for PlatformPredictionEdge
            %
            % Syntax:
            %   obj = PlatformPredictionEdge(dT);
            %
            % Description:
            %   Creates an instance of the PlatformPredictionEdge object.
            %   This predicts the state from one timestep to the next. The
            %   length of the prediction interval is dT.
            %
            % Outputs:
            %   obj - (handle)
            %       An instance of a PlatformPredictionEdge

            assert(dT >= 0);
            obj = obj@g2o.core.BaseBinaryEdge(3);            
            obj.dT = dT;
        end
       
        function initialEstimate(obj)
            % INITIALESTIMATE Compute the initial estimate of a platform.
            %
            % Syntax:
            %   obj.initialEstimate();
            %
            % Description:
            %   Compute the initial estimate of the platform x_(k+1) given
            %   an estimate of the platform at time x_(k) and the control
            %   input u_(k+1)

            % warning('PlatformPredictionEdge.initialEstimate: implement')
            
            % Step 1: Get x_(k) and print it out to see what it looks like: 

            % Extract the heading angle theta from the state vector

            % Compute the rotation matrix M from the process model M = obj.dT * [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];

            % print out the rotation matrix to see what it looks like
            % disp('Rotation matrix M = '), disp(M)
            
            % TODO - find out correct way to get the control input u
            % u = zeros(3, 1); % placeholder for control input [vx; vy; theta_dot]
            % u = obj.z; 

            x_k = obj.edgeVertices{1}.x; 
            theta = x_k(3); 
            M = obj.dT * [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
            u = obj.measurement(); 
            x_pre_k_1 = x_k + M * u;
            obj.edgeVertices{2}.x = x_pre_k_1;
        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for the edge.
            %
            % Syntax:
            %   obj.computeError();
            %
            % Description:
            %   Compute the value of the error, which is the difference
            %   between the measurement and the parameter state in the
            %   vertex. Note the error enters in a nonlinear manner, so the
            %   equation has to be rearranged to make the error the subject
            %   of the formulat
                       
            % warning('PlatformPredictionEdge.computeError: implement')
            
            % Using the formula from description: e(x,z) = inv(M) * (x_(k+1) - x_(k))
            x_k = obj.edgeVertices{1}.x; 
            theta = x_k(3); 
            A = (1/obj.dT) * [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0;0 0 1];
            x_k_1 = obj.edgeVertices{2}.x; 
            u     = obj.measurement();

            error = A * (x_k_1 - x_k) - u; 
            % do angle wrap correctly for rate residual:
            dtheta = wrapToPi(x_k_1(3) - x_k(3));
            error(3) = (1/obj.dT) * dtheta - u(3);

            obj.errorZ = error;
        end
        
        % Compute the Jacobians
        function linearizeOplus(obj)
            % LINEARIZEOPLUS Compute the Jacobians for the edge.
            %
            % Syntax:
            %   obj.computeError();
            %
            % Description:
            %   Compute the Jacobians for the edge. Since we have two
            %   vertices which contribute to the edge, the Jacobians with
            %   respect to both of them must be computed.
            %

            % warning('PlatformPredictionEdge.linearizeOplus: implement')
            x_k = obj.edgeVertices{1}.x;
            x_k_1 = obj.edgeVertices{2}.x;
            x1 = x_k(1);
            y1 = x_k(2);
            theta = x_k(3);
            x2 = x_k_1(1);
            y2 = x_k_1(2);
            theta2 = x_k_1(3);

            delta_x = x2 - x1; 
            delta_y = y2 - y1;

            c = cos(theta); 
            s = sin(theta); 
            dT = obj.dT; 

            obj.J{1} = (1/dT) * [-c, -s, -s*delta_x + c*delta_y; 
                                s, -c, -c*delta_x - s*delta_y; 
                                0, 0, -1]; 

            obj.J{2} = (1/dT) * [c, s, 0; 
                                -s, c, 0; 
                                0, 0, 1]; 

        end
    end    
end