classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    % LandmarkRangeBearingEdge summary of LandmarkRangeBearingEdge
    %
    % This class stores an edge which represents the factor for observing
    % the range and bearing of a landmark from the vehicle. Note that the
    % sensor is fixed to the platform.
    %
    % The measurement model is
    %
    %    z_(k+1)=h[x_(k+1)]+w_(k+1)
    %
    % The measurements are r_(k+1) and beta_(k+1) and are given as follows.
    % The sensor is at (lx, ly).
    %
    %    dx = lx - x_(k+1); dy = ly - y_(k+1)
    %
    %    r(k+1) = sqrt(dx^2+dy^2)
    %    beta(k+1) = atan2(dy, dx) - theta_(k+1)
    %
    % The error term
    %    e(x,z) = z(k+1) - h[x(k+1)]
    %
    % However, remember that angle wrapping is required, so you will need
    % to handle this appropriately in compute error.
    %
    % Note this requires estimates from two vertices - x_(k+1) and l_(k+1).
    % Therefore, this inherits from a binary edge. We use the convention
    % that vertex slot 1 contains x_(k+1) and slot 2 contains l_(k+1).
    
    methods(Access = public)
    
        function obj = LandmarkRangeBearingEdge()
            % LandmarkRangeBearingEdge for LandmarkRangeBearingEdge
            %
            % Syntax:
            %   obj = LandmarkRangeBearingEdge();
            %
            % Description:
            %   Creates an instance of the LandmarkRangeBearingEdge object.
            %   Note we feed in to the constructor the landmark position.
            %   This is to show there is another way to implement this
            %   functionality from the range bearing edge from activity 3.
            %
            % Outputs:
            %   obj - (handle)
            %       An instance of a LandmarkRangeBearingEdge.

            obj = obj@g2o.core.BaseBinaryEdge(2);
        end
        
        function initialEstimate(obj)
            % INITIALESTIMATE Compute the initial estimate of the landmark.
            %
            % Syntax:
            %   obj.initialEstimate();
            %
            % Description:
            %   Compute the initial estimate of the landmark given the
            %   platform pose and observation.

            % warning('LandmarkRangeBearingEdge.initialEstimate: implement')
            

            pose = obj.edgeVertices{1}.x;   % [x; y; theta]
            z = obj.measurement();          % [range; bearing]

            r = z(1);
            beta = z(2);
            phi = beta + pose(3);

            lx = pose(1) + r * cos(phi);
            ly = pose(2) + r * sin(phi);

            obj.edgeVertices{2}.x = [lx; ly];
        
            % obj.edgeVertices{2}.setEstimate(l);
        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for the edge.
            %
            % Syntax:
            %   obj.computeError();
            %
            % Description:
            %   Compute the value of the error, which is the difference
            %   between the predicted and actual range-bearing measurement.

            % warning('LandmarkRangeBearingEdge.computeError: implement')
           
            % e(x,z) = z(k+1) - h[x(k+1)]

            % Predict a measurment z_hat 
            pose = obj.edgeVertices{1}.x;   % [x; y; theta
            landmark = obj.edgeVertices{2}.x; % [lx; ly]
            dx = landmark(1) - pose(1);
            dy = landmark(2) - pose(2);
            r_hat = sqrt(dx^2 + dy^2); % predicted range
            beta_hat = atan2(dy, dx) - pose(3); % predicted bearing

            % get the actual measurement z
            z = obj.measurement(); % [range; bearing]
            r_measured = z(1);
            beta_measured = z(2);
            e = [r_measured - r_hat; wrapToPi(beta_measured - beta_hat)];
            obj.errorZ = e;

        end
        
        function linearizeOplus(obj)
            % linearizeOplus Compute the Jacobian of the error in the edge.
            %
            % Syntax:
            %   obj.linearizeOplus();
            %
            % Description:
            %   Compute the Jacobian of the error function with respect to
            %   the vertex.
            %

            % warning('LandmarkRangeBearingEdge.linearizeOplus: implement')

            % get the platform pose and landmark position

            pose = obj.edgeVertices{1}.x;   % [x; y; theta]
            x = pose(1); y = pose(2); theta = pose(3);
            landmark = obj.edgeVertices{2}.x; % [lx; ly]
            lx = landmark(1); ly = landmark(2);
            dx = lx - x; dy = ly - y;
            q = dx^2 + dy^2;
            if q < 1e-12
                q = 1e-12;
            end
            r = sqrt(q);

            Je_x = [dx/r, dy/r, 0;
                    -dy/q, dx/q, 1];

            Je_m = [-dx/r, -dy/r;
                    dy/q, -dx/q];

            obj.J{1} = Je_x;
            obj.J{2} = Je_m;
        end        
    end
end
