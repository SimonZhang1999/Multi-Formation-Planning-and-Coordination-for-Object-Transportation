classdef GeoProperties < handle
    properties (SetAccess = protected)
        % translation & rotation states in world frame (6 dim)
        % angle expressed in Euler angle rotations "ZYX" ( R = Rz(thy)*Ry(thp)*Rx(thr))
        position
        % frames
        rotMat
        hTransMat
        % a list of vertices and edges in body frame
        vertices % (4x2)
        faces % (1x4)
        edges % (1x2) depth(x) width(y) 
        % vertices list in world frame
        verticesStates = struct % position 
    end
    
    properties (Constant, Access = private)
        templateVertices = [0,0;0,1;1,1;1,0;];
        templateFaces = [1,2,3,4];
    end
    
    methods
        function obj = GeoProperties(inputPosition,inputEdges)
            % basic configuration
            if isvector(inputPosition)&&....
                    length(inputPosition)==3&&...
                    isvector(inputEdges)&&...
                    length(inputEdges)==2
                obj.edges = inputEdges(:)';
                obj.position = inputPosition(:);
                % vertices list in body frame (format: [x y])     
                obj.edge2body;
                % vertices list in world frame
                obj.updateVerticesPosition;
            else
                % error('Improper input dimension.')
                obj.position = inputPosition(:);
                obj.edges = inputEdges;
                obj.edge2obs;
                obj.updateVerticesPosition_obs;
            end
        end
        
        function obj = updateStates(obj,inputPosition)
            obj.position = inputPosition(:);
            obj.updateVerticesPosition;
        end
    end
    
    methods (Access = protected)                     
        function obj = updateVerticesPosition(obj)       
            % frame update
            obj.rotMat = rotz(obj.position(3));
            % rotation matrix -> translation matrix
            obj.hTransMat = rt2tr(obj.rotMat,obj.position(1:3));
            obj.hTransMat(3,4) = 0;
            % vertices position
            obj.verticesStates.position = obj.rotMat(1:2,1:2)*obj.vertices' + obj.position(1:2);
        end
        
        function obj = updateVerticesPosition_obs(obj)
            obs_position = obj.position;
            for edge_num = 1:length(obj.vertices)
                vertices_pos(edge_num,:) = obj.vertices(edge_num,:) + obs_position';
            end
            obj.verticesStates.position = vertices_pos';
        end
        
        function obj = edge2body(obj)
           obj.faces = obj.templateFaces;
           vertice_up = obj.templateVertices(:,1)*obj.edges(1)-obj.edges(1)/2;
           vertice_dowm = obj.templateVertices(:,2)*obj.edges(2)-obj.edges(2)/2;
           obj.vertices = [vertice_up,vertice_dowm]; 
        end
        
        function obj = edge2obs(obj)
            obj.faces = [1 2 3 4];
            for i = 1:4
                rand_theta = rand();
                obj.vertices(i,1) = obj.edges*cos(2*pi*rand_theta);
                obj.vertices(i,2) = obj.edges*sin(2*pi*rand_theta);
            end
        end
    end
end