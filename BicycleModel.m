classdef BicycleModel
    %  Bicycle Model
    %   Calculates forward and inverse kinematics

    properties
        % Wheel radius in meters [m]
        WheelRadius  = 1.0;
        
        % Distance from wheel to wheel in meters [m]
        WheelBase  = 1.0;
        
    end
    
    methods
        function obj = BicycleModel(wheelRadius, wheelBase)
            %   BicycleModel Class Constructor
            %   Inputs: 
            %   wheelRadius: wheel radius [m]
            %   WheelBase: distance from wheel to wheel [m]
            
            % Assign parameters
            obj.WheelRadius = wheelRadius;
            obj.WheelBase = wheelBase;
        end
        
        function [r_xi] = calcForwardKinematics(obj, wr, delta) 
            %CALCFORWARDKINEMATICS Calculates forward kinematics
            % Inputs:
            %    wz: rear wheel speed [rad/s]
            %    delta: angle  [rad/s]
            % Outputs:
            %     r_xi : [vx; 0 ; wz] robot velocity vector in robot base frame                        
            %     vx: robot speed in robot frame [m/s]
            %     wz: robot angular vel in robot frame [rad/s]            
            
            vx = obj.WheelRadius * wr;
            wz = obj.WheelRadius * wr * tan(delta) / obj.WheelBase;
            
            r_xi = [vx;0;wz];
        end
        
        function [wr, delta] = calcInverseKinematics(obj, r_xi)
            %CALCINVERSEKINEMATICS Calculates forward kinematics
            % Inputs:
            %    r_xi : [vx; 0 ; wz] robot velocity vector in robot base frame             
            % Outputs:
            %       wr: rear wheel speed [rad/s]
            %       delta: angle [rad/s]
            
            vx = r_xi(1);
            wz = r_xi(3);
            delta = atan2(wz * obj.WheelBase,vx);
            
            wr = vx / obj.WheelRadius;            
        end 
    end
end