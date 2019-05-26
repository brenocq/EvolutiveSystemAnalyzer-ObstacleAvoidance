classdef EvolutiveSystemConfiguration < robotics.ros.Message
    %EvolutiveSystemConfiguration MATLAB implementation of obstacle_avoidance_simulation/EvolutiveSystemConfiguration
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'obstacle_avoidance_simulation/EvolutiveSystemConfiguration' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'e5487a10374b1974eaaba43f8d80fdec' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        SensorActivation
        LinearVelocity
        AngularVelocity
        RotationTime
        SensorAngle
        BackMutationPrevention
        Predation
        Mutation
        Neutralization
        Crossing
    end
    
    properties (Constant, Hidden)
        PropertyList = {'AngularVelocity', 'BackMutationPrevention', 'Crossing', 'LinearVelocity', 'Mutation', 'Neutralization', 'Predation', 'RotationTime', 'SensorActivation', 'SensorAngle'} % List of non-constant message properties
        ROSPropertyList = {'AngularVelocity', 'BackMutationPrevention', 'Crossing', 'LinearVelocity', 'Mutation', 'Neutralization', 'Predation', 'RotationTime', 'SensorActivation', 'SensorAngle'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = EvolutiveSystemConfiguration(msg)
            %EvolutiveSystemConfiguration Construct the message object EvolutiveSystemConfiguration
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function sensoractivation = get.SensorActivation(obj)
            %get.SensorActivation Get the value for property SensorActivation
            sensoractivation = single(obj.JavaMessage.getSensorActivation);
        end
        
        function set.SensorActivation(obj, sensoractivation)
            %set.SensorActivation Set the value for property SensorActivation
            validateattributes(sensoractivation, {'numeric'}, {'nonempty', 'scalar'}, 'EvolutiveSystemConfiguration', 'SensorActivation');
            
            obj.JavaMessage.setSensorActivation(sensoractivation);
        end
        
        function linearvelocity = get.LinearVelocity(obj)
            %get.LinearVelocity Get the value for property LinearVelocity
            linearvelocity = single(obj.JavaMessage.getLinearVelocity);
        end
        
        function set.LinearVelocity(obj, linearvelocity)
            %set.LinearVelocity Set the value for property LinearVelocity
            validateattributes(linearvelocity, {'numeric'}, {'nonempty', 'scalar'}, 'EvolutiveSystemConfiguration', 'LinearVelocity');
            
            obj.JavaMessage.setLinearVelocity(linearvelocity);
        end
        
        function angularvelocity = get.AngularVelocity(obj)
            %get.AngularVelocity Get the value for property AngularVelocity
            angularvelocity = single(obj.JavaMessage.getAngularVelocity);
        end
        
        function set.AngularVelocity(obj, angularvelocity)
            %set.AngularVelocity Set the value for property AngularVelocity
            validateattributes(angularvelocity, {'numeric'}, {'nonempty', 'scalar'}, 'EvolutiveSystemConfiguration', 'AngularVelocity');
            
            obj.JavaMessage.setAngularVelocity(angularvelocity);
        end
        
        function rotationtime = get.RotationTime(obj)
            %get.RotationTime Get the value for property RotationTime
            rotationtime = single(obj.JavaMessage.getRotationTime);
        end
        
        function set.RotationTime(obj, rotationtime)
            %set.RotationTime Set the value for property RotationTime
            validateattributes(rotationtime, {'numeric'}, {'nonempty', 'scalar'}, 'EvolutiveSystemConfiguration', 'RotationTime');
            
            obj.JavaMessage.setRotationTime(rotationtime);
        end
        
        function sensorangle = get.SensorAngle(obj)
            %get.SensorAngle Get the value for property SensorAngle
            sensorangle = single(obj.JavaMessage.getSensorAngle);
        end
        
        function set.SensorAngle(obj, sensorangle)
            %set.SensorAngle Set the value for property SensorAngle
            validateattributes(sensorangle, {'numeric'}, {'nonempty', 'scalar'}, 'EvolutiveSystemConfiguration', 'SensorAngle');
            
            obj.JavaMessage.setSensorAngle(sensorangle);
        end
        
        function backmutationprevention = get.BackMutationPrevention(obj)
            %get.BackMutationPrevention Get the value for property BackMutationPrevention
            backmutationprevention = logical(obj.JavaMessage.getBackMutationPrevention);
        end
        
        function set.BackMutationPrevention(obj, backmutationprevention)
            %set.BackMutationPrevention Set the value for property BackMutationPrevention
            validateattributes(backmutationprevention, {'logical', 'numeric'}, {'nonempty', 'scalar'}, 'EvolutiveSystemConfiguration', 'BackMutationPrevention');
            
            obj.JavaMessage.setBackMutationPrevention(backmutationprevention);
        end
        
        function predation = get.Predation(obj)
            %get.Predation Get the value for property Predation
            predation = int32(obj.JavaMessage.getPredation);
        end
        
        function set.Predation(obj, predation)
            %set.Predation Set the value for property Predation
            validateattributes(predation, {'numeric'}, {'nonempty', 'scalar'}, 'EvolutiveSystemConfiguration', 'Predation');
            
            obj.JavaMessage.setPredation(predation);
        end
        
        function mutation = get.Mutation(obj)
            %get.Mutation Get the value for property Mutation
            mutation = single(obj.JavaMessage.getMutation);
        end
        
        function set.Mutation(obj, mutation)
            %set.Mutation Set the value for property Mutation
            validateattributes(mutation, {'numeric'}, {'nonempty', 'scalar'}, 'EvolutiveSystemConfiguration', 'Mutation');
            
            obj.JavaMessage.setMutation(mutation);
        end
        
        function neutralization = get.Neutralization(obj)
            %get.Neutralization Get the value for property Neutralization
            neutralization = single(obj.JavaMessage.getNeutralization);
        end
        
        function set.Neutralization(obj, neutralization)
            %set.Neutralization Set the value for property Neutralization
            validateattributes(neutralization, {'numeric'}, {'nonempty', 'scalar'}, 'EvolutiveSystemConfiguration', 'Neutralization');
            
            obj.JavaMessage.setNeutralization(neutralization);
        end
        
        function crossing = get.Crossing(obj)
            %get.Crossing Get the value for property Crossing
            crossing = logical(obj.JavaMessage.getCrossing);
        end
        
        function set.Crossing(obj, crossing)
            %set.Crossing Set the value for property Crossing
            validateattributes(crossing, {'logical', 'numeric'}, {'nonempty', 'scalar'}, 'EvolutiveSystemConfiguration', 'Crossing');
            
            obj.JavaMessage.setCrossing(crossing);
        end
    end
    
    methods (Access = protected)
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.SensorActivation = obj.SensorActivation;
            cpObj.LinearVelocity = obj.LinearVelocity;
            cpObj.AngularVelocity = obj.AngularVelocity;
            cpObj.RotationTime = obj.RotationTime;
            cpObj.SensorAngle = obj.SensorAngle;
            cpObj.BackMutationPrevention = obj.BackMutationPrevention;
            cpObj.Predation = obj.Predation;
            cpObj.Mutation = obj.Mutation;
            cpObj.Neutralization = obj.Neutralization;
            cpObj.Crossing = obj.Crossing;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.SensorActivation = strObj.SensorActivation;
            obj.LinearVelocity = strObj.LinearVelocity;
            obj.AngularVelocity = strObj.AngularVelocity;
            obj.RotationTime = strObj.RotationTime;
            obj.SensorAngle = strObj.SensorAngle;
            obj.BackMutationPrevention = strObj.BackMutationPrevention;
            obj.Predation = strObj.Predation;
            obj.Mutation = strObj.Mutation;
            obj.Neutralization = strObj.Neutralization;
            obj.Crossing = strObj.Crossing;
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.SensorActivation = obj.SensorActivation;
            strObj.LinearVelocity = obj.LinearVelocity;
            strObj.AngularVelocity = obj.AngularVelocity;
            strObj.RotationTime = obj.RotationTime;
            strObj.SensorAngle = obj.SensorAngle;
            strObj.BackMutationPrevention = obj.BackMutationPrevention;
            strObj.Predation = obj.Predation;
            strObj.Mutation = obj.Mutation;
            strObj.Neutralization = obj.Neutralization;
            strObj.Crossing = obj.Crossing;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.obstacle_avoidance_simulation.EvolutiveSystemConfiguration.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.obstacle_avoidance_simulation.EvolutiveSystemConfiguration;
            obj.reload(strObj);
        end
    end
end
