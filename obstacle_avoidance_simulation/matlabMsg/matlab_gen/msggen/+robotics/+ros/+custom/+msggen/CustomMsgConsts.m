classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    properties (Constant)
        turtlebot3_test_obst_avoid_ControlSimulation = 'turtlebot3_test_obst_avoid/ControlSimulation'
        turtlebot3_test_obst_avoid_EvolutiveSystemConfiguration = 'turtlebot3_test_obst_avoid/EvolutiveSystemConfiguration'
        turtlebot3_test_obst_avoid_RobotInfo = 'turtlebot3_test_obst_avoid/RobotInfo'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(3, 1);
                msgList{1} = 'turtlebot3_test_obst_avoid/ControlSimulation';
                msgList{2} = 'turtlebot3_test_obst_avoid/EvolutiveSystemConfiguration';
                msgList{3} = 'turtlebot3_test_obst_avoid/RobotInfo';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
