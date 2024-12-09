% Notes:
% Delete Mode 1: Full Sim Mode
%   This is done because each time we input parameters, it will always just
%   be an array. Therefore, if the array is just one value, it will only do
%   that one mode. If an array of more than one value, then it will
%   automatically run them in that order, as opposed to just one.

% Implement a lamp on the switch case. This will show in the GUI, as
% opposed to the switches so each

% Make a config file and a User's Manual as an example for how user's can
% input their parameters and how to interact with the GUI. The GUI should
% now just be uploading their config file, the lamps near each mode, and
% the current values being scoped. We will have these graphs as before to
% show how it changes in time.
%   It will be similar to Constants_gui., but also include the
%   configuration and function that sorts between each mode and runs it in
%   Simulink. Add the delay at the end of each function so once one mode
%   finishes, it does not immediately switch to the next.

% The config file is READ INTO Matlab (constants_gui.m) which loads them as
% variables. Then Constants_gui starts up simulink (Simulator.slx) and the
% user can go into the GUI.

% The actual function is not defined in this Matlab file. It is defined
% inside Simulink, as the User Defined Matlab function.

% Conclusions.
% USE COMMAND LINE. NO CONFIG FILE NEEDED. THIS IS SIMILAR TO THE
% IMPLEMENTATION BELOW.

%% SDA Small Scale ACS Mode Selector
% Selecting Multiple Modes for Simulator
% Separate by a decasde in frequency (10x slower/faster) when comparing
% different modes such as ACS vs. thrusters
% Separate the ACS vs. thrusters in frequency (time constants things)
% r cross F = -kH h is momentum of wheels for i = 6 for each of the 6
% thrusters
% Create Jacobian, cannot have the negative force from the thrusters
% instead of a delay
% Create chart of when transitions of modes are legal and not. Create
% function.
% COmmanded of the last mode tracks into the intial
% list of tests for fault protection (sturated wheels)
% Dependning of input of tests, if fails test automaticaly switch to fault
% test mode/startup mode, etc.
% Check across simple tests for fault test architecture (every single time
% step)
    % Wheels speeds, battery charges (turn off if reached), leaks in propulsion?, pauses the test so operator can  etc.
    % Run mass balances so the spacecraft is like a simple pendulum?
    % Question: Is there a test that tells us if theres a problem with the
    % mode
    % Different set of values for tests in each mode for what is allowable
    % for each mode. Set up consequences for each, where most of them go to
    % safe mode. Others may include turning off battery, mass balance
    % pendulum.
    % Components of ACS fault protection: Test values (-x and +x), test themselves (greater than, less than, equal to), 
    % outcome of test failing, scaling (is fault protection even active?)
        % Check if holdiong attitude then put disturbance in one
        % direction. COuld cause fault protection to trip, therefore turn off scanning (are you looking at tests or not? IF turned off scaling, then all tests will pass easily.)
        % Scanning is pretty much fault protection on or off. Can actually
        % enable the simulink test block for scanning, or unable it so that
        % it skips over the entire scanning.

% Complete safe mode and individual safe modes in between different modes
% (not everything turned off but no new commands can go to them)
% CAN CREATE MATLAB FUNCTION AND IMPLEMENT THIS INTO THE GUI, BUT WILL BE
% SLOWER

% PDR:
% Review all requirements, and how we implement them
% Find drivers, etc. for next semester once PDR is finished and
% implementing on large scale when next semester comes. 
% Can read all sensors and command all actuators
% How close loop of errors in all the modes
% How to jump between modes
% User interface
% Walkthrough of how to use the testbed
% Fully implement into next semester
% Any risks from the work we have done so far
% List of tasks that are yet to be done in terms of schedule and cost
% Explicitly command what is different between small and large scale testbeds
%{
clear all; close all;

fprintf('Available Modes:\n'); fprintf(['1. Full Sim Test Mode\n2. Start Up Mode\n3.Constant Rate Mode\n4. Slew Mode\n' ...
    '5. Attitude Control Mode\n6. Momentum Dump Mode\n']); % List available programs
WhatProgramRuns = input('\nWhat program(s) would you like (input as #, Ex: 3, 4, 6): ','s');

% for i in range(indicatedModes(-1)):
% Goes by individual i at a time
% Something like if indicatedModes contains ''

indicatedModes = str2num(WhatProgramRuns); % Convert string to numeric array

if isempty(indicatedModes)
    error('No modes were selected.');
end

%% WILL NOT WORK YET SCOPING SINCE NOT CONNECTED TO SIMULATOR, ADJUST LATER
currentQuat = [0, 0, 0, 1]; currentVel = [0, 0, 1];
%currentQuat = get_param('Simulator1/scopeQuat', 'RuntimeObject').Value; % not sure if this will be a giant array because selected array value in To Workspace block.
%currentVel = get_param('Simulator1/scopeAngVel', 'RuntimeObject').Value;

for i = 1:length(indicatedModes)
    mode = indicatedModes(i);    
    switch mode
        case 1 
            disp('Full Sim Test Mode');
            commandVel = input('Enter commanded angular velocities [wx, wy, wz] (rad/s): ');
            commandQuat = input('Enter commanded quaternion [q1, q2, q3, q4]: ');
            % Example: Pass to Simulink
            % set_param('your_model_path', 'cmdVelocity', mat2str(cmdVelocity));
            % set_param('your_model_path', 'cmdQuaternion', mat2str(cmdQuaternion));
            disp(['Commanded Velocity: ', mat2str(commandVel)]);
            disp(['Commanded Quaternion: ', mat2str(commandQuat)]);
        
        case 2 
            disp('Start Up Mode');
            startupDuration = input('Enter startup duration (s): ');
            % Example: Pass to Simulink
            % set_param('your_model_path', 'startupDuration', num2str(startupDuration));
            disp(['Startup Duration: ', num2str(startupDuration)]);
        
        case 3 % 
            disp('Constant Rate Mode');
            cmdAngularVel = input('Enter commanded angular velocities [wx, wy, wz] (rad/s): ');
            % Example: Pass to Simulink
            % set_param('your_model_path', 'cmdAngularVel', mat2str(cmdAngularVel));
            disp(['Commanded Angular Velocities: ', mat2str(cmdAngularVel)]);
        
        case 4
            disp('Slew Mode');
            commandQuat = input('Enter commanded quaternion [q1, q2, q3, q4]: ');
            % Compute quaternion difference for Slew
            % Example: Simulink or MATLAB quaternion difference computation
            SlewQuaternion = quatMultiply(quatConjugate(currentQuat), commandQuat);
            currentQuat = commandQuat; % Update the current quaternion
            disp(['Slew Quaternion: ', mat2str(SlewQuaternion)]);
            disp(['Updated Current Quaternion: ', mat2str(currentQuat)]);
        
        case 5 
            disp('Attitude Control Mode');
            commandQuat = input('Enter target quaternion [q1, q2, q3, q4]: ');
            % Update current quaternion
            currentQuat = commandQuat;
            % Example: Pass to Simulink
            % set_param('your_model_path', 'cmdQuaternion', mat2str(cmdQuaternion));
            disp(['Target Quaternion: ', mat2str(commandQuat)]);
        
        case 6 
            disp('Momentum Dump Mode');
            commandQuat = input('Enter momentum dump quaternion [q1, q2, q3, q4]: ');
            % Example: Pass to Simulink
            % set_param('your_model_path', 'cmdQuaternion', mat2str(cmdQuaternion));
            disp(['Momentum Dump Quaternion: ', mat2str(commandQuat)]);
        
        otherwise
            disp(['Mode ', num2str(mode), ' is not recognized.']);
    end
    
    % Optionally update current state (quaternion, velocity, etc.) from Simulink
    % Uncomment and adjust the following as needed:
    % currentQuaternion = get_param('your_model_path/To_Workspace_Block', 'RuntimeObject').Value;
end

disp('Complete!');
%}
function ACSModeSequence()
    fprintf('Available Modes:\n'); fprintf('0. Start Up Mode\n1. Constant Rate Mode\n2. Slew Mode\n3. Attitude Control Mode\n4. Momentum Dump Mode\n');
    modes = input('Enter modes to run in an array (e.g., [2, 3, 5], ensure to put bracket): ');
    if isempty(modes)
        error('No modes selected.');
    end

    % POTENTIALLY DELETE THIS PORTION FOR NOW.
    % Optional initial conditions
    useDefaultIC = input('Use default initial conditions? (Y or N): ', 's');
    if strcmpi(useDefaultIC, 'N')
        initQuat = input('Enter initial quaternion [q1, q2, q3, q0]: ');
        %initVel = input('Enter initial angular velocity [wx, wy, wz]: ');
    else
        initQuat = [0, 0, 0, 1];
        %initVel = [0, 0, 0];
    end
    
%% CHANGE THIS IT CURRENTLY GOES TO COMMANDED INSTEAD OF INITIAL BLOCK

    % Quaternion Initial Conditions
    set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion1/Constant2', 'Value', mat2str(initQuat(1))); 
    set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion1/Constant', 'Value', mat2str(initQuat(2))); 
    set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion1/Constant1', 'Value', mat2str(initQuat(3))); 
    set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion1/Constant3', 'Value', mat2str(initQuat(4))); 

    % Angular Velocity Initial Conditions
    % Assume 0

    set_param('SimulatorSpencer', 'SimulationCommand', 'start'); % Start Simulink model

    modeTimes = zeros(length(modes), 1); % Track execution time for each mode
    for i = 1:length(modes)
        CheckBox = modes(i);
        % Error with this! Look at individual mode example
        assignin('base', 'CheckBox', CheckBox); % Update base workspace

        modeStartTime = tic; % Start timer for the current mode

        switch CheckBox
            case 0 % Startup Mode
                commandedAngVel = input('Enter commanded angular velocities [wx, wy, wz] (rad/s): ');
                commandedQuat = input('Enter commanded quaternion [q1, q2, q3, q0]: ');
                
                % Commanded Angular Velocities
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Velocity/Constant2', 'Value', mat2str(commandedAngVel(1)));
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Velocity/Constant', 'Value', mat2str(commandedAngVel(2)));
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Velocity/Constant1', 'Value', mat2str(commandedAngVel(3)));
                
                % Commanded Quaternion Conditions
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant2', 'Value', mat2str(commandedQuat(1))); 
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant', 'Value', mat2str(commandedQuat(2))); 
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant1', 'Value', mat2str(commandedQuat(3))); 
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant3', 'Value', mat2str(commandedQuat(4))); 

            case 1 % Constant Rate Mode
                commandedAngVel = input('Enter commanded angular velocities [wx, wy, wz] (rad/s): ');
                % commandedQuat = input('Enter commanded quaternion [q1, q2, q3, q0]: ');
                % Commanded Angular Velocities
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Velocity/Constant2', 'Value', mat2str(commandedAngVel(1)));
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Velocity/Constant', 'Value', mat2str(commandedAngVel(2)));
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Velocity/Constant1', 'Value', mat2str(commandedAngVel(3)));

                % No Commanded Quaternion for this mode

            case 2 % Slew Mode
                % commandedAngVel = input('Enter commanded angular velocities [wx, wy, wz] (rad/s): ');
                commandedQuat = input('Enter commanded quaternion [q1, q2, q3, q0]: ');
                %set_param('SimulatorSpencer', 'CmdQuaternion', mat2str(commandedQuat));

                % No Commanded Angular Velocity for this mode         

                % Commanded Quaternion Conditions
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant2', 'Value', mat2str(commandedQuat(1))); 
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant', 'Value', mat2str(commandedQuat(2))); 
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant1', 'Value', mat2str(commandedQuat(3))); 
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant3', 'Value', mat2str(commandedQuat(4))); 
                
            case 3 % Attitude Control Mode
                % commandedAngVel = input('Enter commanded angular velocities [wx, wy, wz] (rad/s): ');
                commandedQuat = input('Enter target quaternion [q1, q2, q3, q0]: ');
                %set_param('SimulatorSpencer', 'CmdQuaternion', mat2str(commandedQuat));

                % No Commanded Angular Velocity for this mode         

                % Commanded Quaternion Conditions
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant2', 'Value', mat2str(commandedQuat(1))); 
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant', 'Value', mat2str(commandedQuat(2))); 
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant1', 'Value', mat2str(commandedQuat(3))); 
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant3', 'Value', mat2str(commandedQuat(4)));               
                
            case 4 % Momentum Dump Mode
                % commandedAngVel = input('Enter commanded angular velocities [wx, wy, wz] (rad/s): ');
                commandedQuat = input('Enter momentum dump quaternion [q1, q2, q3, q0]: ');
                %set_param('SimulatorSpencer', 'CmdQuaternion', mat2str(commandedQuat));
                
                % No Commanded Angular Velocity for this mode         

                % Commanded Quaternion Conditions
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant2', 'Value', mat2str(commandedQuat(1))); 
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant', 'Value', mat2str(commandedQuat(2))); 
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant1', 'Value', mat2str(commandedQuat(3))); 
                set_param('SimulatorSpencer/Mode Selection/Commanded values/Commanded Quaternion/Constant3', 'Value', mat2str(commandedQuat(4))); 

            otherwise
                disp(['Mode ', num2str(CheckBox), ' is not recognized.']);
                continue;
        end
        
        % Wait until conditions are met
        disp(['Executing Mode ', num2str(CheckBox), '...']);
        %{
        while true
            %currentQuat = evalin('base', 'out.scopeQuat'); currentAngVel = evalin('base', 'out.scopeAngVel');
            
            currentQuat = evalin('SimulatorSpencer/Extras', 'out.quat1');
            currentAngVel = evalin('SimulatorSpencer/Extras', 'out.angVelX');
            %quat: Display2, Display3, Diplsya4, Display5
            %angVel: Display6, Display7, Display8
            %quatError = norm(currentQuat - commandedQuat); velError = norm(currentAngVel - commandedAngVel);
            
            quatError = 0; angVelError = 0;
            if ~isempty(commandedQuat)
                quatError = norm(currentQuat - commandedQuat);
            end
            if ~isempty(commandedAngVel)
                angVelError = norm(currentAngVel - commandedAngVel);
            end

            if quatError < 1e-3 && angVelError < 1e-3
                disp(['Mode ', num2str(CheckBox), ' reached commanded state.']);
                break;
            end     

            pause(0.1); 
        end
        %}
        modeTimes(i) = toc(modeStartTime); % Time to complete mode
        
        % Reset switches (simulate turning off all modes)
        %set_param('SimulatorSpencer', 'CmdAngularVel', '[0, 0, 0]');
        %set_param('SimulatorSpencer', 'CmdQuaternion', '[0, 0, 0, 1]');
        disp(['Mode ', num2str(CheckBox), ' completed. Time taken: ', num2str(modeTimes(i)), ' seconds.']);
        pause(2); % Short delay before starting the next mode
    end 

    set_param('SimulatorSpencer', 'SimulationCommand', 'stop');     % Stop the Simulink simulation

    % Display summary of mode times
    disp('Summary of mode execution times:');
    for i = 1:length(modes)
        disp(['Mode ', num2str(modes(i)), ': ', num2str(modeTimes(i)), ' seconds']);
    end
    disp('All modes completed.');
end