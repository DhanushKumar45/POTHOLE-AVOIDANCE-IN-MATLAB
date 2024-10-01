%% Highway Simulator with Improved Pothole and Car Detection and Avoidance
% This script displays a 3-D animation of a highway where cars detect
% potholes and other vehicles and either change lanes or slow down to avoid them.

if ~license('test','automated_driving_toolbox')
    error('This example requires Automated Driving Toolbox.');
end

rngPrev = rng(0);

if ~isempty(findobj('Name','Highway Scenario'))
    close 'Highway Scenario';
end

numCars = 3;  % Number of cars
numLanes = 2;  % Number of lanes
laneWidth = 3;  % Width of each lane
roadLength = 2000;  % Increased length of the road

% Create a highway scenario with a longer duration
scenario = HighwayScenario(...
    'NumCars', numCars,...
    'NumLanes', numLanes,...
    'LaneWidth', laneWidth,...
    'Dt', 1e-1,...  % Smaller time step for more granular movement
    'AnimateChart', true,...
    'Plot', true,...
    'AverageDesiredSpeed', 4,...  % Slower average speed
    'Sigma', 2,...
    'EgoDesiredSpeed', 6,...  % Slower desired speed for ego vehicle
    'NumUserControlledCars', 0);

% Define potholes
numPotholes = 3;  % Number of potholes
potholePositions = rand(1, numPotholes) * roadLength;  % Random positions for potholes
potholeLanes = randi([1,2], 1, numPotholes);  % Random lane for each pothole (1 for left, 2 for right)

% Display pothole positions and lanes
disp('Pothole positions and lanes:');
for i = 1:numPotholes
    disp(['Pothole ', num2str(i), ': Position = ', num2str(potholePositions(i)), ', Lane = ', num2str(potholeLanes(i))]);
end

% Simulation loop
for step = 1:1000  % Increased number of steps
    scenario.step();
    
    hold on;
    % Plot potholes
    for potholeIdx = 1:numPotholes
        if potholeLanes(potholeIdx) == 1
            % Pothole in the left lane
            plot3(potholePositions(potholeIdx), laneWidth/2, 0, 'ro', 'MarkerSize', 15, 'DisplayName', 'Pothole');
        else
            % Pothole in the right lane
            plot3(potholePositions(potholeIdx), -laneWidth/2, 0, 'ro', 'MarkerSize', 15, 'DisplayName', 'Pothole');
        end
    end
    hold off;
    
    % For each car in the scenario
    for carIdx = 1:numCars
        carPosition = scenario.Drivers{carIdx}.myPos;  % Get current position of the car
        carX = carPosition(1);  % Longitudinal position (X-coordinate)
        currentLane = scenario.Drivers{carIdx}.myLane;  % Current lane of the car
        
        %% Pothole Detection and Avoidance
        for potholeIdx = 1:numPotholes
            % Check if the pothole is within 10 meters ahead and in the same lane
            if (carX < potholePositions(potholeIdx)) && ...
               (potholePositions(potholeIdx) - carX) < 10 && ...
               currentLane == potholeLanes(potholeIdx)
           
                disp(['Car ', num2str(carIdx), ' detected a pothole at position ', num2str(potholePositions(potholeIdx))]);

                % Try to change lanes to avoid the pothole
                if currentLane < numLanes  % If not in the rightmost lane, move to the right
                    scenario.Drivers{carIdx}.myLane = currentLane + 1;
                    disp(['Car ', num2str(carIdx), ' changed to right lane to avoid pothole.']);
                elseif currentLane > 1  % If not in the leftmost lane, move to the left
                    scenario.Drivers{carIdx}.myLane = currentLane - 1;
                    disp(['Car ', num2str(carIdx), ' changed to left lane to avoid pothole.']);
                else
                    % If no lane change is possible, slow down significantly
                    scenario.Drivers{carIdx}.myVel(1) = 2;  % Slow down
                    disp(['Car ', num2str(carIdx), ' slowed down for the pothole.']);
                end
            end
        end
        
        %% Car Detection and Avoidance
        for otherCarIdx = 1:numCars
            if carIdx ~= otherCarIdx  % Avoid self-check
                otherCarPosition = scenario.Drivers{otherCarIdx}.myPos;
                otherCarX = otherCarPosition(1);
                otherCarLane = scenario.Drivers{otherCarIdx}.myLane;
                
                % Detect if another car is within 10 meters ahead in the same lane
                if abs(carX - otherCarX) < 10 && carX < otherCarX && currentLane == otherCarLane
                    disp(['Car ', num2str(carIdx), ' detected car ', num2str(otherCarIdx), ' in front. Slowing down.']);
                    
                    % Slow down
                    scenario.Drivers{carIdx}.myVel(1) = 3;
                    
                    % Attempt to change lanes
                    if currentLane < numLanes  % Move to the right lane
                        scenario.Drivers{carIdx}.myLane = currentLane + 1;
                    elseif currentLane > 1  % Move to the left lane
                        scenario.Drivers{carIdx}.myLane = currentLane - 1;
                    end
                end
            end
        end
        
        % Random lane change attempts
        if rand > 0.7 % 30% chance of lane change attempt
            currentLane = scenario.Drivers{carIdx}.myLane;
            if currentLane < numLanes
                scenario.Drivers{carIdx}.myLane = currentLane + 1; % Lane change to right
            elseif currentLane > 1
                scenario.Drivers{carIdx}.myLane = currentLane - 1; % Lane change to left
            end
        end
    end
    
    pause(0.01);  % Pause for visualization
end

% Clean up after simulation
for i = 1:numCars
    delete(scenario.Drivers{i});
end
delete(scenario);

rng(rngPrev);
