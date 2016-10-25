% Record the performance of PF-UCLT on the OMNI-dataset
rosshutdown
rosinit

% Retrieve some things from the parameter server
% Initialize the parameter tree
ptree = rosparam;
% Get values of parameters or error
if(~has(ptree, 'MAX_ROBOTS'))
    error('Parameter MAX_ROBOTS not found');
else
    maxRobots = get(ptree, 'MAX_ROBOTS');
end

%Currently not working - MATLAB can't read arrays from parameter server?
% if(~has(ptree, 'PLAYING_ROBOTS'))
%     error('Parameter PLAYING_ROBOTS not found');
% else
%     playingRobots = get(ptree, 'PLAYING_ROBOTS');
% end
playingRobots = [1 0 1 1 1];

% Construct ROS Subscriber objects
effectiveRobots = sum(playingRobots);
subscribers = cell(effectiveRobots);
robotNames = cell(effectiveRobots);
k = 1;
legendArr = cell(1, effectiveRobots);
for i=1:length(playingRobots)
    if playingRobots(i)
        robotNames{k} = sprintf('OMNI%d', i);
        subscribers{k} = rossubscriber(sprintf('omni%d/EstimationError', i));
        legendArr{k} = sprintf('%s Euclidean Error',robotNames{k}, i);
        k = k+1;
    end
end

targetSubscriber = rossubscriber('/OrangeBallEstimationError');

disp('Press key to begin, then press any key while focusing the figure to end');
pause;

figure;
set(gcf,'currentchar',' ');
oneArr = zeros(effectiveRobots, 1);
arrRobots = [];
arrTarget = [];
flagFail = false;
while get(gcf,'currentchar')==' '
    % Iterate over subscribers
    flagFail = false;
    for i=1:effectiveRobots
        try
            % 1 second timeout
            msg = receive(subscribers{i}, 1);
            
            % Put data in array
            oneArr(i) = msg.Data;
            
        catch EXCEPTION
            fprintf('No message received for subscriber %d. Trying again\n', i);
            flagFail = true;
            break
        end
    end
    
    if ~flagFail
        % Append the array
        arrRobots = [arrRobots oneArr]; %#ok<AGROW>
    else
        continue
    end
    
    % Now the target estimation error
    try
        msg = receive(targetSubscriber, 1);
        arrTarget = [arrTarget msg.Data]; %#ok<AGROW>
    catch EXCEPTION
        disp('No message received for target and timed-out. Trying again')
    end
end

close all;

figure;
subplot(2,1,1);
plot(arrRobots');
legend(legendArr);
hold on;
subplot(2,1,2);
plot(arrTarget');
legend('Target Euclidean Error');

means = zeros(1,effectiveRobots);
medians = zeros(1,effectiveRobots);
variances = zeros(1,effectiveRobots);

for i=1:effectiveRobots
    means(i) = mean(arrRobots(i,:));
    medians(i) = median(arrRobots(i,:));
    variances(i) = var(arrRobots(i,:));
    
    fprintf('%s mean=%f ; median=%f ; variance = %f\n', robotNames{i}, means(i), medians(i), variances(i));
end

meanTarget = mean(arrTarget);
medianTarget = median(arrTarget);
varianceTarget = var(arrTarget);

fprintf('Target mean=%f ; median=%f ; variance = %f\n', meanTarget, medianTarget, varianceTarget);