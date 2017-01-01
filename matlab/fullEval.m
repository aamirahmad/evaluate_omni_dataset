% Modify these values according to preference
if ~exist('file', 'var')
    file= 'logs/2_robots.txt';
end
if ~exist('minIterationsAfterSeen', 'var')
    minIterationsAfterSeen = 100;
end

if ~exist('minErrorAfterSeen', 'var')
    minErrorAfterSeen= 1;
end

if exist('no_aux_plots', 'var')
    make_plots = ~no_aux_plots;
else
    make_plots = true;
end

% Read and parse output file
[robotErrors, targetErrors, targetSeen, nRobots, ~] = readOutput(file);

% Filter target errors
filterIdx = filterOutput(targetErrors, targetSeen, make_plots, minIterationsAfterSeen, minErrorAfterSeen);
filteredTargetErrors = targetErrors(filterIdx);

% Run statistics on filtered output
[robotStats, targetStats] = runStatistics(robotErrors, filteredTargetErrors);

% Display in a table
rowNames = cell(nRobots + 1, 1);    rowNames{1} = 'Target';
Mean = zeros(nRobots + 1, 1);      Mean(1) = [targetStats.mean];
Median = zeros(nRobots + 1, 1);    Median(1) = [targetStats.median];
Variance = zeros(nRobots + 1, 1);  Variance(1) = [targetStats.variance];
for r=1:nRobots
    rowNames{r+1} = strcat('OMNI',num2str(r));
    Mean(r+1) = robotStats{r}.mean;
    Median(r+1) = robotStats{r}.median;
    Variance(r+1) = robotStats{r}.variance;
end

T = table(Mean, Median, Variance, 'RowNames', rowNames);
disp(T);
