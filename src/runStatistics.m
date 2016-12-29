function [ robotStatistics, targetStatistics ] = runStatistics( robotErrors, targetErrors )
%RUNSTATISTICS Provide mean, median and variance statistics on the output
%of evaluate_omni_dataset
%   Use readOutput() and provide the necessary inputs to this function
%   You should filter the outputs first for accuracte statistics
%   Output:
%   - robotStatistics: nRobots x 1 struct cells, each containing the mean,
%                      median and variance for that robot's errors
%   - targetStatistics: struct containing mean, median and variance for the
%   target errors

nRobots = size(robotErrors, 2);
robotStatistics = cell(nRobots, 1);
for r = 1:nRobots
    errs = robotErrors(:,r);
    s = struct('mean', mean(errs), 'median', median(errs), 'variance', var(errs));
    robotStatistics{r} = s;
end

targetStatistics = struct('mean', mean(targetErrors), 'median', median(targetErrors), 'variance', var(targetErrors));