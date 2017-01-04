close all; clear all; clc;
no_aux_plots = true;
minErrorAfterSeen = 1;
minIterationsAfterSeen = 100;
min_robots = 2;
max_robots = 10;
num_runs = max_robots-min_robots + 1;
num_stats = 2; %OMNI1, OMNI2
robots_arr = min_robots:max_robots;
x_arr = (ones(num_stats,1) * robots_arr)';
colors = ['r', 'b'];
legends = {'OMNI1 mean', 'OMNI2 mean', 'OMNI1 fit', 'OMNI2 fit'};

target_means = zeros(num_runs, 1);
omni_means = zeros(num_runs, num_stats);
run = 0;

for r=robots_arr
    run = run + 1;
    file = sprintf('logs/%d_robots.txt',r);
    
    % run the full eval script 1 time
    fullEval;
    
    target_means(run) = targetStats.mean;
    for stat = 1:num_stats
        omni_means(run, stat) = robotStats{stat}.mean;
    end
end

figMeans = figure('units', 'normalized', 'position', [.2 .25 .6 .5]);
subplot(1,2,1);
for stat = 1:num_stats
    scatter(x_arr(:,stat), omni_means(:,stat), 'markeredgecolor', colors(stat));
    hold on;
end
set(gca, 'XTick', robots_arr);
hold off;
axis([min_robots-0.5, max_robots+0.5, 0, max(omni_means(:))+0.05]);
lines = lsline;
for stat = 1:num_stats
    set(lines(stat), 'color', colors(stat));
end
legend(legends);
title({'Self-localization performance'});
xlabel('Number of robots');
ylabel('Error');


subplot(1,2,2);
scatter(x_arr(:,stat), target_means, 'markeredgecolor', 'g');
axis([min_robots-0.5, max_robots+0.5, 0, max(target_means)+0.05]);
lsline;
title({'Target tracking performance'})
xlabel('Number of robots');
ylabel('Error');