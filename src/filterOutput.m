function [ filterIndices ] = filterOutput( targetErrors, targetSeen, minIterationsAfterSeen, minErrorAfterSeen )
%FILTEROUTPUT Filters the input errors
%   Filters the target errors so that we only count them when the target is
%   seen for some time. In this case, time is measured as iteration count.
%   It is also possible to filter according to the value of the estimation
%   error.
%   filterIndices = FILTEROUTPUT(targetErrors, targetSeen,
%   minIterationsAfterSeen, minErrorAfterSeen) will filter targetErrors so
%   that all iterations where target isn't seen are removed in the output
%   array. The return is a logical array where 1's are the indices that
%   should be counted after filtering
%   Additionally, when the target is seen again, it will take
%   minIterationsAfterSeen until the error is counted, and only after it
%   goes below the minErrorAfterSeen threshold. minIterationsAfterSeen and
%   minErrorAfterSeen are optional

if nargin < 4
    minErrorAfterSeen = 1e10;
end

if nargin < 3
    minIterationsAfterSeen = 0;
end

N = numel(targetErrors);
filterIndices = zeros(size(targetErrors));
iterSeen = 0;
flagErrorCrossed = false;
for iter=1 : N
    % detect target seen
    if targetSeen(iter) == 1
        iterSeen = iterSeen + 1;
        
        % make sure some iterations have passed since it's seen
        % if not, keep as 0
        if iterSeen < minIterationsAfterSeen
            continue
        end
        
        % need to check minimum error?
        if ~flagErrorCrossed
            % check error at least some value
            if targetErrors(iter) > minErrorAfterSeen
                continue % not yet crossed
            end
            
            % error has been crossed - set flag and mark indice OK
            flagErrorCrossed = true;
        end
        
        % mark OK
        filterIndices(iter) = true;
        continue
    end
    
    % target not seen !
    % reset seen iterations and flag
    iterSeen = 0;
    flagErrorCrossed = false;
end

% normalize error to [0, 1]
minim = min(targetErrors);
range = max(targetErrors) - minim;
errNorm = (targetErrors - minim) / range;

% normalized value of the error crossing
minErrorNorm = (minErrorAfterSeen - minim) / range;

% plot stuff
figure;
plot(1:N, targetSeen, 1:N, filterIndices, 1:N, errNorm);
hold on;
if minErrorNorm <= 1
    line([1, N], [minErrorNorm, minErrorNorm], 'Color', 'g', 'LineStyle', '-', 'LineWidth', 2);
    legend('targetSeen', 'filterIndices', 'errorNormalized', 'minErrorAfterSeenNorm');
else
    legend('targetSeen', 'filterIndices', 'errorNormalized')
end
axis([1, N, -0.1, 1.1]);
hold off;

% transform to logical
filterIndices = logical(filterIndices);

end

