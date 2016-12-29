function [robotErrors, targetErrors, targetSeen, nRobots, nValues] = readOutput(file)
%READOUTPUT Read output from a text file written by evaluate_omni_dataset
    [fid, errMsg] = fopen(file, 'r');
    if fid < 0
        error(errMsg);
    end
    
    % read number of values
    line = fgetl(fid);
    if ~ischar(line)
        error('Empty file')
    end
    nValues = str2double(line);
    
    % read targetSeen array
    line = fgetl(fid);
    if ~ischar(line)
        error('Not enough data in the file');
    end
    scan = textscan(line, '%d', nValues);
    targetSeen = scan{1};
    
    % read targetErrors array
    line = fgetl(fid);
    if ~ischar(line)
        error('Not enough data in the file')
    end
    scan = textscan(line, '%f', nValues);
    targetErrors = scan{1};
    
    % read robotErrors array
    robotErrors = [];
    line = fgetl(fid);
    while ischar(line)
        scan = textscan(line, '%f', nValues);
        robotErrors = [robotErrors scan{1}]; %#ok<AGROW>
        line = fgetl(fid);
    end
    nRobots = size(robotErrors, 2);
    
    fclose(fid);
end