function final_keypoints = selectKeypoints(keypoints)
% performs a non-maximum supression of a (2r + 1)*(2r + 1) box around the
% current maximum of input keypoints.
% input: keypoints -> a cornerPoints object, created by e.g. HarrisDetector
% output: subset of keypoints, with non-maximum supression applied

% get parameters
run('parameters.m');

delta = select_uniform.delta;

is_final = logical(zeros(size(keypoints.Metric)));

temp_metric = keypoints.Metric;
k = size(keypoints.Metric,1);

for i = 1:k
    [val, idx] = max(temp_metric);
    if val==0
        break
    end
    is_final(idx) = true;
    % non-maximum supression
    query_curr = keypoints.Location(idx,:);
    for j=1:k
        reference_curr = keypoints.Location(j,:);
        not_valid = norm(query_curr-reference_curr) < delta;
        if not_valid
            temp_metric(j)=0;
        end
    end
end

final_keypoints = keypoints(is_final);

end