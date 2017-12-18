function final_keypoints = selectKeypoints(keypoints)
% performs a non-maximum supression of a (2r + 1)*(2r + 1) box around the
% current maximum of input keypoints.
% input: keypoints -> a cornerPoints object, created by e.g. HarrisDetector
% output: subset of keypoints, with non-maximum supression applied

% get parameters
run('parameters.m');

if select_uniform.viaMatrix_method
    r = select_uniform.delta;
    num = select_uniform.nbr_pts;

    X = keypoints.Location(:,1);
    Y = keypoints.Location(:,2);

    final_keypoints = zeros(num, 2);

    temp_scores = zeros(ceil(max(Y(:))),ceil(max(X(:))));
    ind = sub2ind(size(temp_scores),ceil(Y),ceil(X));
    temp_scores(ind) = keypoints.Metric;
    temp_scores = padarray(temp_scores, [r r]);
    % temp_scores = sparse(temp_scores);
    for i = 1:num
        [val, kp] = max(temp_scores(:));
        if val==0
            final_keypoints(i:end,:) = [];
            break;
        end
        [row, col] = ind2sub(size(temp_scores), kp);
        kp = [col row];
        final_keypoints(i, :) = kp - r;
        temp_scores(kp(2)-r:kp(2)+r, kp(1)-r:kp(1)+r) = ...
            zeros(2*r + 1, 2*r + 1);
    end

    final_keypoints = cornerPoints(final_keypoints);

else
    delta = select_uniform.delta;

    is_final = logical(zeros(size(keypoints.Metric)));

    temp_metric = keypoints.Metric;
    k = size(keypoints.Metric,1);

    keypoints_location = keypoints.Location;
    
    for i = 1:k
        [val, idx] = max(temp_metric);
        if val==0
            break
        end
        is_final(idx) = true;
        % non-maximum supression
        query_curr = keypoints.Location(idx,:);
        for j=1:k
            reference_curr = keypoints_location(j,:);
            not_valid = norm(query_curr-reference_curr) < delta;
            if not_valid
                temp_metric(j)=0;
            end
        end
    end

    final_keypoints = keypoints(is_final);
    
end

end