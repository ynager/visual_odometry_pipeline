function final_keypoints = selectKeypoints(keypoints, delta, nbr_pts, viaMatrix_method)
% performs a non-maximum supression of a (2r + 1)*(2r + 1) box around the
% current maximum of input keypoints.
% input: keypoints -> a cornerPoints object, created by e.g. HarrisDetector
% output: subset of keypoints, with non-maximum supression applied

% get parameters
run('parameters.m');

if viaMatrix_method %note: needs ROUNDING of kp
    r = delta;
    num = nbr_pts;

    X = keypoints.Location(:,1);
    Y = keypoints.Location(:,2);

    final_keypoints = zeros(num, 2);

    temp_scores = zeros(ceil(max(Y(:))),ceil(max(X(:))));
    ind = sub2ind(size(temp_scores),round(Y),round(X));
    temp_scores(ind) = keypoints.Metric;
    temp_scores = padarray(temp_scores, [r r]);
    if processFrame.select_keypoints.sparseMatrix
        temp_scores = sparse(temp_scores);
    end
    
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

    is_final = logical(zeros(size(keypoints.Metric)));

    temp_metric = keypoints.Metric;
    k = size(keypoints.Metric,1);

    keypoints_location = keypoints.Location;
    
    for i = 1:nbr_pts
        [val, idx] = max(temp_metric);
        if val==0
            break
        end
        is_final(idx) = true;
        % non-maximum supression
        query_curr = keypoints.Location(idx,:);
        for j=1:k
            reference_curr = keypoints_location(j,:);
            not_valid = sum((query_curr-reference_curr).^2) < delta^2;
            if not_valid
                temp_metric(j)=0;
            end
        end
    end

    final_keypoints = keypoints(is_final);
    
end

end