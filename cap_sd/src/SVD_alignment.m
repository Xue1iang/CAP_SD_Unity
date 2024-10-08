function [alignedA,R,t] = alignPoints(A, B)
%  ALIGNPOINTS *Function for SVD*
    % 
    centroidA = mean(A, 1);
    centroidB = mean(B, 1);
    % 
    A_centered = A - centroidA;
    B_centered = B - centroidB;
    % 
    H = A_centered' * B_centered;
    % SVD
    [U, S, V] = svd(H);
    % 
    R = V * U';
    % 
    if det(R) < 0
       V(:,3) = V(:,3) * -1;
       R = V * U';
    end
    % 
    t = centroidB - (R * centroidA')';
    % Align A to B
    alignedA = (R * A')' + repmat(t, size(A, 1), 1);
end