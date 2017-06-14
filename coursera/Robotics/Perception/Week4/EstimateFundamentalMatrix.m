function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2

n=size(x1,1);

if ~isequal(size(x1),[n 2]) || ~isequal(size(x2),[n 2])
   error('xy coordinates must form nx2 array');
end
if n<8
   error('Must supply at least eight point correspondences');
end

% create predictor matrix
A = [x1(:,1).*x2(:,1) x1(:,1).*x2(:,2) x1(:,1) ...
      x1(:,2).*x2(:,1) x1(:,2).*x2(:,2) x1(:,2) ...
      x2(:,1) x2(:,2) ones(n,1)];

% initialize fundamental matrix
Fr3 = zeros(3,3);

% solve Af = 0 s.t f'*f = 1
[UA,SA,VA] = svd(A);
Fr3(:) = VA(:,9);

% now come up with rank 2 approximation
[U,S,V] = svd(Fr3);
F = U(:,1:2)*S(1:2,1:2)*V(:,1:2)';

% F = F / norm(F)

% F = F(1:2,1:2)

