function vec_b = nav2body (vec_n, DCMnb_m)
% nav2body: transforms vector from navigation frame to body
% frame.
%
% INPUT
%	vec: Nx3 matrix with [_n, _e, _d] parameters in the navigation
%		 frame.
%   DCMnb_m: Nx9 matrix with nav-to-body direct cosine matrices (DCM).
%            Each row of DCMnb_m contains the 9 elements of a particular DCMnb
%            matrix ordered as [a11 a21 a31 a12 a22 a32 a13 a23 a33].
%
% OUTPUT
%	vec_b: Nx3 matrix with [_x, _y, _z] parameters in the
%		   body frame.
%
% Reference:
%

vec_b = zeros(size(vec_n));

M = max(size(vec_n));

for k = 1:M
    
    dcmnb = reshape(DCMnb_m(k,:), 3, 3);
    vec_b(k,:) = ( dcmnb * ( vec_n(k,:)' ) )';
end

end
