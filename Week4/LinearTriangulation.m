function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 3) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 3) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points

N = size(x1,1);
t1 = -R1*C1;
T1 = [R1,t1];
P1 = K*T1;
t2 = -R2*C2
T2 = [R2,t2];
P2 = K*T2;

for i = 1 : N
    r1 = Vec2Skew([x1(i,:),1]);
    res1 = r1*P1;
    r2 = Vec2Skew([x2(i,:),1]);
    res2 = r2*P2;
    
    A = [res1;res2]; 
    
    [U,D,V] = svd(A);
    
    X(i,:) = (V(1:3,4)/V(4,4))';
end
end
