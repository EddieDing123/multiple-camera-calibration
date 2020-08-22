%P1 = R*P2+T
cameraParams= calibrationOptimized.CameraParameters;
RTcnc1 = calibrationOptimized.Extrinsics;
RTpnc1 = calibrationOptimized.PatternPositions;
n = 11;%set here to project different patterns
RTcnc1{1} = eye(4,4);
for i = 1: numel(RTcnc1) % for each camera
   % for n = 1: numel(RTpnc1{1}(1,1,:)) % for each image
        
        rR =RTcnc1{i}(1:3, 1:3);
        rt = RTcnc1{i}(4, 1:3)';
        Rext = RTpnc1{1}(1:3, 1:3, n);
        text = RTpnc1{1}(4, 1:3, n)';
        
        K = cameraParams{i}.IntrinsicMatrix;
        
        %if (all(isnan(points(:))) == 0 && all(all(Rext))~=0)
            
            R = Rext*rR;
            t = (rt +rR'*text)';
            
            
            cameraMatrix = [R; t(:)'] * K;
            r_dist = cameraParams{i}.RadialDistortion;
            t_dist = cameraParams{i}.TangentialDistortion;
            
            projectedPointsraw =  [genPoints ones(size(genPoints, 1), 1)] * cameraMatrix;
            
            projectedPointsnorm = bsxfun(@rdivide, projectedPointsraw(:, 1:2), projectedPointsraw(:, 3));
            
            distortedPoints =vision.internal.calibration.distortPoints(projectedPointsnorm, K, r_dist, t_dist);
            
            im = imread(convertCharsToStrings(ifn{i,n}));
            figure();
            imshow(im);
            hold on;
            for j=1:length(worldPoints)
                x=distortedPoints(j,1);
                y=distortedPoints(j,2);
                plot(x,y,'o');
                hold on;
            end
            hold off;
end

