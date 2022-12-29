function path = getPath(MeshSize, tracks)
%GETPATH Compute the bundled camera path
%   track is the set of trajectories of the input video, backList indicate
%   the background trajectories.     
    nFrames = tracks.nFrame;
    if nFrames < 2
        error('Wrong inputs') ;
    end
    path = zeros(nFrames, MeshSize, MeshSize, 3, 3);
    for row = 1:MeshSize
        for col = 1:MeshSize
            path(1, row, col, :, :) = eye(3);
        end
    end
    fprintf('%5d', 1);
    for frameIndex = 2:nFrames
        fprintf('%5d', frameIndex);
        if mod(frameIndex, 20) == 0
            fprintf('\n') ;
        end  
        [f1,f2] = getF(tracks,frameIndex-1);
        [homos] = NewWarping(f1,f2,tracks.videoHeight,tracks.videoWidth,tracks.videoHeight/MeshSize,tracks.videoWidth/MeshSize,2.0);
        for row = 1:MeshSize
            for col = 1:MeshSize
%                 for i = 1 : 3
%                     for j = 1 : 3
%                         for k = 1 : 3
%                             path(frameIndex, row, col, i, j) = path(frameIndex-1, row, col, :, :)*homos(row, col, :, :);
%                         end
%                     end
%                 end
                A = zeros(3,3);
                B = zeros(3,3);
                A(:,:) = path(frameIndex-1, row, col, :,:);
                B(:,:) = homos(row, col, :, :);               
                C = B*A;
%               C = C ./ C(3,3);  并没有必要，返回的时候基本都是1，不会有很大影响
                path(frameIndex, row, col, :, :) = C(:,:);
            end
        end
        %TODO：按照论文中的方法，得到path的值，需要用到NewWarping函数和TrackLib.m中的getF函数           
    end
end

