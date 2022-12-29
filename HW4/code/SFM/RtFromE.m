function Rtbest=RtFromE(pair,frames)

% Decompose Essential Matrix
[R1, R2, t1, t2] = PoseEMat(pair.E); % MVG Page 257-259
%首先对E矩阵做SVD分解并直接做讲义中的矩阵变换UWV'等,t1=U(:,3),t2=U(:,-3)
%虽然没有看懂是怎么做SVD的，不过知道有四种可能就足够了
%本征矩阵就是对一些坐标系的变换中得到的东西，有x‘Ex = 0
% Four possible solution about [R|t]
Rt(:,:,1) =[R1 t1];
Rt(:,:,2) =[R1 t2];
Rt(:,:,3) =[R2 t1];
Rt(:,:,4) =[R2 t2];

% ToDo：triangulation 相当于我们将第一个矩阵[R1|t1]=[I|0],则P{1}=K[I|0] (matlab的话 一行代码可能就行)
% ？这里为什么要把R1t1设为I，0？
% 事实上，在triangulation的时候，默认第一个相机是参照系，所以变换矩阵就是原点，第二个相机有四种可能，分别对应
% 上面的四个Rt，注释里的R1t1和R2t2指的也是第一、二个相机对应的变换矩阵，而不是上面那个
% 在报告里有张图
 P{1} = frames.K*[eye(3),zeros(3,1)];

goodCnt = zeros(1,4);
for i=1:4 % ToDo:计算四种可能，你需要读懂vgg_X_from_xP_nonlin()函数（在相应的.m文件下），PS:如果你够仔细和机智，你会发现有个地方类似的用到了这个函数!!!
    clear X;
    % first:先得到的是相对1的K[R2|t2]
    %这里很奇怪，我本来以为按照上面所说应该把[R2|t2]传进去，但发现这样每次循环都用的是一个Rt，根本没法选出正确的那个，换了四种都试一遍，发现可以成功
    % P{2} = frames.K*[R2,t2]; 
    P{2} = frames.K*Rt(:,:,i); 
    % second:应用vgg_X_from_xP_nonlin()计算
    for j = 1 : size(pair.matches,2)
         x = [pair.matches(1,j) pair.matches(3,j);pair.matches(2,j) pair.matches(4,j)];
         X(:,j) = vgg_X_from_xP_nonlin(x,P,repmat([frames.imsize(2);frames.imsize(1)],1,length(P)));       
    end
    %注意事实上现在第三行还是带着Z的，要把Z消去
    X = X(1:3,:) ./ X([4 4 4],:);
    % third:选择（这步我已经给出了，不需要你们写，你们可以根据这句话思考前面second和下一步怎么写）
    dprd = Rt(3,1:3,i) * ((X(:,:) - repmat(Rt(1:3,4,i),1,size(X,2))));%最后选择的规则即为满足(X-C)*R(3,:)是否大于0
    % forth:计算了满足z>0以及(X-C)*R(3,:)>0的个数
    %这里的z指的是第三个分量？
    goodCnt(i) = sum(X(3,:)>0 & dprd > 0);

end


% pick one solution from the four
fprintf('%d\t%d\t%d\t%d\n',goodCnt);
[~, bestIndex]=max(goodCnt);

Rtbest = Rt(:,:,bestIndex);
    
