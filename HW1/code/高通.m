I = imread(".\imgs\sky.jpg");
%I = rgb2gray(I);
[R,C,B] = size(I);
%高斯噪声
% P = 0.0+0.1*randn(R,C,B);
% I1 = uint8(255*(double(I)/255+P));
% %自带
% I5 = imnoise(I,'gaussian',0,0.01);
% imshow(I1);
% %泊松噪声
% Q = -400.0+random('poisson',400,R,C,B);
% I2 = uint8(double(I)+double(Q));
% %自带
% I6 = imnoise(I,'poisson');
% imshow(I2);
% I3 = I;
% %椒盐噪声
% SNR = 0.9;
% N = R*C*B*(1-SNR);
% for i = 0:N-1
%     x = randi(R);
%     y = randi(C);
%     z = randi(B);
%     o = rand(1);
%     if o>0.5
%         I3(x,y,z) = 0;
%     else
%         I3(x,y,z) = 255;
%     end
% end
% %自带
% I7 =imnoise(I1,'salt & pepper',0.003);
% imshow(I3);
% %均值滤波
% tic
% M1 = zeros(5,5);
% M1(:,:) = 1/25;
% for i = 1:B
%     for j = 1:R
%         for k = 1:C
%             S = 0;
%             for l = -2:2
%                 for u = -2:2
%                     %框取
%                     if j+l<1||j+l>R||k+u<1||k+u>C
%                     else
%                         S = S +I1(j+l,k+u,i)*M1(l+3,u+3);
%                     end
%                 end
%             end
%             I1(j , k , i) = S;
%         end
%     end
% end
% toc
% imshow(I1);
%高斯滤波
tic
sigma = 2;
M2 = zeros(3,3);
M2(1,2) = -1;
M2(2,1) = -1;
M2(2,3) = -1;
M2(3,2) = -1;
M2(2,2) = 4;
% M2 = zeros(5,5);
% for i = -2:2
%     for j = -2:2
%         %计算预设
%         M2(i+3,j+3) = 1/2/pi/sigma/sigma*exp(-(i*i+j*j)/2/sigma/sigma);
%     end
% end
%归一化
M2(:,:) = M2(:,:)/sum(sum(M2));
for i = 1:B
    for j = 1:R
        for k = 1:C
            S = 0;
            for l = -1:1
                for u = -1:1
                    %框取
                    if j+l<1||j+l>R||k+u<1||k+u>C
                    else
                        S = S +I2(j+l,k+u,i)*M2(l+2,u+2);
                    end
                end
            end
            I(j , k , i) = S;
        end
    end
end
toc
figure;
imshow(I2);
% %中值滤波
% tic
% I4 = I3;
% for i = 1:B
%     for j = 1:R
%         for k = 1:C
%             c=I3(max(j-2,1):min(j+2,R),max(k-2,1):min(k+2,C),i);
%             d = zeros(1,13);%存前13个 从大到小
%             e = zeros(1,25);%排列成列向量
%             [P,Q] = size(c);
%             O = P*Q;%总数
%             for p = 1:P
%                 for q = 1:Q
%                     e(1,(p-1)*Q+q) = c(p,q);
%                 end
%             end
%             for o = 1 : O
%                 for l = 1:min(13,o)
%                     if d(1,l)<e(1,o)
%                         for u = 13:-1:l+1
%                             d(1,u) = d(1,u-1);%后移
%                         end
%                         d(1,l) = e(1,o);%插入
%                         break
%                     end
%                 end
%             end
%             I4(j,k,i)  = d(1,uint8(O/2));%取中值
%         end
%     end
% end
% toc
% imshow(I4);
% tic
% H=fspecial('average', 5);
% I5=imfilter(I5,H,'replicate');
% toc
% imshow(I5);
% tic
% H=fspecial('gaussian', 5, sigma);
% I6=imfilter(I6,H,'replicate');
% toc
% imshow(I6);
% tic
% for b = 1:B
% I7(:,:,b)=medfilt2(I7(:,:,b),[5,5]);    
% end
% toc
% imshow(I7);
