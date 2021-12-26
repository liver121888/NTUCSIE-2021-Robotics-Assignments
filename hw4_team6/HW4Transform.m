clear;

%some point sets 
%(Xw, Yw): 280 500, 380 400, 430 450
%(Xc, Yc): 282 394, 668 579, 766 393

%we used these three point sets to calculate the transform matrix
%(Xw, Yw): 230 230 300 200 175 200
%(Xc, Yc): 666 713 859 638 614 888


%measured using ruler for three times, then take the average
%formula: Dw(P1w, P2w)/Dc(P1c, P2c), D stands for distance
%unit: mm/pixel
scale = 0.3617993849;

w = [[230,300,175];[230,200,200];[1,1,1]];
%only Xc, Yc should be scaled, the 1 of homogeneous coordinate shold not
c = [[666* scale, 859 * scale,614* scale];[713* scale,638* scale,888* scale];[1,1,1]];

T = w/c;

%given a test point with Xc, Yc, times the transform matrix 
%would get us Xw, Yw
ptestc = [270*scale; 646*scale; 1];
ptestw = T * ptestc;
