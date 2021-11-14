%{
-------------------------
Author : Li-Wei Yang
Date : 2021/11/14
Institution : National Taiwan University
Department : Biomechatronics Engineering
Status : Senior
-------------------------
Description:
    This is a script version of the live script hw2_live.mlx.
    This is a script that would numerically caluculate the IK and FK of a ER-7 arm.
    The script would show the forward transform matrix from the given DH,
    and then calculate the IK of a given pose in ZYX Euler angle, then plot
    the given pose with FK in robotics toolbox. The teach mode is open for
    the user to move the robot with joints twist.
    The ER-7 arm's orientation is intentionally constrained, the tip of the arm would
    always be vertically downward—-without this constraint, the inverse
    kinematics of this kind of underactuated robot would be too complex to
    solve. (Tried to use ikine in robotics toolbox but failed.)

Prerequisite:
    Robotics Toolbox - Peter Corke
    TransformMatrix.m

TODO:
    Symbolic foward and inverse kinematics
%}
clc;
clear;
close all;
syms th1 th2 th3 th4 th5;
T01 = TransformMatrix(0, 0, 358.50, th1, false);
T12 = TransformMatrix(-pi/2, 50, 0, th2, false);
T23 = TransformMatrix(0, 300, 35.3, th3, false);
T34 = TransformMatrix(0, 350, 0, th4, false);
T45 = TransformMatrix(-pi/2, 0, 251, th5, false);
T04 = T01.syms*T12.syms*T23.syms*T34.syms;
T05 = T01.syms*T12.syms*T23.syms*T34.syms*T45.syms;
% part b
simplify(T01.syms)
simplify(T12.syms)
simplify(T23.syms)
simplify(T34.syms)
simplify(T45.syms)
simplify(T05)

%---inverse kinematics---
% ZYX Euler angle
syms phi theta psi x y z
% for verify, can only enter double no symbolic
Tt = [cos(phi)*cos(theta), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), x;
    sin(phi)*cos(theta), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), y;
        -sin(theta), cos(theta)*sin(psi), cos(theta)*cos(psi), z;
        0, 0, 0, 1];

% for numerical solution
% enter the coordinates and orientation
% (1) pi/4 0 pi 600 100 0
% (2) pi/4 0 pi 600 100 100
% (3) -pi/4 0 pi 600 -100 100
Tt_subs = subs(Tt,[phi theta psi x y z], [pi/4 0 pi 600 100 0]);
% fill in phi value for afterward usage
phival = pi/4;
% solve for th1 th2 th3, because the tool is always vertically downward so
% the x y z of frame 4 is x y z+251
eq1 = T04(1,4) == Tt_subs(1,4);
eq2 = T04(2,4) == Tt_subs(2,4);
eq3 = T04(3,4) == Tt_subs(3,4)+251;
eqs = [eq1 eq2 eq3];
symtosolve = [th1 th2 th3];
S = solve(eqs, symtosolve,"Real",true);

T01_INV = inverse(T01);
T15 = T01_INV * Tt_subs;
simplify(T15);
% choose th2 < 0 for elbow up, subsitute for 
T15_subs = subs(T15, [th2 th3], [S.th2(2,1) S.th3(2,1)]);
th5 = -phi + S.th1(2,1);
% remeber to change 
th5_n = subs(th5,phi,phival);
th4_n = atan2(T15_subs(1,3),-T15_subs(3,3)) - S.th2(2,1) - S.th3(2,1);

% use robotics toolbox for visualization
l1 = Link('alpha', 0, 'a', 0, 'd', 358.5, 'modified');
l2 = Link('alpha', -pi/2, 'a', 50, 'd', 0, 'modified');
l3 = Link('alpha', 0, 'a', 300, 'd', 35.3, 'modified');
l4 = Link('alpha', 0, 'a', 350, 'd', 0, 'modified');
l5 = Link('alpha', -pi/2, 'a', 0, 'd', 251, 'modified');
ER7 = SerialLink([l1 l2 l3 l4 l5]);
syms th1 th2 th3 th4 th5
thsyms = [th1, th2, th3, th4, th5];
% choose elbow up configuration
thsyms_subs = subs(thsyms,[th1, th2, th3, th4, th5],[S.th1(2,1) S.th2(2,1) S.th3(2,1) th4_n th5_n]);
fk = ER7.fkine(thsyms);
% answer for part c (2)
thetas = double(thsyms_subs);
ER7.teach(thetas);
ER7.plot(thetas,'jointdiam', 1.5);

% for fix angle
%{
syms phi theta psi x y z
Tt = [ cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll), x;
    sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll), y;
        -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll), z;
        0, 0, 0, 1];
Tt_subs = subs(Tt,[roll pitch yaw x y z], [-pi/4 0 pi 600 -100 100])
%}
