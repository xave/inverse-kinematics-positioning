function pfinal=inverse_kinematics(xd,yd,zd)

% -------------------------------------------------------------------------
% Initialization of Robosim
% (get-agent-joints r)
global ROBOSIMX
InitRoboD3D(900,600);
LoadLisp('redundant-robot.lsp');
%--------------------------------------------------------------------------
% Variable definitions
%--------------------------------------------------------------------------
d2r=pi/180; %Converts degrees to radians because matlab uses radians
% xd=4;
% yd=3;
% zd=2;

%--------------------------------------------------------------------------
% Setting inital joint angles
%--------------------------------------------------------------------------
DriveAgent(40,45,45,45,45,45,32,12,46)
% DriveAgent(40,0,0,0,0,0,0,45,45)

% Ideally, one would start the robot at the home position. However, the
% home position for this robot happens to be at a singular position. Thus,
% the robot must be initiated at a position different than the home
% position.
%--------------------------------------------------------------------------
% Desired Position
%--------------------------------------------------------------------------
% xd=0;%test
% yd=6;%test
% zd=32;%test
%--------------------------------------------------------------------------
% Current Joint values
%--------------------------------------------------------------------------

joint_s=GetAgentJoint('r');%The "s" denotes that joint is comprised of a string
len=length(joint_s);%All this other stuff is just converting the string ROBOSIM provides to actual numbers.
joint_s=joint_s(2:len-1);%This strips the parentheses off of the ends of the string so I can change everything to numbers.
joint=str2num(joint_s);%Vector with initial joint values.

t1=d2r*joint(1);
t2=d2r*joint(2);
t3=d2r*joint(3);
t4=d2r*joint(4);
t5=d2r*joint(5);
t6=d2r*joint(6);
t7=d2r*joint(7);
t8=d2r*joint(8);
%--------------------------------------------------------------------------
% Current Position
%--------------------------------------------------------------------------
xc=32*cos(t1)*sin(t2)-16*(-cos(t1)*cos(t4)*sin(t2)-(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3))*sin(t4))-8*(-cos(t6)*(cos(t1)*cos(t4)*sin(t2)+(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3))*sin(t4))-(cos(t5)*(cos(t4)*(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3))-cos(t1)*sin(t2)*sin(t4))+(-cos(t3)*sin(t1)-cos(t1)*cos(t2)*sin(t3))*sin(t5))*sin(t6));
yc=-32*cos(t1)-16*(cos(t1)*cos(t4)-cos(t3)*sin(t1)*sin(t4))-8*(-cos(t6)*(-cos(t1)*cos(t4)+cos(t3)*sin(t1)*sin(t4))-(cos(t5)*(cos(t3)*cos(t4)*sin(t1)+cos(t1)*sin(t4))-sin(t1)*sin(t3)*sin(t5))*sin(t6));
zc=32*sin(t1)*sin(t2)-16*(-cos(t4)*sin(t1)*sin(t2)-(cos(t2)*cos(t3)*sin(t1)+cos(t1)*sin(t3))*sin(t4))-8*(-cos(t6)*(cos(t4)*sin(t1)*sin(t2)+(cos(t2)*cos(t3)*sin(t1)+cos(t1)*sin(t3))*sin(t4))-(cos(t5)*(cos(t4)*(cos(t2)*cos(t3)*sin(t1)+cos(t1)*sin(t3))-sin(t1)*sin(t2)*sin(t4))+(cos(t1)*cos(t3)-cos(t2)*sin(t1)*sin(t3))*sin(t5))*sin(t6));
pc=[xc yc zc]';%Current position vector
%--------------------------------------------------------------------------
% The loop
%--------------------------------------------------------------------------
ind=10000;
 x_traj = linspace(xc,xd,ind);
 y_traj = linspace(yc,yd,ind);
 z_traj = linspace(zc,zd,ind);
 r2d=180/pi; %converts degrees to Radians
for ii = 2:ind
    theta_new = position_the_robot(x_traj(ii), y_traj(ii),z_traj(ii));
    
    t1=r2d*theta_new(1);% theta_new already in radians, so no conversion.
    t2=r2d*theta_new(2);
    t3=r2d*theta_new(3);
    t4=r2d*theta_new(4);
    t5=r2d*theta_new(5);
    t6=r2d*theta_new(6);
    t7=r2d*theta_new(7);
    t8=r2d*theta_new(8);
    xdc=[x_traj(ii) y_traj(ii), z_traj(ii)]';
end
    DriveAgent(1,t1,t2,t3,t4,t5,t6,t7,t8);

%--------------------------------------------------------------------------
% Final Position
%--------------------------------------------------------------------------
    t1=theta_new(1);% theta_new already in radians, so no conversion.
    t2=theta_new(2);
    t3=theta_new(3);
    t4=theta_new(4);
    t5=theta_new(5);
    t6=theta_new(6);
    t7=theta_new(7);
    t8=theta_new(8);
    xfinal=32*cos(t1)*sin(t2)-16*(-cos(t1)*cos(t4)*sin(t2)-(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3))*sin(t4))-8*(-cos(t6)*(cos(t1)*cos(t4)*sin(t2)+(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3))*sin(t4))-(cos(t5)*(cos(t4)*(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3))-cos(t1)*sin(t2)*sin(t4))+(-cos(t3)*sin(t1)-cos(t1)*cos(t2)*sin(t3))*sin(t5))*sin(t6));
    yfinal=-32*cos(t1)-16*(cos(t1)*cos(t4)-cos(t3)*sin(t1)*sin(t4))-8*(-cos(t6)*(-cos(t1)*cos(t4)+cos(t3)*sin(t1)*sin(t4))-(cos(t5)*(cos(t3)*cos(t4)*sin(t1)+cos(t1)*sin(t4))-sin(t1)*sin(t3)*sin(t5))*sin(t6));
    zfinal=32*sin(t1)*sin(t2)-16*(-cos(t4)*sin(t1)*sin(t2)-(cos(t2)*cos(t3)*sin(t1)+cos(t1)*sin(t3))*sin(t4))-8*(-cos(t6)*(cos(t4)*sin(t1)*sin(t2)+(cos(t2)*cos(t3)*sin(t1)+cos(t1)*sin(t3))*sin(t4))-(cos(t5)*(cos(t4)*(cos(t2)*cos(t3)*sin(t1)+cos(t1)*sin(t3))-sin(t1)*sin(t2)*sin(t4))+(cos(t1)*cos(t3)-cos(t2)*sin(t1)*sin(t3))*sin(t5))*sin(t6));
    pfinal=[xfinal,yfinal,zfinal]';
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% AUXILARY FUNCTIONS
%--------------------------------------------------------------------------
function theta_new=position_the_robot(xd, yd, zd)
% Current Joint values
%--------------------------------------------------------------------------

joint_s=GetAgentJoint('r');%The "s" denotes that joint is comprised of a string
len=length(joint_s);%All this other stuff is just converting the string ROBOSIM provides to actual numbers.
joint_s=joint_s(2:len-1);%This strips the parentheses off of the ends of the string so I can change everything to numbers.
joint=str2num(joint_s);%Vector with initial joint values.
d2r=pi/180; %Converts degrees to radians because matlab uses radians
t1=d2r*joint(1);
t2=d2r*joint(2);
t3=d2r*joint(3);
t4=d2r*joint(4);
t5=d2r*joint(5);
t6=d2r*joint(6);
t7=d2r*joint(7);
t8=d2r*joint(8);
%--------------------------------------------------------------------------
% Current Position Vector
%--------------------------------------------------------------------------
xc=32*cos(t1)*sin(t2)-16*(-cos(t1)*cos(t4)*sin(t2)-(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3))*sin(t4))-8*(-cos(t6)*(cos(t1)*cos(t4)*sin(t2)+(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3))*sin(t4))-(cos(t5)*(cos(t4)*(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3))-cos(t1)*sin(t2)*sin(t4))+(-cos(t3)*sin(t1)-cos(t1)*cos(t2)*sin(t3))*sin(t5))*sin(t6));
yc=-32*cos(t1)-16*(cos(t1)*cos(t4)-cos(t3)*sin(t1)*sin(t4))-8*(-cos(t6)*(-cos(t1)*cos(t4)+cos(t3)*sin(t1)*sin(t4))-(cos(t5)*(cos(t3)*cos(t4)*sin(t1)+cos(t1)*sin(t4))-sin(t1)*sin(t3)*sin(t5))*sin(t6));
zc=32*sin(t1)*sin(t2)-16*(-cos(t4)*sin(t1)*sin(t2)-(cos(t2)*cos(t3)*sin(t1)+cos(t1)*sin(t3))*sin(t4))-8*(-cos(t6)*(cos(t4)*sin(t1)*sin(t2)+(cos(t2)*cos(t3)*sin(t1)+cos(t1)*sin(t3))*sin(t4))-(cos(t5)*(cos(t4)*(cos(t2)*cos(t3)*sin(t1)+cos(t1)*sin(t3))-sin(t1)*sin(t2)*sin(t4))+(cos(t1)*cos(t3)-cos(t2)*sin(t1)*sin(t3))*sin(t5))*sin(t6));
pc=[xc yc zc]';%Current position vector
%--------------------------------------------------------------------------
% Desired Position Vector
%--------------------------------------------------------------------------
pd=[xd yd zd]';%Desired position vector
%--------------------------------------------------------------------------
% Desired Velocity
%--------------------------------------------------------------------------
nrm=abs(norm(xd-xc));
a=.1;% a is a scaling factor
% vd=a*(pd-pc)/nrm; %normalized desired velocity
vd=a*(pd-pc); %non-normalized velocity
% -------------------------------------------------------------------------
% Formation of the Jacobian
% -------------------------------------------------------------------------
%       First Row of Jacobian
% -------------------------------------------------------------------------
df1dt1=-32*sin(t1)*sin(t2)-16*(cos(t4)*sin(t1)*sin(t2)-((-cos(t2))*cos(t3)*sin(t1)-cos(t1)*sin(t3))*sin(t4))-8*((-cos(t6))*((-cos(t4))*sin(t1)*sin(t2)+((-cos(t2))*cos(t3)*sin(t1)-cos(t1)*sin(t3))*sin(t4))-(cos(t5)*(cos(t4)*((-cos(t2))*cos(t3)*sin(t1)-cos(t1)*sin(t3))+sin(t1)*sin(t2)*sin(t4))+((-cos(t1))*cos(t3)+cos(t2)*sin(t1)*sin(t3))*sin(t5))*sin(t6));
df1dt2=32*cos(t1)*cos(t2)-16*(-cos(t1)*cos(t2)*cos(t4)+cos(t1)*cos(t3)*sin(t2)*sin(t4))-8*(-cos(t6)*(cos(t1)*cos(t2)*cos(t4)-cos(t1)*cos(t3)*sin(t2)*sin(t4))-(cos(t5)*(-cos(t1)*cos(t3)*cos(t4)*sin(t2)-cos(t1)*cos(t2)*sin(t4))+cos(t1)*sin(t2)*sin(t3)*sin(t5))*sin(t6));
df1dt3=16*(-cos(t3)*sin(t1)-cos(t1)*cos(t2)*sin(t3))*sin(t4)-8*(-cos(t6)*(-cos(t3)*sin(t1)-cos(t1)*cos(t2)*sin(t3))*sin(t4)-(cos(t4)*cos(t5)*(-cos(t3)*sin(t1)-cos(t1)*cos(t2)*sin(t3))+(-cos(t1)*cos(t2)*cos(t3)+sin(t1)*sin(t3))*sin(t5))*sin(t6));
df1dt4=-16*(-cos(t4)*(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3) )+cos(t1)*sin(t2)*sin(t4) )-8*(-cos(t6)*(cos(t4)*(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3) )-cos(t1)*sin(t2)*sin(t4) )-cos(t5)*(-cos(t1)*cos(t4)*sin(t2)-(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3) )*sin(t4) )*sin(t6));
df1dt5=8*(cos(t5)*(-cos(t3)*sin(t1)-cos(t1)*cos(t2)*sin(t3))-(cos(t4)*(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3))-cos(t1)*sin(t2)*sin(t4))*sin(t5))*sin(t6);
df1dt6=-8*(-cos(t6)*(cos(t5)*(cos(t4)*(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3) )-cos(t1)*sin(t2)*sin(t4) )+(-cos(t3)*sin(t1)-cos(t1)*cos(t2)*sin(t3) )*sin(t5) )+(cos(t1)*cos(t4)*sin(t2)+(cos(t1)*cos(t2)*cos(t3)-sin(t1)*sin(t3) )*sin(t4) )*sin(t6));
df1dt7=0;
df1dt8=0;
% -------------------------------------------------------------------------
%       Second Row of Jacobian
% -------------------------------------------------------------------------
df2dt1=0;
df2dt2=32*sin(t2)-16*(-cos(t4)*sin(t2)-cos(t2)*cos(t3)*sin(t4))-8*(-cos(t6)*(cos(t4)*sin(t2)+cos(t2)*cos(t3)*sin(t4))-(cos(t5)*(cos(t2)*cos(t3)*cos(t4)-sin(t2)*sin(t4))-cos(t2)* sin(t3)*sin(t5))*sin(t6));
df2dt3=-16 *sin(t2) *sin(t3) *sin(t4)-8 *(cos(t6)* sin(t2)* sin(t3)* sin(t4)-(-cos(t4) *cos(t5)* sin(t2)* sin(t3)-cos(t3)* sin(t2)* sin(t5))* sin(t6));
df2dt4=-16* (-cos(t3)* cos(t4)* sin(t2)-cos(t2)* sin(t4))-8* (-cos(t6)* (cos(t3)* cos(t4)* sin(t2)+cos(t2)* sin(t4))-cos(t5)* (cos(t2)* cos(t4)-cos(t3)* sin(t2)* sin(t4))* sin(t6));
df2dt5=8* (-cos(t5)* sin(t2)* sin(t3)-(cos(t3)* cos(t4)* sin(t2)+cos(t2)* sin(t4))* sin(t5))* sin(t6);
df2dt6=-8 *(-cos(t6)* (cos(t5)* (cos(t3)* cos(t4)* sin(t2)+cos(t2)* sin(t4))-sin(t2)* sin(t3)* sin(t5))+(-cos(t2)* cos(t4)+cos(t3)* sin(t2)* sin(t4))* sin(t6));
df2dt7=0;
df2dt8=0;
% -------------------------------------------------------------------------
%       Third Row of Jacobian
% -------------------------------------------------------------------------
df3dt1=32*cos(t1) *sin(t2)-16* (-cos(t1) *cos(t4) *sin(t2)-(cos(t1) *cos(t2)* cos(t3)-sin(t1)* sin(t3))* sin(t4))-8* (-cos(t6)* (cos(t1)* cos(t4)* sin(t2)+(cos(t1)* cos(t2)* cos(t3)-sin(t1) *sin(t3))* sin(t4))-(cos(t5)* (cos(t4)* (cos(t1)* cos(t2)* cos(t3)-sin(t1)* sin(t3))-cos(t1)* sin(t2)* sin(t4))+(-cos(t3)* sin(t1)-cos(t1)* cos(t2)* sin(t3))* sin(t5))* sin(t6));
df3dt2=32* cos(t2)* sin(t1)-16* (-cos(t2)* cos(t4) *sin(t1)+cos(t3)* sin(t1) *sin(t2)* sin(t4))-8 *(-cos(t6)* (cos(t2)* cos(t4)* sin(t1)-cos(t3)* sin(t1)* sin(t2) *sin(t4))-(cos(t5)* (-cos(t3)* cos(t4)* sin(t1)* sin(t2)-cos(t2)* sin(t1)* sin(t4))+sin(t1) *sin(t2) *sin(t3)* sin(t5))* sin(t6));
df3dt3=16* (cos(t1)* cos(t3)-cos(t2)* sin(t1)* sin(t3))* sin(t4)-8* (-cos(t6)* (cos(t1)* cos(t3)-cos(t2)* sin(t1) *sin(t3))* sin(t4)-(cos(t4)* cos(t5) *(cos(t1)* cos(t3)-cos(t2)* sin(t1)* sin(t3))+(-cos(t2)* cos(t3)* sin(t1)-cos(t1) *sin(t3)) *sin(t5))* sin(t6));
df3dt4=-16 *(-cos(t4)* (cos(t2)* cos(t3)* sin(t1)+cos(t1)* sin(t3))+sin(t1) *sin(t2) *sin(t4))-8* (-cos(t6)* (cos(t4)* (cos(t2)* cos(t3)* sin(t1)+cos(t1)* sin(t3))-sin(t1)* sin(t2)* sin(t4))-cos(t5)* (-cos(t4)* sin(t1)* sin(t2)-(cos(t2)* cos(t3) *sin(t1)+cos(t1) *sin(t3)) *sin(t4)) *sin(t6));
df3dt5=8 *(cos(t5)* (cos(t1)* cos(t3)-cos(t2)* sin(t1)* sin(t3))-(cos(t4)* (cos(t2)* cos(t3)* sin(t1)+cos(t1) *sin(t3))-sin(t1) *sin(t2) *sin(t4))* sin(t5))* sin(t6);
df3dt6=-8* (-cos(t6)* (cos(t5)* (cos(t4) *(cos(t2)* cos(t3) *sin(t1)+cos(t1) *sin(t3))-sin(t1)* sin(t2)* sin(t4))+(cos(t1)* cos(t3)-cos(t2)* sin(t1)* sin(t3))* sin(t5))+(cos(t4)* sin(t1)* sin(t2)+(cos(t2)* cos(t3)* sin(t1)+cos(t1)* sin(t3))* sin(t4)) *sin(t6));
df3dt7=0;
df3dt8=0;
% -------------------------------------------------------------------------
% Jacobian of end effector (Je)
% -------------------------------------------------------------------------
Je=zeros(3,8); 
letters={'df' 'dt'};
for ii=1:3
    for jj=1:8
jacobcall=[letters{1},num2str(ii),letters{2},num2str(jj)];
        Je(ii,jj)=eval(jacobcall);
    end
end
% -------------------------------------------------------------------------
% Pseudo-inverse of Jacobian (J_pseudo)
% -------------------------------------------------------------------------
% Jp=Je.'*inv(Je*Je.')%Alternate form of Jacobian. Only works at full rank.
[U,S,V]=svd(Je);
S_pseudo=S'*inv(S*S');%Pseudoinverse of sigma
Je_pseudo=V*S_pseudo*U'; %pseudoinverse of Jacobian
% -------------------------------------------------------------------------
% Calculation of joint velocity
% -------------------------------------------------------------------------
omega=Je_pseudo*vd; % Joint velocities required to give the desired velocities. 8x1 matrix
theta_c=[t1 t2 t3 t4 t5 t6 t7 t8]';%Current joint variables
b=1; %scaling constant 
theta_new=theta_c + b*omega;
%theta_new is in radians.


function DriveAgent(step,a,b,c,d,e,f,g,h)

% This function was tricky to create. You have to use triple apostrophes to
% place just ONE apostrophe within a string. But since the string below
% has to be evaluated twice, you have to DOUBLE the tripling for a grand 
% total of "6" apostrophes!

ss=num2str(step); %step size
q1=num2str(a); %where the ith q is the d or theta in DEGREES!!!!!!
q2=num2str(b);
q3=num2str(c);
q4=num2str(d);
q5=num2str(e);
q6=num2str(f);
q7=num2str(g);
q8=num2str(h);
 
drivecall=['EvalLisp(''(drive-agent R',' ','''''','(',q1,' ',q2,' ',q3,' ',q4,' ',q5,' ',q6,' ',q7,' ',q8,')',' ',ss,')',''')'];
eval(drivecall);


