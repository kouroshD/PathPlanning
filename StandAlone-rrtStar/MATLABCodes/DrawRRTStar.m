clc;clear all;close all;

path_id= fopen(strcat('/home/nasa/Datalog/Planner/01optTraj.txt')); 
pathStruc=textscan(path_id,'%f %f %f');fclose(path_id);

path=[ pathStruc{1},pathStruc{2},pathStruc{3}]; 

X=path(:,1);Y=path(:,2);Z=path(:,3);

figure;
% set(gcf,'Renderer','opengl')
% set(gcf,'Renderer','zbuffer')

plot3(X,Y,Z,'-*');xlabel('x');hold on;
plot3(X(1),Y(1),Z(1),'*r');hold on;
ylabel('y');zlabel('z');title('Optimal Trajectory');hold on;
xlim([-10,10]);ylim([-10,10]);zlim([-10,10]);grid on;

% figure;
 %patch([-3.5 -3.5 -6.5 -6.5], [-3.5 -6.5 -6.5 -3.5],[1 0 0]);hold on

% //plot(path(:,1),path(:,2),'-*')

%% VERTICES
% 
% path_id= fopen(strcat('/home/nasa/Datalog/Planner/01Vertices.txt')); 
% pathStruc=textscan(path_id,'%f %f %f');fclose(path_id);
% 
% path=[ pathStruc{1},pathStruc{2},pathStruc{3}]; 
% 
% X=path(:,1);Y=path(:,2);Z=path(:,3);
% 
% figure;
% plot3(X,Y,Z,'*');xlabel('x');hold on;
% % plot3(X(1),Y(1),Z(1),'*r');hold on;
% ylabel('y');zlabel('z');title('Vertices');


%% OBSTACLE & REGION

% % % figure;
% % format compact 
% % h(1) = axes('Position',[0.2 0.2 0.6 0.6]);
% % WorkingSpace = [1 1 -1; 
% %         -1 1 -1; 
% %         -1 1 1; 
% %         1 1 1; 
% %         -1 -1 1;
% %         1 -1 1; 
% %         1 -1 -1;
% %         -1 -1 -1].*10;
% %  Faces = [1 2 3 4; 
% %        4 3 5 6; 
% %        6 7 8 5; 
% %        1 2 8 7; 
% %        6 7 1 4; 
% %        2 3 5 8];
% % 
% % obs1 = [1 1 -1; 
% %         -1 1 -1; 
% %         -1 1 1; 
% %         1 1 1; 
% %         -1 -1 1;
% %         1 -1 1; 
% %         1 -1 -1;
% %         -1 -1 -1];
% % obs1(:,1)=obs1(:,1).*3;
% % obs1(:,2)=obs1(:,2).*3;
% % obs1(:,3)=obs1(:,3).*4;   
    

% I defined a new cube whose length is 1 and centers at the origin.
% vert2 = WorkingSpace * 0.5;  
% fac2 = obs1;

% % patch('Faces',Faces,'Vertices',WorkingSpace,'FaceColor','b');  % patch function
% % axis([-10, 10, -10, 10, -10, 1]);
% % axis equal;
% % hold on;
% % patch('Faces', Faces, 'Vertices', obs1, 'FaceColor', 'r');
% % material metal;
% % alpha('color');
% % alphamap('rampdown');
% % view(3);



 A = [3 3 4];
 B = [3 3 -4];
 C = [3 -3 4];
 D = [3 -3 -4];
 E = [-3 3 4];
 F = [-3 3 -4];
 G = [-3 -3 4];
 H = [-3 -3 -4];
 
 center2=[5 -4 0];
 A2 =center2+ [4 4 5];
 B2 =center2+ [4 4 -5];
 C2 =center2+ [4 -4 5];
 D2 =center2+ [4 -4 -5];
 E2 =center2+ [-4 4 5];
 F2 =center2+ [-4 4 -5];
 G2 =center2+ [-4 -4 5];
 H2 =center2+ [-4 -4 -5];
 
 
%  
% %  P = [A;B;F;H;G;C;A;D;E;H;F;D;E;C;G;B];
% %  plot3(obs1(:,1),obs1(:,2),obs1(:,3))
% %  axis equal

x=[
 H(1) F(1) B(1) D(1) H(1) % bottom
 G(1) E(1) A(1) C(1) G(1) % top
 G(1) H(1) D(1) C(1) G(1) % left
 E(1) F(1) B(1) A(1) E(1) % right
];
y=[
 H(2) F(2) B(2) D(2) H(2) % bottom
 G(2) E(2) A(2) C(2) G(2) % top
 G(2) H(2) D(2) C(2) G(2) % left
 E(2) F(2) B(2) A(2) E(2) % right
];
z=[
 H(3) F(3) B(3) D(3) H(3) % bottom
 G(3) E(3) A(3) C(3) G(3) % top
 G(3) H(3) D(3) C(3) G(3) % left
 E(3) F(3) B(3) A(3) E(3) % right
];
hold on;
 line(x',y',z','color',[0 0 0]);
 view(3);
 set(gca,'xlim',[-1 2],...
  'ylim',[-1 2],...
  'zlim',[-1 2]);
 xlabel('XXX');
 ylabel('YYY');
 zlabel('ZZZ');
hold on;
plot3(X,Y,Z,'-*');xlabel('x');hold on;
plot3(X(1),Y(1),Z(1),'or','MarkerSize',10,'MarkerFaceColor','y','MarkerFaceColor','R');hold on;
ylabel('y');zlabel('z');title('Optimal Trajectory');hold on;
xlim([-10,10]);ylim([-10,10]);zlim([-10,10]);grid on;



%%

path_id= fopen(strcat('/home/nasa/Datalog/Planner/02optTraj.txt')); 
pathStruc=textscan(path_id,'%f %f %f');fclose(path_id);

path=[ pathStruc{1},pathStruc{2},pathStruc{3}]; 

X=path(:,1);Y=path(:,2);Z=path(:,3);


figure;
x2=[
 H2(1) F2(1) B2(1) D2(1) H2(1) % bottom
 G2(1) E2(1) A2(1) C2(1) G2(1) % top
 G2(1) H2(1) D2(1) C2(1) G2(1) % left
 E2(1) F2(1) B2(1) A2(1) E2(1) % right
];
y2=[
 H2(2) F2(2) B2(2) D2(2) H2(2) % bottom
 G2(2) E2(2) A2(2) C2(2) G2(2) % top
 G2(2) H2(2) D2(2) C2(2) G2(2) % left
 E2(2) F2(2) B2(2) A2(2) E2(2) % right
];
z2=[
 H2(3) F2(3) B2(3) D2(3) H2(3) % bottom
 G2(3) E2(3) A2(3) C2(3) G2(3) % top
 G2(3) H2(3) D2(3) C2(3) G2(3) % left
 E2(3) F2(3) B2(3) A2(3) E2(3) % right
];


 line(x',y',z','color',[0 0 0]);hold on;
 line(x2',y2',z2','color',[0 0 0]);hold on;
 
 view(3);
 
%  set(gca,'xlim',[-1 2],...
%   'ylim',[-1 2],...
%   'zlim',[-1 2]);
%  xlabel('XXX');
%  ylabel('YYY');
%  zlabel('ZZZ');
hold on;
plot3(X,Y,Z,'-*');xlabel('x');hold on;
plot3(X(1),Y(1),Z(1),'or','MarkerSize',10,'MarkerFaceColor','y','MarkerFaceColor','R');hold on;
ylabel('y');zlabel('z');title('Optimal Trajectory');hold on;
xlim([-10,10]);ylim([-10,10]);zlim([-10,10]);grid on;


