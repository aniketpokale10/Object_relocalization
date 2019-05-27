function [frame,Tmax] = visualizeTraj(varargin)

% parameters:
% arg(1). filename
% arg(2). T_ini
% arg(3). sparseICP_T
% arg(4). scale between the two point clouds

filename = varargin{1};
%to plot relocalized trajectory
if(nargin > 1)
   T_ini = varargin{2};
   T_ICP = varargin{3};
end

fileID = fopen(strcat(filename)); color=[1,0,0];
A = fscanf(fileID, '%f');
fclose(fileID);
A = A';
n = A(1);
c = zeros(size(A));
c(1) = 1;
c = logical(c);
A(c) = [];

R = cell(n,1);
T = cell(n,1);
frameList = [];

for i=1:size(A,2)/13
        frameList = [frameList;A( 13*(i-1)+1)];
        R{i} = A( 13*(i-1)+2 : 13*(i-1)+10 );
        R{i} = reshape(R{i},[3,3]);
        T{i} = A( 13*(i-1)+11 : 13*(i-1)+13 );
        T{i} = T{i}';
end


X = [];
axis('equal');
xlabel('x'); ylabel('y'); zlabel('z');

for i=1:size(A,2)/13
    X1 = [0;0;0];
    X1 = X1 + T{i};
    
%     if(nargin > 1)
%         X1 = T_ini(1:3,1:3)*X1 + T_ini(1:3,4);
%         X1 = T_ICP(1:3,1:3)*X1 + T_ICP(1:3,4); % for relocalization
%     end
    
    X = [X,X1];
end

if(nargin > 1)
    X = T_ini(1:3,1:3)*varargin{4}*X + T_ini(1:3,4);
    X = T_ICP(1:3,1:3)*X + T_ICP(1:3,4);
end

hold on; 
plot3(X(1,:),X(2,:),X(3,:),'Color', color);
xlabel('x'); ylabel('y'); zlabel('z');


Tvec=[];
for i=1:length(T)
    Tvec = [Tvec;norm(T{i})];
end

[val,ind] = max(Tvec);   

Tmax = T{ind};
frame = frameList(ind);