function plotGroundTruthTraj(filename,frame,Tmax)


fileID = fopen(strcat(filename)); color=[0,0.5,0];
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

for i=1:size(A,2)/13
        R{i} = A( 13*(i-1)+2 : 13*(i-1)+10 );
        R{i} = reshape(R{i},[3,3]);
        T{i} = A( 13*(i-1)+11 : 13*(i-1)+13 );
        T{i} = T{i}';
end

T1 = T{i};
for i=1:length(T)
    T{i} = T{i} - T1;
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

Tnormvec=[]; Tvec = [];
for i=1:length(T)
    Tnormvec = [Tnormvec;norm(T{i})];
    Tvec = [Tvec,T{i}];
end

[val,ind] = max(Tnormvec);
GTmax = T{ind};

% r = vrrotvec(GTmax,Tmax);


%%
% m = vrrotvec2mat(r)

% for i=1:length(T)
%     T{i} = m*T{i};
% end
%%

% Trotated = rodrigues_rot(Tvec,r(1:3)',r(4));

%%
% rotate again to align
% Taxis = T{ind};
% Trotated = rodrigues_rot(Trotated,Taxis(1:3),pi/2);

Trotated = rodrigues_rot(Tvec,[1,0,0]',pi/2);
scale = norm(Tmax)/norm(Trotated(:,frame));
Trotated = Trotated.*scale;
r = vrrotvec(Trotated(:,frame),Tmax);
Trotated = rodrigues_rot(Trotated,r(1:3)',r(4));

hold on; 
plot3(Trotated(1,:),Trotated(2,:),Trotated(3,:),'Color', color);
xlabel('x'); ylabel('y'); zlabel('z');