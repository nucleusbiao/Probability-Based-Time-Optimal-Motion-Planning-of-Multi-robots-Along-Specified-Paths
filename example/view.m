clear
clc
close all
%%
robotNum=12;
pathLength=70;
data=cell(robotNum,1);
t=cell(robotNum,1);
x=cell(robotNum,1);
y=cell(robotNum,1);
for i=1:1:robotNum
    fileName = ['robot' num2str(i-1) '.txt'];
    data{i}=load(fileName);
    t{i}=data{i}(:,2);
    if i<=robotNum/2
        x{i}=data{i}(:,1);
        y{i}=ones(size(t{i}))*pathLength/(robotNum/2+1)*i;
    else
        y{i}=pathLength-data{i}(:,1);
        x{i}=ones(size(t{i}))*pathLength/(robotNum/2+1)*(i-robotNum/2);
    end
    
end
% for i=1:1:robotNum
%     fileName = ['robot' num2str(i-1) '.txt'];
%     data{i}=load(fileName);
%     t{i}=data{i}(:,2);
%     if i<=(robotNum-2)/2
%         x{i}=data{i}(:,1);
%         y{i}=ones(size(t{i}))*pathLength/((robotNum-2)/2+1)*i;
%     elseif i>(robotNum-2)/2&&i<=(robotNum-2)
%         y{i}=pathLength-data{i}(:,1);
%         x{i}=ones(size(t{i}))*pathLength/((robotNum-2)/2+1)*(i-(robotNum-2)/2);
%     end
% end
% x{robotNum-1}=data{robotNum-1}(:,1)/sqrt(2);
% y{robotNum-1}=pathLength-(data{robotNum-1}(:,1))/sqrt(2);
% x{robotNum}=pathLength-(data{robotNum}(:,1))/sqrt(2);
% y{robotNum}=pathLength-(data{robotNum}(:,1))/sqrt(2);
% axis([-5,pathLength+5,-5,pathLength+5]);
% box on;

% for i=1:1:robotNum
%     fileName = ['robot' num2str(i-1) '.txt'];
%     data{i}=load(fileName);
%     t{i}=data{i}(:,2);
%     if i<=2+(robotNum-2)/2&&i>2
%         x{i}=data{i}(:,1);
%         y{i}=ones(size(t{i}))*pathLength/((robotNum-2)/2+1)*(i-2);
%     elseif i>2+(robotNum-2)/2
%         y{i}=pathLength-data{i}(:,1);
%         x{i}=ones(size(t{i}))*pathLength/((robotNum-2)/2+1)*(i-2-(robotNum-2)/2);
%     end
% end
% x{1}=data{1}(:,1)/sqrt(2);
% y{1}=pathLength-(data{1}(:,1))/sqrt(2);
% x{2}=pathLength-(data{2}(:,1))/sqrt(2);
% y{2}=pathLength-(data{2}(:,1))/sqrt(2);

axis([-5,pathLength+5,-5,pathLength+5]);
box on;
%%
maxT=0;
for i=1:1:robotNum
    if(maxT<size(t{i},1))
        maxT=size(t{i},1);
    end
end
for i=1:1:maxT
    cla;
    hold on;
    for j=1:1:robotNum
        plot(x{j},y{j});
        if(i<=size(t{j},1))
           plot(x{j}(i),y{j}(i),'o');
        end
    end
    frame=getframe(gcf);
    imind=frame2im(frame);
    [imind,cm] = rgb2ind(imind,256);
    if i==1
        imwrite(imind,cm,'test.gif','gif', 'Loopcount',inf,'DelayTime',1e-4);
    else
        imwrite(imind,cm,'test.gif','gif','WriteMode','append','DelayTime',1e-4);
    end
end



