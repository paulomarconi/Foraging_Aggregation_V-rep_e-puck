%% Scenario 1 - Foraging
clear variables; close all; clc;

font=14; 
% plot data for n rounds, n trials
rounds=4; trials=10;
for j=1:rounds
    % obtain number of collected blobs
    for i=1:trials
        t=importdata(['Data1_round',num2str(j),'_',num2str(i),'.txt']);   
        max_val(i)=max(t(:,2));
    end

    m=mean(max_val);
    s=std(max_val);
    
    % plot the mean and SD
    fig=errorbar(j,m,s,'o','Color','r','MarkerSize',7,'LineWidth',1);
    hold on;
    
    %plot values of mean and SD of each round
    text(j-.2,8.7,['\mu=',num2str(m)],'Fontsize',font-2);
    text(j-.2,8,['\sigma=',num2str(s,2)],'Fontsize',font-2);
    
    
    % plot the collected blobs
    b=bone(10); % 'bone' colormap
    for i=1:trials
        plot(j,max_val(i),'-o','MarkerFaceColor',b(i,:),'MarkerEdgeColor','b','MarkerSize',6);
        hold on;
    end
    hold on;
end

set(gca,'FontSize',font) % change the font size of the axis only
axis([0 5,7.5 20]);%,'auto y');
legend('$\mu~\pm~\sigma$','Trials','Fontsize',font-2,'Location','northeast','Interpreter','Latex');
xlabel('Simulation Round (10 Trials/Round)','Fontsize',font); 
ylabel('Total collected Blobs/Trial','Fontsize',font);
% title('Scenario 1 (1 robot)','Fontsize',font); 
grid on;
saveas(fig,'C:/Users/zurits/GoogleDrive_ieee/Sheffield\ACS6121 Robotics and Autonomous Systems/assessment/v-rep lab/report/figures/Robot1.png')

%% Scenario 2 - Foraging
clear variables; close all; clc;

font=14;
% plot data for n rounds, n trials
rounds=4; trials=10;
for j=1:rounds
    % obtain number of collected blobs
    for i=1:trials
        t=importdata(['Data5_round',num2str(j),'_',num2str(i),'.txt']);   
        max_val(i)=max(t(:,2));
    end

    m=mean(max_val);
    s=std(max_val);
    
    % plot the mean and SD
    fig=errorbar(j,m,s,'o','Color','r','MarkerSize',7,'LineWidth',1);
    hold on;
    
    % plot values of mean and SD of each round
    text(j-.2,31.5,['\mu=',num2str(m)],'Fontsize',font-2);
    text(j-.2,30.7,['\sigma=',num2str(s,2)],'Fontsize',font-2);
    
    % plot the collected blobs
    b=bone(trials); % 'bone' colormap
    for i=1:trials
        plot(j,max_val(i),'-o','MarkerFaceColor',b(i,:),'MarkerEdgeColor','b','MarkerSize',6);
        hold on;
    end
    hold on;
end

set(gca,'FontSize',font) % change the font size of the axis only
axis([0 5,30 44]);%,'auto y');
legend('$\mu~\pm~\sigma$','Trials','Fontsize',font-2,'Location','northeast','Interpreter','Latex');
xlabel('Simulation Round (10 Trials/Round)','Fontsize',font); 
ylabel('Total collected Blobs/Trial','Fontsize',font);
% title('Scenario 2 (5 robots)','Fontsize',font); 
grid on;
saveas(fig,'C:/Users/zurits/GoogleDrive_ieee/Sheffield\ACS6121 Robotics and Autonomous Systems/assessment/v-rep lab/report/figures/Robot5.png')


%% Agregation
clear variables; close all; clc;

font=14;
% 4 Rounds
R1=[4,2,3,3,2,3,2,2,0,3];
R2=[4,3,4,3,4,4,2,3,5,4];
R3=[5,3,2,3,2,3,3,3,4,3];
R4=[4,3,5,3,4,0,0,4,2,3];

R=[R1',R2',R3',R4'];

rounds=4; trials=10;

for i=1:rounds
    m(i)=mean(R(:,i));
    s(i)=std(R(:,i));
    
    % plot the mean and SD
    fig=errorbar(i,m(i),s(i),'o','Color','r','MarkerSize',7,'LineWidth',1);
    hold on;   
    
    % plot values of mean and SD of each round
    text(i-.2,-1.5,['\mu=',num2str(m(i))],'Fontsize',font-2);
    text(i-.2,-2,['\sigma=',num2str(s(i),2)],'Fontsize',font-2);
    
    % plot the robots that formed at least a group of 2
    b=bone(6); % 'bone' colormap
    plot(i,R(:,i),'-o','MarkerFaceColor',b(i,:),'MarkerEdgeColor','b','MarkerSize',6);
    hold on;
end

set(gca,'FontSize',font) % change the font size of the axis only
axis([0 5,-3 8]);%,'auto y');
legend('$\mu~\pm~\sigma$','Trials','Fontsize',font-2,'Location','northeast','Interpreter','Latex');
xlabel('Simulation Round (10 Trials/Round)','Fontsize',font); 
ylabel('Aggregated Robots','Fontsize',font);
% title('Scenario 2 (5 robots)','Fontsize',font); 
grid on;
saveas(fig,'C:/Users/zurits/GoogleDrive_ieee/Sheffield\ACS6121 Robotics and Autonomous Systems/assessment/v-rep lab/report/figures/Robot5_agregation.png')



















