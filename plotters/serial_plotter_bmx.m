clear; clc;
 
%User Defined Properties 
serialPort = 'COM6';           % define COM port #
plotTitle = 'PocketCube-MAG';  % plot title
plotGrid = 'on';               % 'off' to turn off grid
min = -10000;                  % set y-min
max = 10000;                   % set y-max
scrollWidth = 10;              % display period in plot, plot entire data log if <= 0
delay = .01;                   % make sure sample faster than resolution
 
%Define Function Variables
time = 0;
data1 = 0;
data2 = 0;
data3 = 0;
data4 = 0;
count = 0;
% minXY corresponds to plus minus 4095, minZ corresponds to pm 16383
% Set up Plot
figure('units','normalized','outerposition',[0 0 1 1]);
plot1 = subplot(4,2,1);
plotGraph1 = plot(time,data1,'-r'); grid on;
ylabel('BH');
xlim([0 10]);% min max]);

plot2 = subplot(4,2,3);
plotGraph2 = plot(time,data2,'-b'); grid on;
ylabel('BV');
xlim([0 10]);% min max]);

plot3 = subplot(4,2,5);
plotGraph3 = plot(time,data3,'-g'); grid on;
ylabel('BZ');
xlim([0 10]); %min max]);

plot4 = subplot(4,2,7);
plotGraph4 = plot(time,data4,'-r'); grid on;
ylabel('B');
xlim([0 10]);% min max]);
% currentFigure = gcf;
% title(currentFigure.Children(end), 'PocketCube-MAG');
% figure(2);

plot5 = subplot(4,2,[2,4,6,8]);
plotGraph5 = plot(data1, data2); grid on; hold on;
plot([-35 35],[0 0], '-r'); 
plot([0 0], [-35 35],'-r');
xlabel('BH');
ylabel('BV');
set(plot5, 'XLim', [-35 35], 'YLim', [-35 35]);

%Open Serial COM Port
s = serial(serialPort, 'baudrate', 115200)
disp('Close Plot to End Session');
fopen(s);

tic
%%
while ishandle(plotGraph1) %Loop when Plot is Active 
    dat = fgets(s); %Read Data from Serial as Float
    C = strsplit(dat);
    temp1 = str2double(C(5)); 
    temp2 = str2double(C(7));
    temp3 = str2double(C(9));
    
    if(~isempty(dat))               % Make sure Data Type is Correct        
        count = count + 1;    
        time(count) = toc;          % Extract Elapsed Time
        data1(count) = temp1/16;    % Extract 1st Data Element         
        data2(count) = temp2/16;
        data3(count) = temp3/16;
        data4(count) = sqrt(data1(count)^2 + data2(count)^2 + data3(count)^2);
%         xbias = xbias + data1(count);
%         ybias = ybias + data2(count);
%         zbias = zbias + data3(count);
        
        % Set xlim according to Scroll Width
        if(scrollWidth > 0)
            set(plotGraph1, 'XData', time(time > time(count)-scrollWidth),'YData', data1(time > time(count)-scrollWidth));
            set(plot1, 'XLim', [time(count)-scrollWidth time(count)]);%, 'YLim', [min max]);
            
            set(plotGraph2, 'XData', time(time > time(count)-scrollWidth),'YData', data2(time > time(count)-scrollWidth));
            set(plot2, 'XLim', [time(count)-scrollWidth time(count)]);%, 'YLim', [min max]);
            
            set(plotGraph3, 'XData', time(time > time(count)-scrollWidth),'YData', data3(time > time(count)-scrollWidth));
            set(plot3, 'XLim', [time(count)-scrollWidth time(count)]);%, 'YLim', [min max]);
            
            set(plotGraph4, 'XData', time(time > time(count)-scrollWidth),'YData', data4(time > time(count)-scrollWidth));
            set(plot4, 'XLim', [time(count)-scrollWidth time(count)]);%, 'YLim', [min max]);

            set(plotGraph5, 'XData', data1(time > time(count)-scrollWidth),'YData', data2(time > time(count)-scrollWidth));
            set(plot5, 'XLim', [-35 35], 'YLim', [-35 35], 'DataAspectRatio', [1 1 1]);
        else
            set(plotGraph1,'XData',time,'YData',data1);
            xlim([0 time(count)]);% min max]);
            set(plotGraph2,'XData',time,'YData',data2);
            xlim([0 time(count)]);% min max]);
            set(plotGraph3,'XData',time,'YData',data3);
            xlim([0 time(count)]);% min max]);
            set(plotGraph4,'XData',time,'YData',data4);
            xlim([0 time(count)]);% min max]);
            
            set(plotGraph5,'XData',data1,'YData',data2);
            xlim([-35 35]);
            ylim([-35 35]);
        end
         
        %Allow MATLAB to Update Plot
        pause(delay);
    end
end
 
%% Close Serial COM Port and Delete useless Variables
fclose(s);
delete(s);
delete(instrfindall); % from random mathworks blog
clear count dat delay max min plotGraph plotGrid plotTitle s ...
        scrollWidth serialPort xLabel yLabel;  
disp('Session Terminated...');
