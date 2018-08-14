clear; clc;
 
%User Defined Properties 
serialPort = 'COM3';            % define COM port #
plotTitle = 'PocketCube-MAG';  % plot title
plotGrid = 'on';                % 'off' to turn off grid
min = -20000;                     % set y-min
max = 20000;                      % set y-max
scrollWidth = 10;               % display period in plot, plot entire data log if <= 0
delay = .01;                    % make sure sample faster than resolution
 
%Define Function Variables
time = 0;
data1 = 0;
data2 = 0;
data3 = 0;
count = 0;
 
%Set up Plot
figure(1);
plot1 = subplot(3,1,1);
plotGraph1 = plot(time,data1,'-r'); grid on;
ylabel('BH');
axis([0 10 min max]);

plot2 = subplot(3,1,2);
plotGraph2 = plot(time,data2,'-b'); grid on;
ylabel('BV');
axis([0 10 min max]);

plot3 = subplot(3,1,3);
plotGraph3 = plot(time,data3,'-g'); grid on;
ylabel('BZ');
axis([0 10 min max]);

% currentFigure = gcf;
% title(currentFigure.Children(end), 'PocketCube-MAG');
             
%Open Serial COM Port
s = serial(serialPort, 'baudrate', 115200)
disp('Close Plot to End Session');
fopen(s);
 
tic
%%
while ishandle(plotGraph1) %Loop when Plot is Active 
    dat = fgets(s); %Read Data from Serial as Float
    C = strsplit(dat);
    temp1 = str2double(C(3)); 
    temp2 = str2double(C(5));
    temp3 = str2double(C(7));
    if(~isempty(dat)) %Make sure Data Type is Correct        
        count = count + 1;    
        time(count) = toc;    %Extract Elapsed Time
        data1(count) = temp1; %Extract 1st Data Element         
        data2(count) = temp2;
        data3(count) = temp3;
        
        %Set Axis according to Scroll Width
        if(scrollWidth > 0)
            set(plotGraph1, 'XData', time(time > time(count)-scrollWidth),'YData', data1(time > time(count)-scrollWidth));
            set(plot1, 'XLim', [time(count)-scrollWidth time(count)], 'YLim', [min max]);
            
            set(plotGraph2, 'XData', time(time > time(count)-scrollWidth),'YData', data2(time > time(count)-scrollWidth));
            set(plot2, 'XLim', [time(count)-scrollWidth time(count)], 'YLim', [min max]);
            
            set(plotGraph3, 'XData', time(time > time(count)-scrollWidth),'YData', data3(time > time(count)-scrollWidth));
            set(plot3, 'XLim', [time(count)-scrollWidth time(count)], 'YLim', [min max]);

        else
            set(plotGraph1,'XData',time,'YData',data1);
            axis([0 time(count) min max]);
            set(plotGraph2,'XData',time,'YData',data2);
            axis([0 time(count) min max]);
            set(plotGraph3,'XData',time,'YData',data3);
            axis([0 time(count) min max]);
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
