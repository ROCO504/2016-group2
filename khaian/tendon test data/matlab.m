%% Initialize variables.
clear all
close all
clc

filename = 'I:\504\tendon tests\raw data.csv';
delimiter = ',';
startRow = 3;

%% Format string for each line of text:
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Create output variable
rawdata = [dataArray{1:end-1}];
%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;
%% scale extension to % of original length
origLength=[156,84,79,92,123,56,14.5,15,80];
scaledData=rawdata;
for i=0:8 %scale data for 100mm sample
scaledData(:,1+(i*2))=(rawdata(:,1+(i*2))/origLength(i+1))*100;
end

%% plot
close all
hold on
title('100mm tendon force/extension')
xlabel('extension(MM)')
ylabel('force(N)')

for i=0:8
plot(scaledData(:,1+(i*2)),scaledData(:,2+(i*2)))
end
legend('semiflex','fabric elastic','nylon/ninjaflex','ninjaflex','nylon','double scp','pp scp','pp','scp');
%test 2/3 merged elastic string (pre and post sheath snap)
%-ve values removed
%large slip in ninjaflex removed

