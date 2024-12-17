clear all
close all

filename="results_2x2.csv"

steps = 5:5:50;

% Specify the file type explicitly
opts = detectImportOptions(filename);
opts.VariableNamesLine = 1;

% Read the table
T2 = readtable(filename, opts);


filename="results_3x3.csv"

% Specify the file type explicitly
opts = detectImportOptions(filename);
opts.VariableNamesLine = 1;

% Read the table
T3 = readtable(filename, opts);


filename="results_4x4.csv"

% Specify the file type explicitly
opts = detectImportOptions(filename);
opts.VariableNamesLine = 1;

% Read the table
T4 = readtable(filename, opts);


filename="results_5x5.csv"

% Specify the file type explicitly
opts = detectImportOptions(filename);
opts.VariableNamesLine = 1;

% Read the table
T5 = readtable(filename, opts);



% Step 1: Extract all numerical data from the table
data = T2{:, :};  % Access the data in the table (all rows, all columns)

% Step 2: Replace all 0 values with NaN
data(data == 0) = NaN;  % Find all 0s and replace them with NaN

% Step 3: Replace the table with the modified data
T2{:, :} = data;
% Get the mean values of the columns
columnMeans2 = mean(T2{2:end, :}, 'omitnan')

% Step 1: Extract all numerical data from the table
data = T3{:, :};  % Access the data in the table (all rows, all columns)

% Step 2: Replace all 0 values with NaN
data(data == 0) = NaN;  % Find all 0s and replace them with NaN

% Step 3: Replace the table with the modified data
T3{:, :} = data;
% Get the mean values of the columns
columnMeans3 = mean(T3{2:end, :}, 'omitnan')



% Step 1: Extract all numerical data from the table
data = T4{:, :};  % Access the data in the table (all rows, all columns)

% Step 2: Replace all 0 values with NaN
data(data == 0) = NaN;  % Find all 0s and replace them with NaN

% Step 3: Replace the table with the modified data
T4{:, :} = data;
% Get the mean values of the columns
columnMeans4 = mean(T4{2:end, :}, 'omitnan')



% Step 1: Extract all numerical data from the table
data = T5{:, :};  % Access the data in the table (all rows, all columns)

% Step 2: Replace all 0 values with NaN
data(data == 0) = NaN;  % Find all 0s and replace them with NaN

% Step 3: Replace the table with the modified data
T5{:, :} = data;
% Get the mean values of the columns
columnMeans5 = mean(T5{2:end, :}, 'omitnan')


cM2 = columnMeans2';
cM3 = columnMeans3';
cM4 = columnMeans4';
cM5 = columnMeans5';
% Create a new figure for the combined plot
figure;

% Define custom colors
colorNoHARQ = [0, 0.6, 0.8]; % A shade of blue for Recommended No HARQ
colorNoHARQMax = [0, 0.3, 0.5]; % A slightly darker shade of blue for Max No HARQ
colorWithHARQ = [0.85, 0.2, 0.35]; % A shade of red for Recommended With HARQ
colorWithHARQMax = [0.55, 0.00, 0.05]; % A darker shade of red for Max With HARQ


% Plot the mean values with line plots

fill([5 50 50 5],[0.01 0.01 1 1],'green','FaceAlpha',0.1, 'EdgeColor','none')
hold on;grid on;
plot(steps, cM2, 'LineWidth', 2, 'Color', colorNoHARQMax); % No HARQ Max

plot(steps, cM3, 'LineWidth', 2, 'Color', colorNoHARQ); % No HARQ
plot(steps, cM4, 'LineWidth', 2, 'Color', colorWithHARQMax); % With HARQ Max
plot(steps, cM5, 'LineWidth', 2, 'Color', colorWithHARQ); % With HARQ
% After plotting your data and before saving the figure
length(cM2)
set(gca, 'YScale', 'log');



% Set common properties for the plot
ylim([0.1 100]);

xlabel('Step Size Between Devices (m)','FontName','Century');
ylabel('Log Average Latency (ms)','FontName','Century');
legend({'URLLC Requirement','2x2','3x3', '4x4','5x5'}, 'Location', 'best');
title('Latency variation based on the mesh size and device distance')



% Optionally, save the plot to a file
saveas(gca, 'data_rate_on_mcs_change.png');
hold off;