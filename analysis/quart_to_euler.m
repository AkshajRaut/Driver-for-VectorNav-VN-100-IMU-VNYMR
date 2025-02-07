inputFileName = 'Book1.csv';  
outputFileName = 'quaternions_with_euler.csv'; 
quaternions = readmatrix(inputFileName);
eulerAnglesRad = quat2eul(quaternions, 'ZYX');
eulerAnglesDeg = rad2deg(eulerAnglesRad); 
outputData = [quaternions, eulerAnglesDeg];
outputDataStr = arrayfun(@(x) sprintf('%.16f', x), outputData, 'UniformOutput', false);
eulerTable = array2table(outputDataStr, 'VariableNames', {'q1', 'q2', 'q3', 'q4', 'Yaw', 'Pitch', 'Roll'});
writetable(eulerTable, outputFileName, 'WriteVariableNames', true, 'Delimiter', ',');
format long g; 
disp('Conversion complete. Output saved to:');
disp(outputFileName);
