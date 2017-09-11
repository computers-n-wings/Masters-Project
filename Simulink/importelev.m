function [VarName3,VarName4] = importelev(filename, startRow, endRow)
%IMPORTFILE4 Import numeric data from a text file as column vectors.
%   [VARNAME3,VARNAME4] = IMPORTFILE4(FILENAME) Reads data from text file
%   FILENAME for the default selection.
%
%   [VARNAME3,VARNAME4] = IMPORTFILE4(FILENAME, STARTROW, ENDROW) Reads
%   data from rows STARTROW through ENDROW of text file FILENAME.
%
% Example:
%   [VarName3,VarName4] = importfile4('elevation1a.txt',4, 4004);
%
%    See also TEXTSCAN.

% Auto-generated by MATLAB on 2017/08/13 13:02:56

%% Initialize variables.
delimiter = '\t';
if nargin<=2
    startRow = 4;
    endRow = 4004;
end

%% Format string for each line of text:
%   column3: double (%f)
%	column4: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%*s%*s%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, endRow(1)-startRow(1)+1, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines', startRow(1)-1, 'ReturnOnError', false);
for block=2:length(startRow)
    frewind(fileID);
    dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines', startRow(block)-1, 'ReturnOnError', false);
    for col=1:length(dataArray)
        dataArray{col} = [dataArray{col};dataArrayBlock{col}];
    end
end

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
VarName3 = dataArray{:, 1};
VarName4 = dataArray{:, 2};


