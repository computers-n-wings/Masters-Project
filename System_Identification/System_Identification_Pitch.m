%% System Identification Pitch: 3DOF Helicopter
% Author: James Gross
% Date: 09/06/2017

z = merge(s{1}, s{2}, s{3}, s{4}, s{5}, s{6}, s{7}, s{8}, s{9}, s{10}, s{11}, s{12});

InputName = {'Voltage applied to front motor'; ...
             'Voltage applied to back motor'};
InputUnit = {'V'; 'V'};

StateName = {'Elevation'; 'Pitch'; 'Travel'; 'Elevation Rate'; ...
    'Pitch Rate'; 'Travel Rate'};
StateUnit = {'rad'; 'rad'; 'rad'; 'rad/s'; 'rad/s'; 'rad/s'};
InitialStates = struct('Name', StateName, 'Unit', StateUnit, 'Value',  ...
                       {-27.5*pi/180; 0; 0; 0; 0; 0}, 'Minimum', -Inf, ...
                       'Maximum', Inf, 'Fixed', true);

ParName = {'Pitch Damping'; 'Pitch Static Friction'};
ParUnit = {'N*s/rad'; 'N'};
ParValue = {-0.01; -0.01};
ParMin = {-Inf; -Inf};
ParMax = {0; 0};
Parameters = struct('Name', ParName, 'Unit', ParUnit, 'Value', ParValue, ...
    'Minimum', ParMin, 'Maximum', ParMax, 'Fixed', false);

FileName = 'Pitch_Friction_Model'; % Name of Mex File
Order = [1 2 6]; % Order of system [ny nu nx]

nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, 0);

opt = nlgreyestOptions;
opt.Display = 'on';
opt.Regularization.Nominal = 'model';
opt.SearchOption.MaxIter = 50;

nlgr = nlgreyest(z, nlgr, opt)

%    -0.0000
%    -0.0016