%% System Identification Elevation: 3DOF Helicopter
% Author: James Gross
% Date: 09/06/2017

clear all

s = IDdata_Elev1();
z = s{1};

% z = merge(s{1}, s{2}, s{3});

InputName = {'Voltage applied to front motor'; ...
             'Voltage applied to back motor'};
InputUnit = {'V'; 'V'};

StateName = {'Elevation'; 'Pitch'; 'Travel'; 'Elevation Rate'; ...
    'Pitch Rate'; 'Travel Rate'; 'Delayed Front Voltage'; 'Delayed Back Voltage'};
StateUnit = {'rad'; 'rad'; 'rad'; 'rad/s'; 'rad/s'; 'rad/s'; 'V'; 'V'};
InitialStates = struct('Name', StateName, 'Unit', StateUnit, 'Value',  ...
                       {-27.5*pi/180; 0; 0; 0; 0; 0; 0; 0},'Minimum', -Inf, ...
                       'Maximum', Inf, 'Fixed', true);

ParName = {'Elevation Damping'; 'Elevation Static Friction'; 'Mass of Counterweight'; ...
    'Force Constant'; 'Length of arm'; 'Length of helicopter'; 'Distance of weight'; 'delay'};
ParUnit = {'N*s/rad'; 'N'; 'kg'; 'N/V'; 'm'; 'm'; 'm'; 'V/s'};
ParValue = {-1; -1.5; 1.924; 0.1188; 26.0 * 0.0254; 7.0 * 0.0254; 18.5 * 0.0254; -1.2615};
ParMin = {-2; -2.5; 1.5; 0.1; 20.0 * 0.0254; 5.0 * 0.0254; 15.0 * 0.0254; -2};
ParMax = {0; 0; 2.5; 1; 30.0 * 0.0254; 10.0 * 0.0254; 25.0 * 0.0254; -1};
Parameters = struct('Name', ParName, 'Unit', ParUnit, 'Value', ParValue, ...
    'Minimum', ParMin, 'Maximum', ParMax, 'Fixed', false);

FileName = 'Elevation_Friction_Model'; % Name of Mex File
Order = [1 2 8]; % Order of system [ny nu nx]

nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, 0);
% nlgr.SimulationOptions.Solver = 'ode1';

opt = nlgreyestOptions;
opt.Display = 'on';
% opt.Regularization.Nominal = 'model';
opt.SearchOption.MaxIter = 50;

nlgr = nlgreyest(z, nlgr, opt);

compare(z,nlgr);
