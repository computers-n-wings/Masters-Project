function options = settings

%SETTINGS - General and solver-specific settings are selected here
% Unless specified otherwise the options are set using 0 => no and 1 => yes
%
% Syntax:  options = Settings
%
% Output:
%    options - Structure containing the settings
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% Copyright (C) 2010 Paola Falugi, Eric Kerrigan and Eugene van Wyk. All Rights Reserved.
% This code is published under the BSD License.
% Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) 5 May 2010
% iclocs@imperial.ac.uk

%------------- BEGIN CODE --------------


% Transcription Method:
%---------------------------------------
% Discrete-time model       ('discrete')
% Multiple shooting method  ('multiple_shooting') WARNING: The
%                                                 'quasi-newton' option for the hessian 
%                                                   computation has to be selected
%                                                 (options.ipopt.hessian_approximation='limited-memory').
% Euler method              ('euler')
% Trapezoidal method        ('trapezoidal')
% Hermite-Simpson method    ('hermite')

options.transcription='hermite';

% Derivative generation :
%---------------------------------------
% Whenever the analytic differentiation is enabled it is necessary to
% specify the available analytic forms for the cost function, the dynamic equations 
% and  the constraints in the appropriate files .m

% Numerical differentiation: finite differences  ('numeric')
% Analytic differentiation: analytic gradients   ('analytic')

options.derivatives='numeric';

% Numeric generation of the Hessian:
%----------------------------------------------------------------

% Whenever the numeric differentiation is enabled it is necessary to
% specify which kind of finite difference approximation to use  between 
% the following ones:
% 
% Central difference ('central')  
% forward difference ('forward')

options.hessianFD='central';

%  The perturbation size for numerical second derivatives 
%  can be set in options.perturbation.H. The perturbation size for numerical first derivatives 
%  can be set in  options.perturbation.J. 
%  It is possible to select default values for the perturbations by setting  options.perturbation.H and 
%  options.perturbation.J to the empty matrix.
%  The default values for the gradient approximation is (eps/2)^(1/3)
%  while for the  second derivative is (8*eps)^(1/3). 

options.perturbation.H=[];  % Perturbation size for the second derivatives
options.perturbation.J=[];  % Perturbation size for the first derivatives

% NLP solver
%---------------------------------------
% IPOPT: recommended but needs ipopt.mex        ('ipopt')
% fmincon                                       ('fmincon')

options.NLPsolver='ipopt';

% IPOPT settings (if required)
options.ipopt.tol=1e-8;                        % Desired convergence tolerance (relative). The default value is  1e-8. 
options.ipopt.print_level=5;                   % Print level. The valid range for this integer option is [0,12] and its default value is 5.
options.ipopt.max_iter=5000;                   % Maximum number of iterations. The default value is 3000.
 
options.ipopt.mu_strategy ='monotone';         % Determines which barrier parameter update strategy is to be used. 
					       % The default value for this string option is "monotone".
                                               % Possible values:
                                               %   'monotone': use the monotone (Fiacco-McCormick) strategy
                                               %   'adaptive': use the adaptive update strategy

options.ipopt.hessian_approximation='limited-memory';   %  Indicates what information for the Hessian of the Lagrangian function is                                                    
                                               %  used by the algorithm. The default value is 'exact'.
                                               %  Possible values:
                                               %   'exact': Use second derivatives provided by ICLOCS.
                                               %   'limited-memory': Perform a limited-memory quasi-Newton approximation
					       %		     implemented inside IPOPT

%options.ipopt.limited_memory_max_history=6;   % Maximum size of the history for the limited quasi-Newton Hessian approximation. The valid range for this integer option is [0, +inf) 
                                               % and its default value is 6. 
%options.ipopt.limited_memory_max_skipping=1;  % Threshold for successive iterations where update is skipped for the quasi-Newton approximation.
                                               % The valid range for this integer option is [1,+inf) and its default value is 2. 

                                               
                            

% fmincon settings (if required)
%options.fmincon=optimset;
options.fmincon=optimset; 

% Automatic scaling (recommended)
%---------------------------------------
options.scaling=1;


% Output settings
%---------------------------------------

% Display computation time
options.print.time=1;

% Display relative local discretization error (recommended for direct transcription)
options.print.relative_local_error=1;


% Display cost
options.print.cost=1;


% Plot states
options.plot.states=1;

% Plot inputs
options.plot.inputs=1;

% Plot Lagrange multipliers
options.plot.multipliers=1;


% Direct transcription settings
%---------------------------------------

% Number of   integration nodes in the interval t=[0,tf]; nodes=steps+1.
% The quantity steps/N (N number of control actions) must be a positive
% integer.

options.nodes=51; 

% Distribution of integration steps. Set tau=0 for equispaced steps.
% Otherwise: tau is a vector of length M-1 with 0<tau(i)<1 and sum(tau)=1.
% For discrete time system  set  tau=0.

options.tau=0;


% Multiple shooting settings
%---------------------------------------
% N/S=normal/stiff. H/M/L=high/medium/low accuracy
%
% 'cvodes'   N/S    H    High accuracy, difficult problems(slow)
%
%
%  Note: 'cvodes' requires the sundialsTB. 
%        

options.ODEsolver='cvodes';

% CVODES settings (if required)

Method='Adams';  % Method: Adams, BDF
Solver='Newton'; % Solver: Newton, Functional (requires dfdx)


% Options for forward integration when cvodes is enabled

%  options.cvodes = CVodeSetOptions('RelTol',1.e-4,...
%                            'AbsTol',1.e-6,...
%                            'LinearSolver','Dense',...
%                            'MaxNumSteps',10000,...
%                            'LMM',Method,...
%                            'NonlinearSolver',Solver); 
                 

% Forward sensitivity options when cvodes is enabled 

% options.cvodesf=CVodeSensSetOptions('ErrControl',true,'method','Staggered'); % FSA initialization
















