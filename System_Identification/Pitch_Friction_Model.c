/*   Copyright 2005-2006 The MathWorks, Inc. */

/* Template file for IDNLGREY model specification.
   
   Use this file to create a MEX function that specifies the model
   structure and equations. The MEX file syntax is
      [dx, y] = mymodel(t, x, u, p1, p2, ..., pn, auxvar)
   where
      * t is the time (scalar).
      * x is the state vector at time t (column vector).
      * u is the vector of inputs at time t (column vector).
      * p1, p2,... pn: values of the estimated parameters specified
        in the IDNLGREY model.
      * auxvar: a cell array containing auxiliary data in any format
        (optional).
      * dx is the vector of state derivatives at time t (column vector).
      * y is the vector of outputs at time t.
   
   To create the MEX file "mymodel", do the following:
      1) Save this template as "mymodel.c" (replace "mymodel" by the
         name of your choice).
      2) Define the number NY of outputs below.
      3) Specify the state derivative equations in COMPUTE_DX below.
      4) Specify the output equations in COMPUTE_Y below.
      5) Build the MEX file using
            >> mex mymodel.c
*/

/* Include libraries. */
#include "mex.h"
#include "math.h"

/* Specify the number of outputs here. */
#define NY 1

void compute_dx(
    double *dx,  /* Vector of state derivatives (length nx). */
    double t,    /* Time t (scalar). */
    double *x,   /* State vector (length nx). */
    double *u,   /* Input vector (length nu). */
    double **p,  /* p[j] points to the j-th estimated model parameters (a double array). */
    const mxArray *auxvar  /* Cell array of additional data. */
   )
{
    /*
      Define the state equation dx = f(t, x, u, p[0],..., p[np-1], auvar)
      in the body of this function.
    */
    /*
      Accessing the contents of auxvar:
      
      Use mxGetCell to fetch pointers to individual cell elements, e.g.:
          mxArray* auxvar1 = mxGetCell(auxvar, 0);
      extracts the first cell element. If this element contains double
      data, you may obtain a pointer to the double array using mxGetPr:
          double *auxData = mxGetPr(auxvar1);
      
      See MATLAB documentation on External Interfaces for more information
      about functions that manipulate mxArrays.
    */
    
    /* Example code from ODE function for DCMOTOR example
       used in idnlgreydemo1 (dcmotor_c.c) follows.
    */
    
    double *F_v2, *F_c2; /* Estimated model parameters. */
    const double mh = 1.2883;
    const double mw = 1.9430;
    const double Kf = 0.0167;
    const double g = 9.81;
    const double La = 26.0 * 0.0254;
    const double Lh = 7.0 * 0.0254;
    const double Lw = 18.5 * 0.0254;
    const double mf = 0.5*mh;
    const double mb = 0.5*mh;

    F_v2 = p[0];
    F_c2 = p[1];

    dx[0] = x[3]; /* Elevation Rate. */
    dx[1] = x[4]; /* Pitch Rate. */
    dx[2] = x[5]; /* Travel Rate. */

	/* Pitch Friction Model */
  dx[3] = (Lh * (0.8e1 * La * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * x[3] * x[5] * Lh * (mb - mf) * pow(cos(x[0]), 0.2e1) + ((g * mb + g * mf + Kf * (u[0] + u[1])) * La - Lw * mw * g) * (pow(mf + mb, 0.2e1) * Lh * Lh + La * La * pow(mb - mf, 0.2e1))) * pow(cos(x[1]), 0.3e1) + (-0.4e1 * La * sin(x[1]) * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * pow(x[5], 0.2e1) * Lh * (mb - mf) * pow(cos(x[0]), 0.2e1) + (-0.8e1 * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * (mf + mb) * pow(x[5], 0.2e1) * Lh * Lh * sin(x[0]) - 0.4e1 * (mf + mb) * (0.2e1 * x[4] * x[5] * La * La * mb * mf + g * La * mb * mf - Lw * mw * (mf + mb) * (-0.2e1 * Lw * x[4] * x[5] + g) / 0.4e1) * Lh * Lh + g * La * Lw * mw * pow(mb - mf, 0.2e1) * (La + Lw)) * cos(x[0]) + 0.4e1 * La * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * pow(x[3], 0.2e1) * Lh * (mb - mf) * sin(x[1]) + ((g * mb + g * mf + Kf * (u[0] + u[1])) * La - Lw * mw * g) * (pow(mf + mb, 0.2e1) * Lh * Lh + La * La * pow(mb - mf, 0.2e1))) * Lh * cos(x[0]) * pow(cos(x[1]), 0.2e1) + (-0.8e1 * La * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * x[3] * x[5] * Lh * Lh * (mb - mf) * pow(cos(x[0]), 0.2e1) + (-0.8e1 * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * x[3] * (mf + mb) * pow(Lh, 0.3e1) * (x[5] * sin(x[0]) + x[4]) * sin(x[1]) + (Lh * Lh * (mf + mb) + (mf + mb) * La * La + Lw * Lw * mw) * La * (Kf * (u[0] - u[1]) * Lh + F_v2[0] * x[4] + (double) (F_c2[0] * (x[1]>0 ? 1 : (x[1]<0 ? -1 : 0)))) * (mb - mf)) * cos(x[0]) - (-(Kf * (u[0] - u[1]) * Lh + F_v2[0] * x[4] + (double) (F_c2[0] * (x[1]>0 ? 1 : (x[1]<0 ? -1 : 0)))) * sin(x[0]) * sin(x[1]) + (g * mb + g * mf + Kf * (u[0] + u[1])) * La - Lw * mw * g) * Lh * (pow(mf + mb, 0.2e1) * Lh * Lh + La * La * pow(mb - mf, 0.2e1))) * cos(x[1]) - 0.4e1 * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * Lh * cos(x[0]) * (-0.2e1 * pow(x[5], 0.2e1) * sin(x[1]) * La * Lh * (mb - mf) * pow(cos(x[0]), 0.2e1) + (((-mb - mf) * Lh * Lh + (mf + mb) * La * La + Lw * Lw * mw) * pow(x[5], 0.2e1) * sin(x[0]) - 0.2e1 * x[5] * x[4] * (mf + mb) * Lh * Lh + ((mf + mb) * La - Lw * mw) * g) * cos(x[0]) + Lh * ((0.2e1 * x[4] * x[5] * La + g) * sin(x[0]) + La * (pow(x[4], 0.2e1) + pow(x[5], 0.2e1))) * (mb - mf) * sin(x[1]) + (-g * mb - g * mf - Kf * (u[0] + u[1])) * La + Lw * mw * g)) / (Lh * Lh * (mf + mb) + (mf + mb) * La * La + Lw * Lw * mw) / (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) / Lh / cos(x[0]) / 0.4e1;
  dx[4] = (-0.4e1 * cos(x[1]) * ((-mb - mf) * Lh * Lh + (mf + mb) * La * La + Lw * Lw * mw) * sin(x[1]) * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * pow(x[5], 0.2e1) * Lh * Lh * pow(cos(x[0]), 0.4e1) + cos(x[1]) * (0.4e1 * La * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * pow(x[5], 0.2e1) * sin(x[0]) * Lh * Lh * (mb - mf) * pow(cos(x[1]), 0.2e1) + 0.8e1 * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * x[3] * x[5] * Lh * ((mf + mb) * La * La + Lw * Lw * mw) * cos(x[1]) + (-0.8e1 * La * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * pow(x[5], 0.2e1) * Lh * Lh * sin(x[0]) - 0.4e1 * La * (0.2e1 * x[4] * x[5] * La * La * mb * mf + g * La * mb * mf - Lw * mw * (mf + mb) * (-0.2e1 * Lw * x[4] * x[5] + g) / 0.4e1) * Lh * Lh + Lw * mw * g * ((mf + mb) * La * La + Lw * Lw * mw) * (La + Lw)) * (mb - mf)) * Lh * pow(cos(x[0]), 0.3e1) + (Lh * Lh * (0.8e1 * La * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * x[3] * x[5] * sin(x[0]) * Lh * (mb - mf) * sin(x[1]) + (Kf * (u[0] - u[1]) * Lh + F_v2[0] * x[4] + (double) (F_c2[0] * (x[1]>0 ? 1 : (x[1]<0 ? -1 : 0)))) * (pow(mf + mb, 0.2e1) * Lh * Lh + La * La * pow(mb - mf, 0.2e1))) * pow(cos(x[1]), 0.2e1) + Lh * (((-0.4e1 * (mf + mb) * (0.2e1 * x[4] * x[5] * La * La * mb * mf + g * La * mb * mf - Lw * mw * (mf + mb) * (-0.2e1 * Lw * x[4] * x[5] + g) / 0.4e1) * Lh * Lh + g * La * Lw * mw * pow(mb - mf, 0.2e1) * (La + Lw)) * sin(x[0]) + 0.4e1 * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * ((pow(x[3], 0.2e1) - 0.2e1 * pow(x[5], 0.2e1)) * (mf + mb) * Lh * Lh + pow(x[3], 0.2e1) * ((mf + mb) * La * La + Lw * Lw * mw))) * Lh * sin(x[1]) + (Lh * Lh * (mf + mb) + (mf + mb) * La * La + Lw * Lw * mw) * La * ((g * mb + g * mf + Kf * (u[0] + u[1])) * La - Lw * mw * g) * (mb - mf)) * cos(x[1]) + (Lh * Lh * (mf + mb) + (mf + mb) * La * La + Lw * Lw * mw) * ((-mb - mf) * Lh * Lh + (mf + mb) * La * La + Lw * Lw * mw) * (Kf * (u[0] - u[1]) * Lh + F_v2[0] * x[4] + (double) (F_c2[0] * (x[1]>0 ? 1 : (x[1]<0 ? -1 : 0))))) * pow(cos(x[0]), 0.2e1) + Lh * (-0.4e1 * La * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * pow(x[3], 0.2e1) * sin(x[0]) * Lh * Lh * (mb - mf) * pow(cos(x[1]), 0.3e1) + (0.8e1 * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * x[3] * (mf + mb) * pow(Lh, 0.3e1) * x[4] * sin(x[0]) + 0.8e1 * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * x[3] * (mf + mb) * x[5] * pow(Lh, 0.3e1) + La * ((g * mb + g * mf + Kf * (u[0] + u[1])) * La - Lw * mw * g) * (mf + mb) * (mb - mf) * Lh * Lh + La * ((g * mb + g * mf + Kf * (u[0] + u[1])) * La - Lw * mw * g) * ((mf + mb) * La * La + Lw * Lw * mw) * (mb - mf)) * pow(cos(x[1]), 0.2e1) + (((g * mb + g * mf + Kf * (u[0] + u[1])) * La - Lw * mw * g) * sin(x[0]) * (pow(mf + mb, 0.2e1) * Lh * Lh + La * La * pow(mb - mf, 0.2e1)) * sin(x[1]) + 0.4e1 * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * (La * (pow(x[4], 0.2e1) + pow(x[5], 0.2e1)) * sin(x[0]) + 0.2e1 * x[4] * x[5] * La + g) * Lh * (mb - mf)) * Lh * cos(x[1]) - (Lh * Lh * (mf + mb) + (mf + mb) * La * La + Lw * Lw * mw) * (-0.2e1 * La * (Kf * (u[0] - u[1]) * Lh + F_v2[0] * x[4] + (double) (F_c2[0] * (x[1]>0 ? 1 : (x[1]<0 ? -1 : 0)))) * sin(x[0]) * (mb - mf) * sin(x[1]) + 0.8e1 * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * x[3] * x[5] * Lh + La * ((g * mb + g * mf + Kf * (u[0] + u[1])) * La - Lw * mw * g) * (mb - mf))) * cos(x[0]) + ((pow(mf + mb, 0.2e1) * Lh * Lh + La * La * pow(mb - mf, 0.2e1)) * pow(cos(x[1]), 0.2e1) - (Lh * Lh * (mf + mb) + (mf + mb) * La * La + Lw * Lw * mw) * (mf + mb)) * (((g * mb + g * mf + Kf * (u[0] + u[1])) * La - Lw * mw * g) * sin(x[0]) * sin(x[1]) - (double) (F_c2[0] * (x[1]>0 ? 1 : (x[1]<0 ? -1 : 0))) - Kf * (u[0] - u[1]) * Lh - F_v2[0] * x[4]) * Lh * Lh) / (Lh * Lh * (mf + mb) + (mf + mb) * La * La + Lw * Lw * mw) / (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * pow(Lh, -0.2e1) * pow(cos(x[0]), -0.2e1) / 0.4e1;
  dx[5] = (0.4e1 * La * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * (x[5] * cos(x[0]) + x[3]) * Lh * Lh * (-x[5] * cos(x[0]) + x[3]) * (mb - mf) * cos(x[0]) * pow(cos(x[1]), 0.3e1) - ((0.8e1 * La * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * x[3] * x[5] * Lh * (mb - mf) * pow(cos(x[0]), 0.2e1) + ((g * mb + g * mf + Kf * (u[0] + u[1])) * La - Lw * mw * g) * (pow(mf + mb, 0.2e1) * Lh * Lh + La * La * pow(mb - mf, 0.2e1))) * sin(x[1]) + 0.8e1 * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * x[3] * (mf + mb) * Lh * Lh * (x[5] * sin(x[0]) + x[4]) * cos(x[0]) - (Kf * (u[0] - u[1]) * Lh + F_v2[0] * x[4] + (double) (F_c2[0] * (x[1]>0 ? 1 : (x[1]<0 ? -1 : 0)))) * sin(x[0]) * (pow(mf + mb, 0.2e1) * Lh * Lh + La * La * pow(mb - mf, 0.2e1))) * Lh * pow(cos(x[1]), 0.2e1) - (((-0.8e1 * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * (mf + mb) * pow(x[5], 0.2e1) * Lh * Lh * sin(x[0]) - 0.4e1 * (mf + mb) * (0.2e1 * x[4] * x[5] * La * La * mb * mf + g * La * mb * mf - Lw * mw * (mf + mb) * (-0.2e1 * Lw * x[4] * x[5] + g) / 0.4e1) * Lh * Lh + g * La * Lw * mw * pow(mb - mf, 0.2e1) * (La + Lw)) * cos(x[0]) + ((g * mb + g * mf + Kf * (u[0] + u[1])) * La - Lw * mw * g) * (pow(mf + mb, 0.2e1) * Lh * Lh + La * La * pow(mb - mf, 0.2e1))) * sin(x[1]) + 0.4e1 * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * (-0.2e1 * pow(x[5], 0.2e1) * La * pow(cos(x[0]), 0.2e1) + (0.2e1 * x[4] * x[5] * La + g) * sin(x[0]) + La * (pow(x[4], 0.2e1) + pow(x[5], 0.2e1))) * Lh * (mb - mf)) * Lh * cos(x[0]) * cos(x[1]) + (Lh * Lh * (mf + mb) + (mf + mb) * La * La + Lw * Lw * mw) * ((-La * (Kf * (u[0] - u[1]) * Lh + F_v2[0] * x[4] + (double) (F_c2[0] * (x[1]>0 ? 1 : (x[1]<0 ? -1 : 0)))) * (mb - mf) * cos(x[0]) + ((g * mb + g * mf + Kf * (u[0] + u[1])) * La - Lw * mw * g) * (mf + mb) * Lh) * sin(x[1]) - sin(x[0]) * Lh * (-0.8e1 * (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) * x[3] * x[5] * cos(x[0]) + (mf + mb) * (Kf * (u[0] - u[1]) * Lh + F_v2[0] * x[4] + (double) (F_c2[0] * (x[1]>0 ? 1 : (x[1]<0 ? -1 : 0))))))) / (Lh * Lh * (mf + mb) + (mf + mb) * La * La + Lw * Lw * mw) / (La * La * mb * mf + Lw * Lw * mw * (mf + mb) / 0.4e1) / Lh * pow(cos(x[0]), -0.2e1) / 0.4e1;
}

/* Output equations. */
void compute_y(
    double *y,   /* Vector of outputs (length NY). */
    double t,    /* Time t (scalar). */
    double *x,   /* State vector (length nx). */
    double *u,   /* Input vector (length nu). */
    double **p,  /* p[j] points to the j-th estimated model parameters (a double array). */
    const mxArray *auxvar  /* Cell array of additional data. */
   )
{
    /*
      Define the output equation y = h(t, x, u, p[0],..., p[np-1], auvar)
      in the body of this function.
    */
    
    /*
      Accessing the contents of auxvar: see the discussion in compute_dx.
    */
    
    y[0] = x[1]; /* Pitch. */

}



/*----------------------------------------------------------------------- *
   DO NOT MODIFY THE CODE BELOW UNLESS YOU NEED TO PASS ADDITIONAL
   INFORMATION TO COMPUTE_DX AND COMPUTE_Y
 
   To add extra arguments to compute_dx and compute_y (e.g., size
   information), modify the definitions above and calls below.
 *-----------------------------------------------------------------------*/

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    /* Declaration of input and output arguments. */
    double *x, *u, **p, *dx, *y, *t;
    int     i, np, nu, nx;
    const mxArray *auxvar = NULL; /* Cell array of additional data. */
    
    if (nrhs < 3) {
        mexErrMsgIdAndTxt("IDNLGREY:ODE_FILE:InvalidSyntax",
        "At least 3 inputs expected (t, u, x).");
    }
    
    /* Determine if auxiliary variables were passed as last input.  */
    if ((nrhs > 3) && (mxIsCell(prhs[nrhs-1]))) {
        /* Auxiliary variables were passed as input. */
        auxvar = prhs[nrhs-1];
        np = nrhs - 4; /* Number of parameters (could be 0). */
    } else {
        /* Auxiliary variables were not passed. */
        np = nrhs - 3; /* Number of parameters. */
    }
    
    /* Determine number of inputs and states. */
    nx = mxGetNumberOfElements(prhs[1]); /* Number of states. */
    nu = mxGetNumberOfElements(prhs[2]); /* Number of inputs. */
    
    /* Obtain double data pointers from mxArrays. */
    t = mxGetPr(prhs[0]);  /* Current time value (scalar). */
    x = mxGetPr(prhs[1]);  /* States at time t. */
    u = mxGetPr(prhs[2]);  /* Inputs at time t. */
    
    p = mxCalloc(np, sizeof(double*));
    for (i = 0; i < np; i++) {
        p[i] = mxGetPr(prhs[3+i]); /* Parameter arrays. */
    }
    
    /* Create matrix for the return arguments. */
    plhs[0] = mxCreateDoubleMatrix(nx, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(NY, 1, mxREAL);
    dx      = mxGetPr(plhs[0]); /* State derivative values. */
    y       = mxGetPr(plhs[1]); /* Output values. */
    
    /*
      Call the state and output update functions.
      
      Note: You may also pass other inputs that you might need,
      such as number of states (nx) and number of parameters (np).
      You may also omit unused inputs (such as auxvar).
      
      For example, you may want to use orders nx and nu, but not time (t)
      or auxiliary data (auxvar). You may write these functions as:
          compute_dx(dx, nx, nu, x, u, p);
          compute_y(y, nx, nu, x, u, p);
    */
    
    /* Call function for state derivative update. */
    compute_dx(dx, t[0], x, u, p, auxvar);
    
    /* Call function for output update. */
    compute_y(y, t[0], x, u, p, auxvar);
    
    /* Clean up. */
    mxFree(p);
}
