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
    
    double *F_v1, *F_c1, *mw, *Kf, *La, *Lh, *Lw, *ta; /* Estimated model parameters. */
    const double mh = 1.308;
    const double g = 9.81;
    const double mf = 0.5*mh;
    const double mb = 0.5*mh;
    

    F_v1 = p[0];
    F_c1 = p[1];
    mw = p[2];
    Kf = p[3];
    La = p[4];
    Lh = p[5];
    Lw = p[6];
    ta = p[7];

    dx[0] = x[3]; /* Elevation Rate. */
    dx[1] = x[4]; /* Pitch Rate. */
    dx[2] = x[5]; /* Travel Rate. */

	/* Elevation Friction Model */
	dx[3] = (-0.4e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * sin(x[1]) * (pow(cos(x[1]), 0.2e1) - 0.2e1) * (mb - mf) * pow(x[5], 0.2e1) * La[0] * Lh[0] * pow(cos(x[0]), 0.3e1) + (0.8e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * x[3] * (mb - mf) * x[5] * La[0] * Lh[0] * pow(cos(x[1]), 0.3e1) + (-0.8e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * (mf + mb) * pow(x[5], 0.2e1) * Lh[0] * Lh[0] * sin(x[0]) + ((-0.8e1 * x[4] * x[5] * Lh[0] * Lh[0] * mf + Lw[0] * mw[0] * g) * mb * mb + (-0.8e1 * Lh[0] * Lh[0] * mf * mf * x[4] * x[5] - 0.2e1 * Lw[0] * g * mf * mw[0]) * mb + g * Lw[0] * mf * mf * mw[0]) * La[0] * La[0] - 0.4e1 * ((Lh[0] * Lh[0] * mf - Lw[0] * Lw[0] * mw[0] / 0.4e1) * mb * mb + mf * (Lh[0] * Lh[0] * mf + Lw[0] * Lw[0] * mw[0] / 0.2e1) * mb - Lw[0] * Lw[0] * mf * mf * mw[0] / 0.4e1) * g * La[0] + Lh[0] * Lh[0] * Lw[0] * mw[0] * pow(mf + mb, 0.2e1) * (-0.2e1 * Lw[0] * x[4] * x[5] + g)) * pow(cos(x[1]), 0.2e1) - 0.8e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * x[3] * (mb - mf) * x[5] * La[0] * Lh[0] * cos(x[1]) - 0.4e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * (((mf + mb) * La[0] * La[0] - Lh[0] * Lh[0] * mb - Lh[0] * Lh[0] * mf + Lw[0] * Lw[0] * mw[0]) * pow(x[5], 0.2e1) * sin(x[0]) + La[0] * g * (mf + mb) - 0.2e1 * x[4] * x[5] * Lh[0] * Lh[0] * mb - 0.2e1 * x[4] * x[5] * Lh[0] * Lh[0] * mf - Lw[0] * mw[0] * g)) * pow(cos(x[0]), 0.2e1) + ((0.4e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * pow(x[3], 0.2e1) * (mb - mf) * La[0] * Lh[0] * sin(x[1]) + ((double) (F_c1[0] * (x[0]>0 ? 1 : (x[0]<0 ? -1 : 0))) + (g * mb + g * mf + Kf[0] * (x[7] + x[6])) * La[0] - Lw[0] * mw[0] * g + F_v1[0] * x[3]) * (pow(mb - mf, 0.2e1) * La[0] * La[0] + Lh[0] * Lh[0] * pow(mf + mb, 0.2e1))) * pow(cos(x[1]), 0.2e1) + (-0.8e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * (mf + mb) * x[3] * (x[5] * sin(x[0]) + x[4]) * Lh[0] * Lh[0] * sin(x[1]) - ((mf + mb) * La[0] * La[0] + Lh[0] * Lh[0] * mb + Lh[0] * Lh[0] * mf + Lw[0] * Lw[0] * mw[0]) * (mb - mf) * (x[7] - x[6]) * La[0] * Kf[0]) * cos(x[1]) - 0.4e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * ((mb - mf) * Lh[0] * ((0.2e1 * x[4] * x[5] * La[0] + g) * sin(x[0]) + La[0] * (pow(x[4], 0.2e1) + pow(x[5], 0.2e1))) * sin(x[1]) - (double) (F_c1[0] * (x[0]>0 ? 1 : (x[0]<0 ? -1 : 0))) + (-g * mb - g * mf - Kf[0] * (x[7] + x[6])) * La[0] + Lw[0] * mw[0] * g - F_v1[0] * x[3])) * cos(x[0]) + cos(x[1]) * (((g * mb + g * mf + Kf[0] * (x[7] + x[6])) * La[0] - Lw[0] * mw[0] * g) * pow(cos(x[1]), 0.2e1) - Kf[0] * sin(x[0]) * Lh[0] * (x[7] - x[6]) * sin(x[1]) + (-g * mb - g * mf - Kf[0] * (x[7] + x[6])) * La[0] + Lw[0] * mw[0] * g) * (pow(mb - mf, 0.2e1) * La[0] * La[0] + Lh[0] * Lh[0] * pow(mf + mb, 0.2e1))) / (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) / ((mf + mb) * La[0] * La[0] + Lh[0] * Lh[0] * mb + Lh[0] * Lh[0] * mf + Lw[0] * Lw[0] * mw[0]) / cos(x[0]) / 0.4e1;
	dx[4] = (-0.4e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * cos(x[1]) * ((-mb - mf) * Lh[0] * Lh[0] + (mf + mb) * La[0] * La[0] + Lw[0] * Lw[0] * mw[0]) * sin(x[1]) * pow(x[5], 0.2e1) * Lh[0] * pow(cos(x[0]), 0.4e1) + cos(x[1]) * (0.4e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * (mb - mf) * pow(x[5], 0.2e1) * La[0] * sin(x[0]) * Lh[0] * Lh[0] * pow(cos(x[1]), 0.2e1) + 0.8e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * x[3] * x[5] * Lh[0] * ((mf + mb) * La[0] * La[0] + Lw[0] * Lw[0] * mw[0]) * cos(x[1]) + (mb - mf) * (-0.8e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * pow(x[5], 0.2e1) * La[0] * Lh[0] * Lh[0] * sin(x[0]) - 0.4e1 * (0.2e1 * x[4] * x[5] * La[0] * La[0] * mb * mf + g * La[0] * mb * mf - Lw[0] * mw[0] * (mf + mb) * (-0.2e1 * Lw[0] * x[4] * x[5] + g) / 0.4e1) * La[0] * Lh[0] * Lh[0] + Lw[0] * mw[0] * g * ((mf + mb) * La[0] * La[0] + Lw[0] * Lw[0] * mw[0]) * (La[0] + Lw[0]))) * pow(cos(x[0]), 0.3e1) + (-(-0.8e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * x[3] * (mb - mf) * x[5] * La[0] * sin(x[0]) * sin(x[1]) + (x[7] - x[6]) * (pow(mb - mf, 0.2e1) * La[0] * La[0] + Lh[0] * Lh[0] * pow(mf + mb, 0.2e1)) * Kf[0]) * Lh[0] * Lh[0] * pow(cos(x[1]), 0.2e1) + (Lh[0] * ((-0.4e1 * (mf + mb) * (0.2e1 * x[4] * x[5] * La[0] * La[0] * mb * mf + g * La[0] * mb * mf - Lw[0] * mw[0] * (mf + mb) * (-0.2e1 * Lw[0] * x[4] * x[5] + g) / 0.4e1) * Lh[0] * Lh[0] + g * La[0] * Lw[0] * mw[0] * pow(mb - mf, 0.2e1) * (La[0] + Lw[0])) * sin(x[0]) + 0.4e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * ((pow(x[3], 0.2e1) - 0.2e1 * pow(x[5], 0.2e1)) * (mf + mb) * Lh[0] * Lh[0] + pow(x[3], 0.2e1) * ((mf + mb) * La[0] * La[0] + Lw[0] * Lw[0] * mw[0]))) * sin(x[1]) + ((double) (F_c1[0] * (x[0]>0 ? 1 : (x[0]<0 ? -1 : 0))) + (g * mb + g * mf + Kf[0] * (x[7] + x[6])) * La[0] - Lw[0] * mw[0] * g + F_v1[0] * x[3]) * (Lh[0] * Lh[0] * (mf + mb) + (mf + mb) * La[0] * La[0] + Lw[0] * Lw[0] * mw[0]) * (mb - mf) * La[0]) * cos(x[1]) - ((-mb - mf) * Lh[0] * Lh[0] + (mf + mb) * La[0] * La[0] + Lw[0] * Lw[0] * mw[0]) * (Lh[0] * Lh[0] * (mf + mb) + (mf + mb) * La[0] * La[0] + Lw[0] * Lw[0] * mw[0]) * (x[7] - x[6]) * Kf[0]) * pow(cos(x[0]), 0.2e1) + (-0.4e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * pow(x[3], 0.2e1) * (mb - mf) * La[0] * sin(x[0]) * Lh[0] * Lh[0] * pow(cos(x[1]), 0.3e1) + (0.8e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * (mf + mb) * x[3] * pow(Lh[0], 0.3e1) * x[4] * sin(x[0]) + 0.8e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * (mf + mb) * x[3] * x[5] * pow(Lh[0], 0.3e1) + (mf + mb) * ((g * mb + g * mf + Kf[0] * (x[7] + x[6])) * La[0] - Lw[0] * mw[0] * g) * (mb - mf) * La[0] * Lh[0] * Lh[0] + ((g * mb + g * mf + Kf[0] * (x[7] + x[6])) * La[0] - Lw[0] * mw[0] * g) * (mb - mf) * La[0] * ((mf + mb) * La[0] * La[0] + Lw[0] * Lw[0] * mw[0])) * pow(cos(x[1]), 0.2e1) + (((double) (F_c1[0] * (x[0]>0 ? 1 : (x[0]<0 ? -1 : 0))) + (g * mb + g * mf + Kf[0] * (x[7] + x[6])) * La[0] - Lw[0] * mw[0] * g + F_v1[0] * x[3]) * sin(x[0]) * (pow(mb - mf, 0.2e1) * La[0] * La[0] + Lh[0] * Lh[0] * pow(mf + mb, 0.2e1)) * sin(x[1]) + 0.4e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * (mb - mf) * (La[0] * (pow(x[4], 0.2e1) + pow(x[5], 0.2e1)) * sin(x[0]) + 0.2e1 * x[4] * x[5] * La[0] + g) * Lh[0]) * Lh[0] * cos(x[1]) - (Lh[0] * Lh[0] * (mf + mb) + (mf + mb) * La[0] * La[0] + Lw[0] * Lw[0] * mw[0]) * (0.2e1 * Kf[0] * sin(x[0]) * La[0] * Lh[0] * (mb - mf) * (x[7] - x[6]) * sin(x[1]) + 0.8e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * x[3] * x[5] * Lh[0] + ((g * mb + g * mf + Kf[0] * (x[7] + x[6])) * La[0] - Lw[0] * mw[0] * g) * (mb - mf) * La[0])) * cos(x[0]) + ((pow(mb - mf, 0.2e1) * La[0] * La[0] + Lh[0] * Lh[0] * pow(mf + mb, 0.2e1)) * pow(cos(x[1]), 0.2e1) - (mf + mb) * (Lh[0] * Lh[0] * (mf + mb) + (mf + mb) * La[0] * La[0] + Lw[0] * Lw[0] * mw[0])) * (((g * mb + g * mf + Kf[0] * (x[7] + x[6])) * La[0] - Lw[0] * mw[0] * g) * sin(x[0]) * sin(x[1]) + Kf[0] * Lh[0] * (x[7] - x[6])) * Lh[0]) / (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) / (Lh[0] * Lh[0] * (mf + mb) + (mf + mb) * La[0] * La[0] + Lw[0] * Lw[0] * mw[0]) / Lh[0] * pow(cos(x[0]), -0.2e1) / 0.4e1;
	dx[5] = (0.4e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * (mb - mf) * La[0] * Lh[0] * (-x[5] * cos(x[0]) + x[3]) * (x[5] * cos(x[0]) + x[3]) * cos(x[0]) * pow(cos(x[1]), 0.3e1) + ((-0.8e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * x[3] * (mb - mf) * x[5] * La[0] * Lh[0] * pow(cos(x[0]), 0.2e1) - ((g * mb + g * mf + Kf[0] * (x[7] + x[6])) * La[0] - Lw[0] * mw[0] * g) * (pow(mb - mf, 0.2e1) * La[0] * La[0] + Lh[0] * Lh[0] * pow(mf + mb, 0.2e1))) * sin(x[1]) - Lh[0] * (0.8e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * (mf + mb) * x[3] * (x[5] * sin(x[0]) + x[4]) * Lh[0] * cos(x[0]) + (x[7] - x[6]) * sin(x[0]) * (pow(mb - mf, 0.2e1) * La[0] * La[0] + Lh[0] * Lh[0] * pow(mf + mb, 0.2e1)) * Kf[0])) * pow(cos(x[1]), 0.2e1) - (((-0.8e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * (mf + mb) * pow(x[5], 0.2e1) * Lh[0] * Lh[0] * sin(x[0]) + ((-0.8e1 * x[4] * x[5] * Lh[0] * Lh[0] * mf + Lw[0] * mw[0] * g) * mb * mb + (-0.8e1 * Lh[0] * Lh[0] * mf * mf * x[4] * x[5] - 0.2e1 * Lw[0] * g * mf * mw[0]) * mb + g * Lw[0] * mf * mf * mw[0]) * La[0] * La[0] - 0.4e1 * ((Lh[0] * Lh[0] * mf - Lw[0] * Lw[0] * mw[0] / 0.4e1) * mb * mb + mf * (Lh[0] * Lh[0] * mf + Lw[0] * Lw[0] * mw[0] / 0.2e1) * mb - Lw[0] * Lw[0] * mf * mf * mw[0] / 0.4e1) * g * La[0] + Lh[0] * Lh[0] * Lw[0] * mw[0] * pow(mf + mb, 0.2e1) * (-0.2e1 * Lw[0] * x[4] * x[5] + g)) * cos(x[0]) + ((double) (F_c1[0] * (x[0]>0 ? 1 : (x[0]<0 ? -1 : 0))) + (g * mb + g * mf + Kf[0] * (x[7] + x[6])) * La[0] - Lw[0] * mw[0] * g + F_v1[0] * x[3]) * (pow(mb - mf, 0.2e1) * La[0] * La[0] + Lh[0] * Lh[0] * pow(mf + mb, 0.2e1))) * sin(x[1]) + 0.4e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * (-0.2e1 * pow(x[5], 0.2e1) * La[0] * pow(cos(x[0]), 0.2e1) + (0.2e1 * x[4] * x[5] * La[0] + g) * sin(x[0]) + La[0] * (pow(x[4], 0.2e1) + pow(x[5], 0.2e1))) * (mb - mf) * Lh[0]) * cos(x[0]) * cos(x[1]) + ((mf + mb) * La[0] * La[0] + Lh[0] * Lh[0] * mb + Lh[0] * Lh[0] * mf + Lw[0] * Lw[0] * mw[0]) * ((Kf[0] * La[0] * (mb - mf) * (x[7] - x[6]) * cos(x[0]) + (mf + mb) * ((g * mb + g * mf + Kf[0] * (x[7] + x[6])) * La[0] - Lw[0] * mw[0] * g)) * sin(x[1]) + (0.8e1 * (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) * x[3] * x[5] * cos(x[0]) + Kf[0] * Lh[0] * (mf + mb) * (x[7] - x[6])) * sin(x[0]))) / (La[0] * La[0] * mb * mf + Lw[0] * Lw[0] * mw[0] * (mf + mb) / 0.4e1) / ((mf + mb) * La[0] * La[0] + Lh[0] * Lh[0] * mb + Lh[0] * Lh[0] * mf + Lw[0] * Lw[0] * mw[0]) * pow(cos(x[0]), -0.2e1) / 0.4e1;
	dx[6] = ta[0]*x[6] - ta[0]*u[0];
	dx[7] = ta[0]*x[7] - ta[0]*u[1];
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
    
    y[0] = x[0]; /* Elevation. */

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
