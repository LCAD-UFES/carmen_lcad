/*
    polynomial line fitting
    Copyright (C) 2009 Bob Mottram and Giacomo Spigler
    fuzzgun@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef polynomial_h
#define polynomial_h

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <assert.h>
#include "drawing.h"

#ifndef ABS
    #define ABS(a) (((a) < 0) ? -(a) : (a))
#endif

class polynomial
{
    public:

        polynomial();
        ~polynomial();

        void Solve();
        void Init();
        void SetCoeff(int Exponent, float value);
        float Coeff(int Exponent);
        int GetDegree();
        void SetDegree(int NewVal);
        int XYCount();
        void AddPoint(float x, float y);
        float RegVal(float x);
        float GetRMSerror();
        void Show(unsigned char *img, int width, int height);


    private:
        int MaxO;            // max polynomial degree
        int GlobalO;         // degree of the polynomial expected
        bool Finished;
        float* SumX;
        float* SumYX;
        float** M;
        float* C; // coefficients
        std::vector<float> Xpoints;
        std::vector<float> Ypoints;

        void GaussSolve(int O);
        void BuildMatrix(int O);
        void FinalizeMatrix(int O);


};

#endif
