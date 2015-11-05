#include "Utm_To_Gdc_Converter.h"

#include <math.h>
#include "Utm_Coord_3d.h"
#include "Gdc_Coord_3d.h"

const double Utm_To_Gdc_Converter::DEGREES_PER_RADIAN = 57.2957795130823208768;

double			  Utm_To_Gdc_Converter::A,
                  Utm_To_Gdc_Converter::F,
                  Utm_To_Gdc_Converter::C,
                  Utm_To_Gdc_Converter::Eps2,
                  Utm_To_Gdc_Converter::Eps21,
                  Utm_To_Gdc_Converter::Eps25,
                  Utm_To_Gdc_Converter::Con,
                  Utm_To_Gdc_Converter::Con2,
                  Utm_To_Gdc_Converter::EF,
                  Utm_To_Gdc_Converter::Epsp2,
                  Utm_To_Gdc_Converter::Con6,
                  Utm_To_Gdc_Converter::Con24,
                  Utm_To_Gdc_Converter::Con120,
                  Utm_To_Gdc_Converter::Con720,
                  Utm_To_Gdc_Converter::polx2b,
                  Utm_To_Gdc_Converter::polx3b,
                  Utm_To_Gdc_Converter::polx4b,
                  Utm_To_Gdc_Converter::polx5b,
                  Utm_To_Gdc_Converter::conap;


void Utm_To_Gdc_Converter::Init()
    {
        CreateConstants(6378137,298.257223563); // default to wgs 84
    }

void Utm_To_Gdc_Converter::CreateConstants(double a, double f)
    {
        double polx1a,polx2a,polx4a,polx6a,polx8a;
        
        A = a;
        F = f;

        //  Create the ERM constants.

        F      = 1/(F);
        C      = (A) * (1-F);
        Eps2   = (F) * (2.0-F);
        Eps25  = .25 * (Eps2);
        EF     = F/(2.0-F);
        Con    = (1.0-Eps2);
        Con2   = 2 / (1.0-Eps2);
        Con6   = .166666666666667;
        Con24  = 4 * .0416666666666667/(1-Eps2);
        Con120 = .00833333333333333;
        Con720 = 4 * .00138888888888888/(1-Eps2);
        polx1a = 1.0 - Eps2 / 4.0 - 3.0/64.0 *
                          pow(Eps2,2) - 5.0/256.0 * pow(Eps2,3)
                          - 175.0/16384.0 * pow(Eps2,4);

        conap  = A * polx1a;

        polx2a = 3.0/2.0 * EF - 27.0/32.0 * pow(EF,3);

        polx4a = 21.0/16.0 * pow(EF,2) - 55.0/32.0 * pow(EF,4);

        polx6a = 151.0/96.0 * pow(EF,3);

        polx8a = 1097.0/512.0 * pow(EF,4);

        polx2b = polx2a * 2.0 + polx4a * 4.0 + polx6a * 6.0 + polx8a * 8.0;

        polx3b = polx4a * -8.0 - polx6a * 32.0- 80.0 *polx8a;
        polx4b = polx6a * 32.0 + 192.0*polx8a;
        polx5b = -128.0 *polx8a;


    } // end init

void Utm_To_Gdc_Converter::Convert(Utm_Coord_3d utm_coord, Gdc_Coord_3d& gdc_coord)
    {
        Utm_Coord_3d utm[1];
        Gdc_Coord_3d gdc[1];

        utm[0] = utm_coord;

        Convert(1,utm,gdc);

		gdc_coord = gdc[0];
    }

void Utm_To_Gdc_Converter::Convert(int count, const Utm_Coord_3d utm[], Gdc_Coord_3d gdc[])
    {
        double source_x, source_y,u,su,cu,su2,xlon0,temp,phi1,sp,sp2,cp,cp2,tp,tp2,eta2,top,rn,b3,b4,b5,b6,d1,d2;

        for(int i=0; i < count; i++)
        {

            gdc[i].elevation = utm[i].z;

            source_x = utm[i].x;

            source_x = (source_x - 500000.0)/.9996;

            if (utm[i].hemisphere_north==true)
                source_y = utm[i].y / .9996;
            else
                source_y = (utm[i].y - 1.0E7)/.9996;

            u = source_y / conap;

            /* TEST U TO SEE IF AT POLES */

            su = sin(u);
            cu = cos(u);
            su2 = su * su;

            /* THE SNYDER FORMULA FOR PHI1 IS OF THE FORM
             PHI1=U+POLY2A*SIN(2U)+POLY3A*SIN(4U)+POLY4ASIN(6U)+...
             BY USING MULTIPLE ANGLE TRIGONOMETRIC IDENTITIES AND APPROPRIATE FACTORING
            JUST THE SINE AND COSINE ARE REQUIRED
             NOW READY TO GET PHI1
             */

            xlon0= ( 6.0 * ((double) utm[i].zone) - 183.0) / DEGREES_PER_RADIAN;

            temp = polx2b + su2 * (polx3b + su2 * (polx4b + su2 * polx5b));

            phi1 = u + su * cu * temp;

             /* COMPUTE VARIABLE COEFFICIENTS FOR FINAL RESULT
                COMPUTE THE VARIABLE COEFFICIENTS OF THE LAT AND LON
                EXPANSIONS */

            sp = sin(phi1);
            cp = cos(phi1);
            tp = sp / cp;
            tp2 = tp * tp;
            sp2 = sp * sp;
            cp2 = cp * cp;
            eta2 = Epsp2 * cp2;

            top = .25-(sp2*(Eps2 / 4));

             /* inline sq root*/

            rn = A / ( (.25 - Eps25 * sp2 + .9999944354799/4) +
                (.25-Eps25 * sp2)/(.25 - Eps25 * sp2 + .9999944354799/4));

            b3 = 1.0 + tp2 + tp2 + eta2;

            b4 = 5 + tp2 * (3 - 9 * eta2) + eta2 * ( 1 - 4 * eta2);

            b5 = 5 + tp2 * (tp2 * 24.0 + 28.0);

            b5 += eta2 * (tp2 * 8.0 + 6.0);

            b6 = 46.0 - 3.0 * eta2 + tp2 * (-252.0 - tp2 * 90.0);

            b6 = eta2 * (b6 + eta2 * tp2 * (tp2 * 225.0 - 66.0));
            b6 += 61.0 + tp2 * (tp2 * 45.0 + 90.0);

            d1 = source_x / rn;
            d2 = d1 * d1;

            gdc[i].latitude = phi1 - tp * top * (d2 * (Con2 + d2 * ((-Con24) * b4 + d2 *
                        Con720 * b6)));

            gdc[i].longitude = xlon0 + d1 * (1.0 + d2 * (-Con6 * b3 + d2 * Con120 * b5)) / cp;

             /* TRANSVERSE MERCATOR COMPUTATIONS DONE */

            gdc[i].latitude *= DEGREES_PER_RADIAN;

            gdc[i].longitude *= DEGREES_PER_RADIAN;

        } // end for

    } // end Convert
