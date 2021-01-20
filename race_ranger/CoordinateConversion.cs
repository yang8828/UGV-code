using System;
using System.Collections.Generic;
using System.Text;

namespace PathEx_Analysis
{
    class CoordinateConversion
    {
        public double latitude, lontitude;
        public double map_x, map_y;
        double pi = Math.PI;
        double sm_a = 6378137.0;
        double sm_b = 6356752.314;
        double UTMScaleFactor = 0.9996;
        public void UTMtoLatLon(double x, double y, bool southhemi, int zone)
        {
            double cmeridian;

            x -= 500000.0;
            x /= UTMScaleFactor;

            /* If in southern hemisphere, adjust y accordingly. */
            if (southhemi)
                y -= 10000000.0;

            y /= UTMScaleFactor;

            cmeridian = UTMCentralMeridian(zone);
            MapXYToLatLon(x, y, cmeridian);
        }
        double UTMCentralMeridian(int zone)
        {
            return DegToRad(-183.0 + (zone * 6.0));
        }
        double DegToRad(double deg)
        {
            return (deg / 180.0 * pi);
        }
        void MapXYToLatLon(double x, double y, double lambda0)
        {
            double phif, Nf, Nfpow, nuf2, ep2, tf, tf2, tf4, cf;
            double x1frac, x2frac, x3frac, x4frac, x5frac, x6frac, x7frac, x8frac;
            double x2poly, x3poly, x4poly, x5poly, x6poly, x7poly, x8poly;

            /* Get the value of phif, the footpoint latitude. */
            phif = FootpointLatitude(y);

            /* Precalculate ep2 */
            ep2 = (Math.Pow(sm_a, 2.0) - Math.Pow(sm_b, 2.0)) / Math.Pow(sm_b, 2.0);

            /* Precalculate cos (phif) */
            cf = Math.Cos(phif);

            /* Precalculate nuf2 */
            nuf2 = ep2 * Math.Pow(cf, 2.0);

            /* Precalculate Nf and initialize Nfpow */
            Nf = Math.Pow(sm_a, 2.0) / (sm_b * Math.Sqrt(1 + nuf2));
            Nfpow = Nf;

            /* Precalculate tf */
            tf = Math.Tan(phif);
            tf2 = tf * tf;
            tf4 = tf2 * tf2;

            /* Precalculate fractional coefficients for x**n in the equations
            below to simplify the expressions for latitude and longitude. */
            x1frac = 1.0 / (Nfpow * cf);

            Nfpow *= Nf;   /* now equals Nf**2) */
            x2frac = tf / (2.0 * Nfpow);

            Nfpow *= Nf;   /* now equals Nf**3) */
            x3frac = 1.0 / (6.0 * Nfpow * cf);

            Nfpow *= Nf;   /* now equals Nf**4) */
            x4frac = tf / (24.0 * Nfpow);

            Nfpow *= Nf;   /* now equals Nf**5) */
            x5frac = 1.0 / (120.0 * Nfpow * cf);

            Nfpow *= Nf;   /* now equals Nf**6) */
            x6frac = tf / (720.0 * Nfpow);

            Nfpow *= Nf;   /* now equals Nf**7) */
            x7frac = 1.0 / (5040.0 * Nfpow * cf);

            Nfpow *= Nf;   /* now equals Nf**8) */
            x8frac = tf / (40320.0 * Nfpow);

            /* Precalculate polynomial coefficients for x**n.
            -- x**1 does not have a polynomial coefficient. */
            x2poly = -1.0 - nuf2;

            x3poly = -1.0 - 2 * tf2 - nuf2;

            x4poly = 5.0 + 3.0 * tf2 + 6.0 * nuf2 - 6.0 * tf2 * nuf2 - 3.0 * (nuf2 * nuf2) - 9.0 * tf2 * (nuf2 * nuf2);

            x5poly = 5.0 + 28.0 * tf2 + 24.0 * tf4 + 6.0 * nuf2 + 8.0 * tf2 * nuf2;

            x6poly = -61.0 - 90.0 * tf2 - 45.0 * tf4 - 107.0 * nuf2 + 162.0 * tf2 * nuf2;

            x7poly = -61.0 - 662.0 * tf2 - 1320.0 * tf4 - 720.0 * (tf4 * tf2);

            x8poly = 1385.0 + 3633.0 * tf2 + 4095.0 * tf4 + 1575 * (tf4 * tf2);

            /* Calculate latitude */
            latitude = phif + x2frac * x2poly * (x * x) + x4frac * x4poly * Math.Pow(x, 4.0) + x6frac * x6poly * Math.Pow(x, 6.0) + x8frac * x8poly * Math.Pow(x, 8.0);
            latitude = latitude * 180.0 / pi;
            /* Calculate longitude */
            lontitude = lambda0 + x1frac * x + x3frac * x3poly * Math.Pow(x, 3.0) + x5frac * x5poly * Math.Pow(x, 5.0) + x7frac * x7poly * Math.Pow(x, 7.0);
            lontitude = lontitude * 180 / pi;
        }
        double FootpointLatitude(double y)
        {
            double y_, alpha_, beta_, gamma_, delta_, epsilon_, n;
            double result;

            /* Precalculate n (Eq. 10.18) */
            n = (sm_a - sm_b) / (sm_a + sm_b);

            /* Precalculate alpha_ (Eq. 10.22) */
            /* (Same as alpha in Eq. 10.17) */
            alpha_ = ((sm_a + sm_b) / 2.0) * (1 + (Math.Pow(n, 2.0) / 4) + (Math.Pow(n, 4.0) / 64));

            /* Precalculate y_ (Eq. 10.23) */
            y_ = y / alpha_;

            /* Precalculate beta_ (Eq. 10.22) */
            beta_ = (3.0 * n / 2.0) + (-27.0 * Math.Pow(n, 3.0) / 32.0) + (269.0 * Math.Pow(n, 5.0) / 512.0);

            /* Precalculate gamma_ (Eq. 10.22) */
            gamma_ = (21.0 * Math.Pow(n, 2.0) / 16.0) + (-55.0 * Math.Pow(n, 4.0) / 32.0);

            /* Precalculate delta_ (Eq. 10.22) */
            delta_ = (151.0 * Math.Pow(n, 3.0) / 96.0) + (-417.0 * Math.Pow(n, 5.0) / 128.0);

            /* Precalculate epsilon_ (Eq. 10.22) */
            epsilon_ = (1097.0 * Math.Pow(n, 4.0) / 512.0);

            /* Now calculate the sum of the series (Eq. 10.21) */
            result = y_ + (beta_ * Math.Sin(2.0 * y_)) + (gamma_ * Math.Sin(4.0 * y_)) + (delta_ * Math.Sin(6.0 * y_)) + (epsilon_ * Math.Sin(8.0 * y_));

            return result;
        }
        public void LonLat2UTM(double lat, double lon)
        {
            double WGS84_A = 6378137.0;		// major axis
            double WGS84_E = 0.0818191908;		// first eccentricity
            // UTM Parameters
            double UTM_K0 = 0.9996;			// scale factor
            double UTM_FE = 500000.0;		// false easting
            double UTM_FN_N = 0.0;           // false northing, northern hemisphere
            double UTM_FN_S = 10000000.0;    // false northing, southern hemisphere
            double UTM_E2 = (WGS84_E * WGS84_E);	// e^2
            double UTM_E4 = (UTM_E2 * UTM_E2);		// e^4
            double UTM_E6 = (UTM_E4 * UTM_E2);		// e^6
            double UTM_EP2 = (UTM_E2 / (1 - UTM_E2));	// e'^2
            double m0 = (1 - UTM_E2 / 4 - 3 * UTM_E4 / 64 - 5 * UTM_E6 / 256);
            double m1 = -(3 * UTM_E2 / 8 + 3 * UTM_E4 / 32 + 45 * UTM_E6 / 1024);
            double m2 = (15 * UTM_E4 / 256 + 45 * UTM_E6 / 1024);
            double m3 = -(35 * UTM_E6 / 3072);
            double RADIANS_PER_DEGREE = Math.PI / 180.0;
            // compute the central meridian
            int cm = ((lon >= 0.0)
                               ? ((int)lon - ((int)lon) % 6 + 3)
                               : ((int)lon - ((int)lon) % 6 - 3));
            //int cm = 117;
            // convert degrees into radians
            double rlat = lat * RADIANS_PER_DEGREE;
            double rlon = lon * RADIANS_PER_DEGREE;
            double rlon0 = cm * RADIANS_PER_DEGREE;

            // compute trigonometric functions
            double slat = Math.Sin(rlat);
            double clat = Math.Cos(rlat);
            double tlat = Math.Tan(rlat);

            // decide the false northing at origin
            double fn = (lat > 0) ? UTM_FN_N : UTM_FN_S;

            double T = tlat * tlat;
            double C = UTM_EP2 * clat * clat;
            double A = (rlon - rlon0) * clat;
            double M = WGS84_A * (m0 * rlat + m1 * Math.Sin(2 * rlat)
                      + m2 * Math.Sin(4 * rlat) + m3 * Math.Sin(6 * rlat));
            double V = WGS84_A / Math.Sqrt(1 - UTM_E2 * slat * slat);

            // compute the easting-northing coordinates
            map_x = UTM_FE + UTM_K0 * V * (A + (1 - T + C) * Math.Pow(A, 3) / 6
                            + (5 - 18 * T + T * T + 72 * C - 58 * UTM_EP2) * Math.Pow(A, 5) / 120);
            map_y = fn + UTM_K0 * (M + V * tlat * (A * A / 2
                                + (5 - T + 9 * C + 4 * C * C) * Math.Pow(A, 4) / 24
                                + ((61 - 58 * T + T * T + 600 * C - 330 * UTM_EP2)
                               * Math.Pow(A, 6) / 720)));
        }
    }
}
