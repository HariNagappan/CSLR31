/***************************************************************************************************
    * File:           Coordinator.cpp (Original: geo_utils.cpp)
    * Description:    This file provides a set of geographic utility functions encapsulated within
    * the GeoUtils class. These functions perform common geodetic calculations
    * on a spherical Earth model, including distance, bearing, and destination
    * point determination. The main function demonstrates their usage.
    ***************************************************************************************************/


#pragma once

#include <bits/stdc++.h>
using namespace std;

namespace Utils 
{

    /***************************************************************************************************
    * Struct:         LatLon
    * Description:    A simple structure to represent a geographic coordinate with latitude and
    * longitude values in degrees.
    ***************************************************************************************************/
    typedef struct LatLon
    {
        double lat; // degrees
        double lon; // degrees
        LatLon() : lat(0), lon(0) {}
        LatLon(double lat_, double lon_) : lat(lat_), lon(lon_) {}
    }LatLon;

    /***************************************************************************************************
    * Class:          GeoUtils
    * Description:    A utility class containing a collection of static methods for performing
    * geographic calculations. All calculations assume a spherical Earth model.
    ***************************************************************************************************/
    class GeoUtils 
    {
    public:
        // Earth radius (mean) in meters
        static constexpr double R = 6371000.0;

        /***********************************************************************************************
        * Function:       toRad / toDeg
        * Description:    Converts angles between degrees and radians.
        * Inputs:         deg - Angle in degrees. / rad - Angle in radians.
        * Outputs:        The converted angle.
        ***********************************************************************************************/
        static double toRad(double deg) { return deg * M_PI / 180.0; }
        
        static double toDeg(double rad) { return rad * 180.0 / M_PI; }

        /***********************************************************************************************
        * Function:       distance
        * Description:    Calculates the great-circle distance between two coordinates using the
        * Haversine formula.
        * Inputs:         a - The starting coordinate.
        * b - The ending coordinate.
        * Outputs:        The distance in meters.
        ***********************************************************************************************/
        static double distance(const LatLon &a, const LatLon &b) 
        {
            double phi1 = toRad(a.lat), phi2 = toRad(b.lat);
            double dphi = toRad(b.lat - a.lat);
            double dlambda = toRad(b.lon - a.lon);

            double s = sin(dphi/2.0);
            double t = sin(dlambda/2.0);
            double A = s*s + cos(phi1) * cos(phi2) * t*t;
            double C = 2.0 * atan2(sqrt(A), sqrt(1.0 - A));
            
            return R * C;
        }

        /***********************************************************************************************
        * Function:       bearing
        * Description:    Calculates the initial bearing (forward azimuth) from a starting point
        * to an ending point.
        * Inputs:         from - The starting coordinate.
        * to   - The ending coordinate.
        * Outputs:        The initial bearing in degrees (0 to 360).
        ***********************************************************************************************/
        static double bearing(const LatLon &from, const LatLon &to) 
        {
            double phi1 = toRad(from.lat), phi2 = toRad(to.lat);
            double lambda1 = toRad(from.lon), lambda2 = toRad(to.lon);
            double dlambda = lambda2 - lambda1;

            double y = sin(dlambda) * cos(phi2);
            double x = cos(phi1)*sin(phi2) - sin(phi1)*cos(phi2)*cos(dlambda);
            double theta = atan2(y, x); // radians
            double deg = fmod(toDeg(theta) + 360.0, 360.0);
            
            return deg;
        }

        /***********************************************************************************************
        * Function:       destinationPoint
        * Description:    Calculates the destination coordinate when starting from a point and
        * traveling a specific distance along a constant bearing.
        * Inputs:         from        - The starting coordinate.
        * bearing_deg - The initial bearing in degrees.
        * distance_m  - The distance to travel in meters.
        * Outputs:        The destination LatLon coordinate.
        ***********************************************************************************************/
        static LatLon destinationPoint(const LatLon &from, double bearing_deg, double distance_m) 
        {
            double phi1 = toRad(from.lat);
            double lambda1 = toRad(from.lon);
            double theta = toRad(bearing_deg);
            double delta = distance_m / R; // angular distance in radians

            double phi2 = asin( sin(phi1)*cos(delta) + cos(phi1)*sin(delta)*cos(theta) );
            double lambda2 = lambda1 + atan2(sin(theta)*sin(delta)*cos(phi1),
                                            cos(delta) - sin(phi1)*sin(phi2));
            
            // Normalize lon to -180..+180
            double lon_deg = fmod(toDeg(lambda2) + 540.0, 360.0) - 180.0;
            
            return LatLon(toDeg(phi2), lon_deg);
        }

        /***********************************************************************************************
        * Function:       angleBetween
        * Description:    Calculates the interior angle in degrees formed by the path A-B-C at point B.
        * It computes the bearings B->A and B->C and finds the smallest
        * difference between them.
        * Inputs:         A, B, C - The three LatLon coordinates forming the angle.
        * Outputs:        The angle at B in degrees [0, 180].
        ***********************************************************************************************/
        static double angleBetween(const LatLon &A, const LatLon &B, const LatLon &C) 
        {
            double brBA = bearing(B, A);
            double brBC = bearing(B, C);
            double diff = fabs(brBA - brBC);

            if (diff > 180.0) diff = 360.0 - diff;
            
            return diff;
        }

        /***********************************************************************************************
        * Function:       selectCoordinate
        * Description:    Finds a coordinate 'E' such that the angle E-fixed-dest is approximately
        * equal to a desired angle. It works by rotating the bearing from 'fixed'
        * to 'dest' by the desired angle and placing 'E' at the same distance.
        * Inputs:         fixed      - The vertex of the angle.
        * dest       - The reference point for the angle.
        * angle_deg  - The desired angle of rotation in degrees (clockwise).
        * Outputs:        A pair containing the new coordinate 'E' and the actual computed angle.
        ***********************************************************************************************/
        static pair<LatLon, double> selectCoordinate(const LatLon &fixed,
                                                    const LatLon &dest,
                                                    double angle_deg) 
        {
            // 1) compute bearing from fixed to dest
            double brFD = bearing(fixed, dest);

            // 2) compute distance from fixed to dest
            double distFD = distance(fixed, dest);

            // 3) rotate bearing by angle_deg (positive = clockwise)
            //    If you prefer counter-clockwise, pass negative angle_deg.
            double brExpected = fmod(brFD + angle_deg + 360.0, 360.0);

            // 4) compute expected coordinate at same distance
            LatLon expected = destinationPoint(fixed, brExpected, distFD);

            // 5) compute actual angle between expected and dest around fixed to report back
            double actualAngle = angleBetween(expected, fixed, dest); // angle at fixed

            return { expected, actualAngle };
        }
    };

}

//***************************************************************************************************

/***************************************************************************************************
* Function:       main
* Description:    Entry point of the program. Demonstrates the use of the GeoUtils class by
* calculating distance, angle, and selecting a new coordinate based on an
* angle constraint.
***************************************************************************************************/

/*

using namespace Utils;
int main() 
{
    // Example points:
    // Fixed (F): New Delhi (approx)
    LatLon F(28.6139, 77.2090);
    // Destination (D): Mumbai (approx)
    LatLon D(19.0760, 72.8777);
    // Another point (A) for angle demo: Kolkata
    LatLon A(22.5726, 88.3639);

    cout << fixed << setprecision(3);

    double dFD = GeoUtils::distance(F, D);
    cout << "Distance F -> D: " << dFD << " meters\n";

    double angleAtF = GeoUtils::angleBetween(A, F, D); // angle A-F-D in degrees
    cout << "Angle A - F - D (degrees): " << angleAtF << "\n";

    // Suppose we want an expected coordinate E such that angle(E - F - D) = 30 degrees (clockwise).
    double desiredAngle = 30.0;
    auto result = GeoUtils::selectCoordinate(F, D, desiredAngle);
    LatLon E = result.first;
    double actualAngle = result.second;

    cout << "Selected coordinate E (lat,lon): " << E.lat << ", " << E.lon << "\n";
    cout << "Requested rotation (deg): " << desiredAngle << ", actual angle E-F-D (deg): " << actualAngle << "\n";

    // Example: distance between E and D (optional)
    double distED = GeoUtils::distance(E, D);
    cout << "Distance E -> D: " << distED << " meters\n";

    return 0;
}

*/
