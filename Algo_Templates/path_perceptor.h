#ifndef M_PI //Include guard for M_PI
#define M_PI 3.14159265358979323846


namespace Utils
{


    /***************************************************************************************************
    * File:           path_perceptor.cpp (Original: flight_path_smoother.cpp)
    * Description:    This file contains the implementation for the PathSmoother class,
    * which generates a smooth, adaptively sampled flight path from a given set
    * of waypoints. It uses geographic utility functions to calculate distances
    * and bearings on a spherical Earth model, producing more points near the
    * start and end of the journey.
    ***************************************************************************************************/


    #include <bits/stdc++.h>
    using namespace std;

    /***************************************************************************************************
    * Struct:         LatLon
    * Description:    A simple structure to represent a geographic coordinate with latitude and
    * longitude values in degrees.
    ***************************************************************************************************/
    struct LatLon 
    {
        double lat; // degrees
        double lon; // degrees
        LatLon() : lat(0), lon(0) {}
        LatLon(double la, double lo) : lat(la), lon(lo) {}
    };

    /***************************************************************************************************
    * Struct:         Geo
    * Description:    A utility struct containing static methods for geographic calculations based
    * on a spherical Earth model. It includes functions for distance (Haversine),
    * bearing, and calculating a destination point.
    ***************************************************************************************************/
    struct Geo 
    {
        static constexpr double R = 6371000.0; // mean Earth radius (meters)
        static double toRad(double deg) { return deg * M_PI / 180.0; }
        static double toDeg(double rad) { return rad * 180.0 / M_PI; }

        /***********************************************************************************************
        * Function:       distance_m
        * Description:    Calculates the great-circle distance between two LatLon points using the
        * Haversine formula.
        * Inputs:         a - The starting coordinate.
        * b - The ending coordinate.
        * Outputs:        The distance in meters.
        ***********************************************************************************************/
        static double distance_m(const LatLon &a, const LatLon &b) 
        {
            double phi1 = toRad(a.lat), phi2 = toRad(b.lat);
            double dphi = toRad(b.lat - a.lat);
            double dlambda = toRad(b.lon - a.lon);
            double s1 = sin(dphi / 2.0);
            double s2 = sin(dlambda / 2.0);
            double A = s1*s1 + cos(phi1) * cos(phi2) * s2*s2;
            double C = 2.0 * atan2(sqrt(A), sqrt(max(0.0, 1.0 - A)));
            return R * C;
        }

        /***********************************************************************************************
        * Function:       bearing_deg
        * Description:    Calculates the initial bearing (forward azimuth) from a starting point
        * to an ending point.
        * Inputs:         from - The starting coordinate.
        * to   - The ending coordinate.
        * Outputs:        The initial bearing in degrees (0 to 360).
        ***********************************************************************************************/
        static double bearing_deg(const LatLon &from, const LatLon &to) 
        {
            double phi1 = toRad(from.lat), phi2 = toRad(to.lat);
            double lam1 = toRad(from.lon), lam2 = toRad(to.lon);
            double dl = lam2 - lam1;
            double y = sin(dl) * cos(phi2);
            double x = cos(phi1)*sin(phi2) - sin(phi1)*cos(phi2)*cos(dl);
            double th = atan2(y, x);
            double deg = fmod(toDeg(th) + 360.0, 360.0);
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
            double delta = distance_m / R; // angular distance

            double phi2 = asin( sin(phi1)*cos(delta) + cos(phi1)*sin(delta)*cos(theta) );
            double lambda2 = lambda1 + atan2(sin(theta)*sin(delta)*cos(phi1),
                                            cos(delta) - sin(phi1)*sin(phi2));
            // normalize lon to -180..+180
            double lon_deg = fmod(toDeg(lambda2) + 540.0, 360.0) - 180.0;
            return LatLon(toDeg(phi2), lon_deg);
        }

    };

    /***************************************************************************************************
    * Class:          PathSmoother
    * Description:    The main class for generating a smoothed flight path. It provides a static
    * method to process a list of waypoints into a high-resolution path with
    * adaptive sampling density based on proximity to the start and end points.
    ***************************************************************************************************/
    class PathSmoother 
    {
    public:
        /***********************************************************************************************
        * Struct:         Params
        * Description:    A configuration struct for the path generation algorithm. It defines the
        * radii and spacing parameters for "near", "mid", and "far" zones relative
        * to the start and destination points.
        ***********************************************************************************************/
        struct Params 
        {
            double near_radius_km = 50.0;
            double near_spacing_km = 50.0;
            double mid_radius_km = 250.0;
            double mid_spacing_km = 250.0;
            double far_spacing_km = 500.0;
            double min_spacing_km = 0.1;
        };

        /***********************************************************************************************
        * Function:       generatePath
        * Description:    Takes a list of waypoints and generates a smoothed, high-resolution path.
        * The path is sampled more densely near the global start and end points.
        * It iterates through each segment between waypoints, adding interpolated
        * points along the great-circle path.
        * Inputs:         waypoints - An ordered vector of LatLon waypoints.
        * params    - A Params struct with configuration for sampling density.
        * Outputs:        A vector of LatLon points representing the smoothed path.
        ***********************************************************************************************/
        static vector<LatLon> generatePath(const vector<LatLon> &waypoints, const Params &params = Params()) 
        {
            vector<LatLon> out;
            
            if (waypoints.empty()) return out;
            
            if (waypoints.size() == 1) 
            {
                out.push_back(waypoints.front());
                return out;
            }

            const LatLon &globalStart = waypoints.front();
            const LatLon &globalEnd = waypoints.back();

            // meters conversion
            const double near_radius_m = params.near_radius_km * 1000.0;
            const double mid_radius_m  = params.mid_radius_km  * 1000.0;
            const double near_spacing_m = max(1.0, params.near_spacing_km * 1000.0); // at least 1 m
            const double mid_spacing_m  = max(1.0, params.mid_spacing_km  * 1000.0);
            const double far_spacing_m  = max(1.0, params.far_spacing_km  * 1000.0);
            const double min_spacing_m = max(0.001, params.min_spacing_km * 1000.0);

            // helper: spacing at a particular candidate point determined by proximity to start/end
            auto choose_spacing_at = [&](const LatLon &p)->double 
            {
                double dstart = Geo::distance_m(p, globalStart);
                double dend   = Geo::distance_m(p, globalEnd);
                double dmin = min(dstart, dend);
                
                if (dmin <= near_radius_m) return max(min_spacing_m, near_spacing_m);
                
                if (dmin <= mid_radius_m)  return max(min_spacing_m, mid_spacing_m);
                
                return max(min_spacing_m, far_spacing_m);
            };

            // Iterate segments
            for (size_t i = 0; i + 1 < waypoints.size(); ++i) 
            {
                const LatLon &A = waypoints[i];
                const LatLon &B = waypoints[i+1];

                // Always add A unless it's a duplicate of last added
                if (out.empty()) out.push_back(A);
                else 
                {
                    const LatLon &last = out.back();
                    if (Geo::distance_m(last, A) > 0.5) // avoid duplicates (0.5 m tolerance)
                        out.push_back(A);
                }

                double segLen = Geo::distance_m(A, B); // meters
                if (segLen <= 1.0) 
                {
                    // almost zero length; continue
                    continue;
                }

                double bearingAB = Geo::bearing_deg(A, B);

                // Walk along the great-circle from A to B using adaptive step sizes.
                // We'll step using spacing determined at the current candidate point.
                double s = 0.0; // distance from A already placed
                
                while (true) 
                {
                    // determine spacing at current point (point at s)
                    LatLon currP = Geo::destinationPoint(A, bearingAB, s);
                    double spacing = choose_spacing_at(currP);

                    // Ensure we don't get stuck: ensure spacing is reasonable relative to remaining distance
                    double remaining = segLen - s;
                    if (remaining <= 0.0) break;

                    // If spacing is larger than remaining, place final point B and break
                    if (spacing >= remaining - 1e-6) 
                    {
                        // Add B (will be added by next outer loop iteration as A, but ensure last point of segment)
                        out.push_back(B);
                        break;
                    }

                    // advance by spacing, but ensure we always make progress
                    double step = max(min_spacing_m, min(spacing, remaining));
                    s += step;

                    // generate new point at distance s from A
                    LatLon nextP = Geo::destinationPoint(A, bearingAB, s);

                    // avoid adding points that are extremely close to previous (numerical)
                    if (Geo::distance_m(out.back(), nextP) > 0.5) 
                    {
                        out.push_back(nextP);
                    }

                    // If we reached very near to B due to cumulative steps, add B and exit loop
                    if (segLen - s <= 0.5) 
                    {
                        out.push_back(B);
                        break;
                    }
                }
                
                // ensure B is present at end of segment
                if (Geo::distance_m(out.back(), B) > 0.5) out.push_back(B);
                // continue to next segment; note next segment will re-add B as its A if not filtered out
            }

            // Guarantee last waypoint included
            const LatLon &lastWp = waypoints.back();
            if (out.empty() || Geo::distance_m(out.back(), lastWp) > 0.5) out.push_back(lastWp);

            // Optionally: remove near-duplicate consecutive points (safety cleanup)
            vector<LatLon> cleaned;
            for (const auto &p : out) 
            {
                if (cleaned.empty() || Geo::distance_m(cleaned.back(), p) > 0.5) cleaned.push_back(p);
            }

            return cleaned;
        }
    };

}

// ***************************************************************************************************

/***************************************************************************************************
* Function:       main
* Description:    Entry point and demonstration of the PathSmoother. It defines a set of
* waypoints for a flight path and uses the generatePath method to create a
* smoothed version, printing the resulting coordinates to the console.
***************************************************************************************************/


/*
using namespace Utils;

int main() 
{
    ios::fmtflags f = cout.flags();
    cout.setf(std::ios::fixed);
    cout << setprecision(6);

    // Example waypoints: start -> mid -> destination
    vector<LatLon> waypoints = {
        LatLon(28.6139, 77.2090), // New Delhi (start)
        LatLon(25.5941, 85.1376), // Patna (reference)
        LatLon(19.0760, 72.8777)  // Mumbai (destination)
    };

    
PathSmoother::Params p;
    // Defaults already: near 50km => 50km spacing, mid 250km => 250km spacing, far 500km => 500km spacing.
    // If you prefer denser sampling near endpoints, reduce near_spacing_km (e.g., 10 km).
    // e.g.: p.near_spacing_km = 10.0;

    auto path = 
PathSmoother::generatePath(waypoints, p);

    cout << "Generated path points: " << path.size() << "\n";
    for (size_t i = 0; i < path.size(); ++i) 
    {
        cout << i+1 << ": " << path[i].lat << ", " << path[i].lon << "\n";
    }

    cout.flags(f);
    return 0;
}

*/

#endif // M_PI guard