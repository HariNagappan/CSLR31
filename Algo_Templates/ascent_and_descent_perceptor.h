/***************************************************************************************************
    * File:           ascent_and_descent_perceptor.cpp (Original: flight_profile.cpp)
    * Description:    This file implements the FlightProfile class, which generates a simplified
    * averaged flight profile consisting of climb, cruise, and descent phases.
    * It can calculate the aircraft's altitude and speed at any given distance
    * along the flight path and can handle both long and short routes.
***************************************************************************************************/


#ifndef FLIGHT_PROFILE_CPP // include guard
#define FLIGHT_PROFILE_CPP

#include <bits/stdc++.h>
using namespace std;

namespace FlightProfile 
{

    
    /***************************************************************************************************
    * Struct:         ProfilePoint
    * Description:    A data structure to hold the state of an aircraft at a specific point in its
    * flight profile. It includes altitude, horizontal and vertical speeds, and
    * the current flight phase.
    ***************************************************************************************************/
    struct ProfilePoint 
    {
        double altitude_m;          // altitude above destination/sea-level (m)
        double horizontal_speed_m_s; // ground/track speed (m/s)
        double vertical_speed_m_s;   // positive for climb, negative for descent (m/s)
        string phase;                // "on-ground", "climb", "cruise", or "descent"
    };

    /***************************************************************************************************
    * Class:          FlightProfile
    * Description:    Models a flight's vertical profile based on key performance parameters like
    * climb/descent rates and cruise altitude. It can determine the flight phase
    * and state for any point along a given route distance.
    ***************************************************************************************************/
    class FlightProfile 
    {
    public:
        /***********************************************************************************************
        * Function:       FlightProfile (Constructor)
        * Description:    Initializes the flight profile with typical, default parameters for cruise
        * altitude, climb/descent rates, and speeds. These values can be tuned.
        * Inputs:         cruise_altitude_m (double)  - ~36,000 ft
        * climb_rate_m_s (double)     - vertical climb rate (~1575 ft/min)
        * climb_speed_m_s (double)    - horizontal speed during climb (~292 kt)
        * cruise_speed_m_s (double)   - cruise speed (~447 kt)
        * descent_rate_m_s (double)   - vertical descent magnitude (~1378 ft/min)
        * descent_speed_m_s (double)  - horizontal speed during descent
        * Outputs:        None.
        ***********************************************************************************************/
        FlightProfile(double cruise_altitude_m = 11000.0,
                    double climb_rate_m_s = 8.0,
                    double climb_speed_m_s = 150.0,
                    double cruise_speed_m_s = 230.0,
                    double descent_rate_m_s = 7.0,
                    double descent_speed_m_s = 200.0)
        : H_cruise(cruise_altitude_m),
            Vz_climb(climb_rate_m_s),
            V_climb(climb_speed_m_s),
            V_cruise(cruise_speed_m_s),
            Vz_desc(descent_rate_m_s),
            V_desc(descent_speed_m_s),
            total_distance_m(-1.0)
        {
            recomputeDerived();
        }

        /***********************************************************************************************
        * Function:       setTotalDistance
        * Description:    Sets the total distance of the flight route. This allows the model to
        * accurately calculate the cruise phase length and handle short routes where
        * the aircraft may not reach its planned cruise altitude.
        * Inputs:         total_m - The total route distance in meters.
        * Outputs:        None.
        ***********************************************************************************************/
        void setTotalDistance(double total_m) 
        {
            if (total_m > 0.0) total_distance_m = total_m;
            else total_distance_m = -1.0;
            recomputeDerived();
        }

        /***********************************************************************************************
        * Function:       setCruiseAltitude, setClimbParams, etc.
        * Description:    Setter methods to update primary flight parameters and recompute derived
        * values for the profile.
        * Inputs:         Various doubles representing flight parameters.
        * Outputs:        None.
        ***********************************************************************************************/
        void setCruiseAltitude(double m) { H_cruise = m; recomputeDerived(); }
        
        void setClimbParams(double vz_m_s, double v_climb_m_s) { Vz_climb = vz_m_s; V_climb = v_climb_m_s; recomputeDerived(); }
        
        void setDescentParams(double vz_m_s, double v_desc_m_s) { Vz_desc = vz_m_s; V_desc = v_desc_m_s; recomputeDerived(); }
        
        void setCruiseSpeed(double v_cruise_m_s) { V_cruise = v_cruise_m_s; recomputeDerived(); }

        /***********************************************************************************************
        * Function:       profileAtDistanceFromStart
        * Description:    Calculates the flight profile state at a specific along-track distance
        * from the starting point. It determines the correct phase (climb, cruise,
        * or descent) and returns the corresponding altitude and speeds. Handles
        * both normal long routes and short routes with no cruise phase.
        * Inputs:         s_from_start_m - The distance from the start in meters.
        * Outputs:        A ProfilePoint struct containing the aircraft's state.
        ***********************************************************************************************/
        ProfilePoint profileAtDistanceFromStart(double s_from_start_m) const 
        {
            ProfilePoint p;
            if (s_from_start_m <= 0.0) 
            {
                p.phase = "on-ground";
                p.altitude_m = 0.0;
                p.horizontal_speed_m_s = 0.0;
                p.vertical_speed_m_s = 0.0;
                
                return p;
            }

            if (has_total_distance && total_distance_m < (D_climb + D_desc)) 
            {
                double x_meet = (m2 * total_distance_m) / (m1 + m2);
                double Hpeak = m1 * x_meet;

                if (s_from_start_m <= x_meet) 
                {
                    double altitude = m1 * s_from_start_m;
                    p.altitude_m = altitude;
                    p.horizontal_speed_m_s = V_climb;
                    p.vertical_speed_m_s = Vz_climb;
                    p.phase = "climb";
                    
                    return p;
                } 
                else if (s_from_start_m < total_distance_m - 1e-9) 
                {
                    double d_to_end = total_distance_m - s_from_start_m;
                    double altitude = m2 * d_to_end;
                    p.altitude_m = altitude;
                    p.horizontal_speed_m_s = V_desc;
                    p.vertical_speed_m_s = -Vz_desc;
                    p.phase = "descent";
                    
                    return p;
                } 
                else 
                {
                    p.phase = "on-ground";
                    p.altitude_m = 0.0;
                    p.horizontal_speed_m_s = 0.0;
                    p.vertical_speed_m_s = 0.0;
                    
                    return p;
                }
            }

            if (s_from_start_m < D_climb - 1e-9) 
            {
                double altitude = m1 * s_from_start_m;
                p.altitude_m = min(altitude, H_cruise);
                double frac = (D_climb > 1e-9) ? (s_from_start_m / D_climb) : 1.0;
                p.horizontal_speed_m_s = V_climb + (V_cruise - V_climb) * frac;
                p.vertical_speed_m_s = Vz_climb;
                p.phase = "climb";
                
                return p;
            }

            double descent_start_from_start = has_total_distance ? (total_distance_m - D_desc) : numeric_limits<double>::infinity();

            if (s_from_start_m <= descent_start_from_start - 1e-9) 
            {
                p.phase = "cruise";
                p.altitude_m = H_cruise;
                p.horizontal_speed_m_s = V_cruise;
                p.vertical_speed_m_s = 0.0;
                
                return p;
            }

            if (s_from_start_m < total_distance_m - 1e-9) 
            {
                double d_to_end = total_distance_m - s_from_start_m;
                double altitude = m2 * d_to_end;
                p.altitude_m = min(H_cruise, max(0.0, altitude));
                double distance_into_descent = D_desc - d_to_end;
                double frac = (D_desc > 1e-9) ? (distance_into_descent / D_desc) : 1.0;
                p.horizontal_speed_m_s = V_cruise + (V_desc - V_cruise) * frac;
                p.vertical_speed_m_s = -Vz_desc;
                p.phase = "descent";
                
                return p;
            }

            p.phase = "on-ground";
            p.altitude_m = 0.0;
            p.horizontal_speed_m_s = 0.0;
            p.vertical_speed_m_s = 0.0;
            
            return p;
        }

        /***********************************************************************************************
        * Function:       profileAtDistanceFromDestination
        * Description:    Calculates the flight profile state at a specific distance from the
        * destination. Convenient for queries related to the arrival phase.
        * Inputs:         d_from_dest_m - The distance from the destination in meters.
        * Outputs:        A ProfilePoint struct containing the aircraft's state.
        ***********************************************************************************************/
        ProfilePoint profileAtDistanceFromDestination(double d_from_dest_m) const 
        {
            if (!has_total_distance) 
            {
                ProfilePoint p;
                if (d_from_dest_m <= 0.0) 
                {
                    p.phase = "on-ground"; 
                    p.altitude_m = 0; 
                    p.horizontal_speed_m_s=0; 
                    p.vertical_speed_m_s=0; 
                    
                    return p;
                }
                if (d_from_dest_m <= D_desc) 
                {
                    double altitude = m2 * d_from_dest_m;
                    p.altitude_m = min(altitude, H_cruise);
                    p.horizontal_speed_m_s = V_desc;
                    p.vertical_speed_m_s = -Vz_desc;
                    p.phase = "descent";
                    return p;
                } 
                else 
                {
                    p.phase = "cruise"; 
                    p.altitude_m = H_cruise; 
                    p.horizontal_speed_m_s = V_cruise; 
                    p.vertical_speed_m_s = 0.0;
                    
                    return p;
                }
            } 
            else 
            {
                double s_from_start = total_distance_m - d_from_dest_m;
                return profileAtDistanceFromStart(s_from_start);
            }
        }

        /***********************************************************************************************
        * Function:       sampleAlongRoute
        * Description:    Generates a series of profile points at equally spaced intervals along the
        * entire flight path. Requires total distance to be set.
        * Inputs:         samples - The number of points to sample.
        * Outputs:        A vector of pairs, each containing a distance and its ProfilePoint.
        ***********************************************************************************************/
        vector<pair<double, ProfilePoint>> sampleAlongRoute(int samples) const 
        {
            vector<pair<double, ProfilePoint>> out;
            if (!has_total_distance || samples <= 0) return out;
            
            for (int i = 0; i <= samples; ++i) 
            {
                double s = (double(i) / double(max(1, samples))) * total_distance_m;
                out.push_back({ s, profileAtDistanceFromStart(s) });
            }
            
            return out;
        }

        /***********************************************************************************************
        * Function:       climbDistance, descentDistance, etc.
        * Description:    Public getters to expose some derived diagnostics about the flight profile.
        * Inputs:         None.
        * Outputs:        A double or bool with the requested diagnostic value.
        ***********************************************************************************************/
        double climbDistance() const { return D_climb; }
        double descentDistance() const { return D_desc; }
        bool hasTotalDistance() const { return has_total_distance; }
        double totalDistance() const { return total_distance_m; }
        double peakAltitudeForTotalRoute() const { return has_total_distance && total_distance_m < (D_climb + D_desc) ? H_peak_for_short : H_cruise; }

    private:
        // primary parameters
        double H_cruise;
        double Vz_climb;
        double V_climb;
        double V_cruise;
        double Vz_desc;
        double V_desc;

        // optional route info
        double total_distance_m;
        bool has_total_distance = false;

        // derived
        double m1;
        double m2;
        double D_climb;
        double D_desc;
        double H_peak_for_short;

        /***********************************************************************************************
        * Function:       recomputeDerived (private)
        * Description:    Updates internal derived values (slopes, distances) whenever a primary
        * flight parameter is changed.
        * Inputs:         None.
        * Outputs:        None.
        ***********************************************************************************************/
        void recomputeDerived() 
        {
            m1 = (V_climb > 1e-9) ? (Vz_climb / V_climb) : 0.0;
            m2 = (V_desc  > 1e-9) ? (Vz_desc  / V_desc)  : 0.0;

            D_climb = (m1 > 1e-12) ? (H_cruise / m1) : numeric_limits<double>::infinity();
            D_desc  = (m2 > 1e-12) ? (H_cruise / m2) : numeric_limits<double>::infinity();

            has_total_distance = (total_distance_m > 0.0);

            if (has_total_distance && total_distance_m < (D_climb + D_desc)) 
            {
                double L = total_distance_m;
                double x_meet = (m2 * L) / (m1 + m2);
                H_peak_for_short = m1 * x_meet;
                if (H_peak_for_short > H_cruise) H_peak_for_short = H_cruise;
            } 
            else 
            {
                H_peak_for_short = H_cruise;
            }
        }
    };

}

// **************************************************************************************************

/***************************************************************************************************
* Function:       main
* Description:    Entry point and demonstration of the FlightProfile class. It creates a profile
* for a 2500 km route, queries the profile at various points, and then samples
* the entire route to print a summary.
***************************************************************************************************/
/*
using namespace FlightProfile;

int main() 
{
    ios::fmtflags f = cout.flags();
    cout.setf(std::ios::fixed);
    cout << setprecision(2);

    // Create a typical profile (defaults)
    FlightProfile prof; // defaults: cruise 11000 m, climb_rate 8 m/s, climb_speed 150 m/s, cruise 230 m/s, descent_rate 7 m/s, descent_speed 200 m/s

    // Optionally set a total route length (meters). Example: 2500 km route:
    double route_km = 2500.0;
    prof.setTotalDistance(route_km * 1000.0);

    cout << "Cruise altitude (m): " << 11000.0 << "\n";
    cout << "Climb horizontal distance to cruise (km): " << prof.climbDistance() / 1000.0 << "\n";
    cout << "Descent horizontal distance from cruise (km): " << prof.descentDistance() / 1000.0 << "\n";
    cout << "Peak altitude for route (m): " << prof.peakAltitudeForTotalRoute() << "\n\n";

    // Query: 50 km from start
    double q1_km = 50.0;
    auto p1 = prof.profileAtDistanceFromStart(q1_km * 1000.0);
    cout << q1_km << " km from start -> phase: " << p1.phase
         << ", alt: " << p1.altitude_m << " m, Vh: " << p1.horizontal_speed_m_s << " m/s, Vz: " << p1.vertical_speed_m_s << " m/s\n";

    // Query: 500 km from start
    double q2_km = 500.0;
    auto p2 = prof.profileAtDistanceFromStart(q2_km * 1000.0);
    cout << q2_km << " km from start -> phase: " << p2.phase
         << ", alt: " << p2.altitude_m << " m, Vh: " << p2.horizontal_speed_m_s << " m/s, Vz: " << p2.vertical_speed_m_s << " m/s\n";

    // Query: 200 km from destination
    double q3_km_from_dest = 200.0;
    auto p3 = prof.profileAtDistanceFromDestination(q3_km_from_dest * 1000.0);
    cout << q3_km_from_dest << " km from destination -> phase: " << p3.phase
         << ", alt: " << p3.altitude_m << " m, Vh: " << p3.horizontal_speed_m_s << " m/s, Vz: " << p3.vertical_speed_m_s << " m/s\n";

    // Sample entire route (every 500 km)
    cout << "\nSampling route every 500 km:\n";
    int samples = int((route_km / 500.0) + 0.5);
    if (samples < 1) samples = 1;
    auto samps = prof.sampleAlongRoute(samples);
    for (auto &pr : samps) 
    {
        double s_km = pr.first / 1000.0;
        auto &pp = pr.second;
        cout << s_km << " km: phase=" << pp.phase << ", alt=" << pp.altitude_m << " m, Vh=" << pp.horizontal_speed_m_s << " m/s\n";
    }

    cout.flags(f);
    return 0;
}

*/

#endif // include guard /* FLIGHT_PROFILE_CPP */