

#include "path_perceptor.h"
                           
#include <bits/stdc++.h>   

using namespace std;
using namespace Utils;

/***************************************************************************************************

// PathSmoother Test
int main()
{
    
    cout << fixed << setprecision(6);

    // 1. Define the base waypoints for the flight
    cout << "Base waypoints:" << endl;
    vector<Utils::LatLon> flight_wps = {
    {40.6413, -73.7781}, // New York JFK Airport
    {51.4700, -0.4543},  // London Heathrow Airport (waypoint)
    {22.3080, 113.9185}  // Hong Kong International Airport
    };

    for (size_t i = 0; i < flight_wps.size(); ++i) 
    {
        cout << "  - WP " << i + 1 << ": " << flight_wps[i].lat << ", " << flight_wps[i].lon << endl;
    }

    cout << "------------------------------------------\n" << endl;

    // 2. Define the high-density parameters (reduced spacing)
    Utils::PathSmoother::Params highDensityParams;

    cout << "Generating high-density path..." << endl;
    cout << "(Using spacing: "
         << "Near=" << highDensityParams.near_spacing_km << "km, "
         << "Mid=" << highDensityParams.mid_spacing_km << "km, "
         << "Far=" << highDensityParams.far_spacing_km << "km)" << endl;

    // 3. Call the generatePath function
    vector<Utils::LatLon> smoothed_path = Utils::PathSmoother::generatePath(flight_wps, highDensityParams);

    // 4. Print the resulting list of generated waypoints
    cout << "\n--- Path Generation Complete ---" << endl;
    cout << "Total intermediate waypoints generated: " << smoothed_path.size() << endl;
    cout << "------------------------------------------" << endl;

    for (size_t i = 0; i < smoothed_path.size(); ++i) 
    {
        cout << "  Point " << setw(3) << (i + 1) << ": \t"
             << smoothed_path[i].lat << ", \t"
             << smoothed_path[i].lon << endl;
    }

    return 0;
}

*******************************************************************************************/