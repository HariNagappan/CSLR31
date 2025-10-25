#include "coordinator.h"
#include "path_perceptor.h"
#include "ascent_and_descent_perceptor.h"
#include "graph_map.h"
#include <bits/stdc++.h>
using namespace std;

int main()
{
    try
    {
        vector<Utils::LatLon> flight_wps = {
            {28.5665, 77.1031}, // Delhi Airport
            {24.0, 75.0},       // Waypoint
            {19.0896, 72.8656}  // Mumbai Airport
        };

        vector<Utils::LatLon> bird_wps = {
            {34.5553, 69.2075}, // Kabul, AF
            {26.5, 73.0},       // Intersecting region
            {22.3039, 70.8022}  // Rajkot, IN
        };

        ApproachDetector detector(40000.0, 500.0, 30.0);
        detector.initializeFlightPath(flight_wps);
        detector.initializeBirdPath(bird_wps, 16.7, 2000.0);
        detector.runSimulation(60.0);
    }
    catch (const exception& e)
    {
        cerr << "An error occurred: " << e.what() << '\n';
        return 1;
    }
    return 0;
}