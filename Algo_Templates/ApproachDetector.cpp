#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <stdexcept>
#include <iomanip> // Required for std::fixed and std::setprecision

// Corrected include order
#include "Coordinator.h"
#include "path_perceptor.h"
#include "ascent_and_descent_perceptor.h"

// Use all standard library names without the std:: prefix
using namespace std;

// Use the namespaces from your header files
using namespace Utils;
using namespace FlightProfile;

// A simple struct to hold a complete 4D point in a trajectory
struct PathPoint4D
{
    double time_s;      // Time in seconds from the start
    Utils::LatLon pos;  // Latitude and Longitude
    double altitude_m;  // Altitude in meters
    double speed_ms;    // Horizontal speed in m/s
};


/***************************************************************************************************
* Class:          ApproachDetector
* Description:    A class to simulate and detect potential approaches between a flight path
* and a migratory bird path using dynamic 4D trajectories.
***************************************************************************************************/
class ApproachDetector
{
private:
    // Trajectories
    vector<PathPoint4D> flight_trajectory;
    vector<PathPoint4D> bird_trajectory;

    // Detection Parameters
    double detection_range_m;
    double altitude_buffer_m;
    double detection_angle_deg;

    /***********************************************************************************************
    * Function:       getInterpolatedState (private)
    ***********************************************************************************************/
    PathPoint4D getInterpolatedState(const vector<PathPoint4D>& trajectory, double current_time_s) const
    {
        if (trajectory.empty()) {
            throw runtime_error("Cannot interpolate on an empty trajectory.");
        }
        for (size_t i = 0; i + 1 < trajectory.size(); ++i)
        {
            const auto& p1 = trajectory[i];
            const auto& p2 = trajectory[i+1];

            if (current_time_s >= p1.time_s && current_time_s <= p2.time_s)
            {
                double time_diff = p2.time_s - p1.time_s;
                if (time_diff < 1e-6) return p1;
                double factor = (current_time_s - p1.time_s) / time_diff;

                PathPoint4D interpolated_state;
                interpolated_state.time_s = current_time_s;
                interpolated_state.pos.lat = p1.pos.lat + factor * (p2.pos.lat - p1.pos.lat);
                interpolated_state.pos.lon = p1.pos.lon + factor * (p2.pos.lon - p1.pos.lon);
                interpolated_state.altitude_m = p1.altitude_m + factor * (p2.altitude_m - p1.altitude_m);
                interpolated_state.speed_ms = p1.speed_ms + factor * (p2.speed_ms - p1.speed_ms);

                return interpolated_state;
            }
        }
        return trajectory.back();
    }


    /***********************************************************************************************
    * Function:       checkApproachAtTime (private)
    ***********************************************************************************************/
    void checkApproachAtTime(double current_time_s)
    {
        PathPoint4D flight_state = getInterpolatedState(flight_trajectory, current_time_s);
        PathPoint4D bird_state = getInterpolatedState(bird_trajectory, current_time_s);

        if (abs(flight_state.altitude_m - bird_state.altitude_m) > this->altitude_buffer_m)
        {
            return;
        }

        double ground_distance_m = Utils::GeoUtils::distance(flight_state.pos, bird_state.pos);
        if (ground_distance_m > this->detection_range_m)
        {
            return;
        }

        double flight_bearing = Utils::GeoUtils::bearing(flight_state.pos, getInterpolatedState(flight_trajectory, current_time_s + 1.0).pos);
        double bearing_to_bird = Utils::GeoUtils::bearing(flight_state.pos, bird_state.pos);
        double angle_diff = abs(flight_bearing - bearing_to_bird);
        if (angle_diff > 180.0) angle_diff = 360.0 - angle_diff;

        if (angle_diff > this->detection_angle_deg)
        {
            return;
        }

        stringstream approach_log;
        approach_log << "[!] APPROACH DETECTED at Time: " << static_cast<int>(current_time_s / 60) << " mins\n"
                  << "    - Distance: " << fixed << setprecision(2) << ground_distance_m / 1000.0 << " km\n"
                  << "    - Flight Alt: " << static_cast<int>(flight_state.altitude_m) << " m | Bird Alt: " << static_cast<int>(bird_state.altitude_m) << " m\n"
                  << "    - Angle: " << angle_diff << " degrees\n\n";
        cout << approach_log.str() << endl;
    }


public:
    /***********************************************************************************************
    * Function:       ApproachDetector (Constructor)
    ***********************************************************************************************/
    ApproachDetector(double range_m, double alt_buffer_m, double angle_deg)
        : detection_range_m(range_m), altitude_buffer_m(alt_buffer_m), detection_angle_deg(angle_deg)
    {
        if (range_m <= 0 || alt_buffer_m <= 0 || angle_deg <= 0)
        {
            throw invalid_argument("Detection parameters must be positive.");
        }
        cout << "Approach Detector Initialized.\n"
                  << " - Detection Range: " << range_m / 1000.0 << " km\n"
                  << " - Altitude Buffer: " << alt_buffer_m << " m\n"
                  << " - Detection Angle: " << angle_deg << " degrees\n\n";
    }

    /***********************************************************************************************
    * Function:       initializeFlightPath
    ***********************************************************************************************/
    void initializeFlightPath(const vector<Utils::LatLon>& waypoints)
    {
        // Explicitly create default Params object for the call
        auto smoothed_path = Utils::PathSmoother::generatePath(waypoints, Utils::PathSmoother::Params());
        
        double total_distance_m = 0;
        for (size_t i = 0; i + 1 < smoothed_path.size(); ++i) {
            total_distance_m += Utils::GeoUtils::distance(smoothed_path[i], smoothed_path[i+1]);
        }

        FlightProfile::FlightProfile profile;
        profile.setTotalDistance(total_distance_m);

        double current_time_s = 0;
        double distance_traveled_m = 0;
        flight_trajectory.clear();

        for (size_t i = 0; i + 1 < smoothed_path.size(); ++i) {
            auto p_start = profile.profileAtDistanceFromStart(distance_traveled_m);
            flight_trajectory.push_back(PathPoint4D{current_time_s, smoothed_path[i], p_start.altitude_m, p_start.horizontal_speed_m_s});
            
            double segment_dist = Utils::GeoUtils::distance(smoothed_path[i], smoothed_path[i+1]);
            double avg_speed_in_segment = (p_start.horizontal_speed_m_s + profile.profileAtDistanceFromStart(distance_traveled_m + segment_dist).horizontal_speed_m_s) / 2.0;
            
            if (avg_speed_in_segment < 1.0) avg_speed_in_segment = 1.0;
            current_time_s += segment_dist / avg_speed_in_segment;
            distance_traveled_m += segment_dist;
        }
        flight_trajectory.push_back(PathPoint4D{current_time_s, smoothed_path.back(), 0.0, 0.0});
        cout << "Flight path initialized. Total distance: " << total_distance_m / 1000.0 << " km. Estimated time: " << current_time_s / 3600.0 << " hours.\n";
    }

    /***********************************************************************************************
    * Function:       initializeBirdPath
    ***********************************************************************************************/
    void initializeBirdPath(const vector<Utils::LatLon>& waypoints, double avg_speed_ms, double cruise_alt_m)
    {
        // Explicitly create default Params object for the call
        auto smoothed_path = Utils::PathSmoother::generatePath(waypoints, Utils::PathSmoother::Params());
        
        double total_distance_m = 0;
        for (size_t i = 0; i + 1 < smoothed_path.size(); ++i) {
            total_distance_m += Utils::GeoUtils::distance(smoothed_path[i], smoothed_path[i+1]);
        }

        double current_time_s = 0;
        double distance_traveled_m = 0;
        bird_trajectory.clear();
        
        for (size_t i = 0; i < smoothed_path.size(); ++i) {
            const auto& point = smoothed_path[i];
            double current_speed_ms = avg_speed_ms;
            double dist_frac = (total_distance_m > 0) ? (distance_traveled_m / total_distance_m) : 0;

            if (dist_frac < 0.1) current_speed_ms = avg_speed_ms * (dist_frac / 0.1);
            else if (dist_frac > 0.9) current_speed_ms = avg_speed_ms * ((1.0 - dist_frac) / 0.1);
            
            if (current_speed_ms < 1.0) current_speed_ms = 1.0;

            bird_trajectory.push_back(PathPoint4D{current_time_s, point, cruise_alt_m, current_speed_ms});

            if (i + 1 < smoothed_path.size()) {
                double segment_dist = Utils::GeoUtils::distance(point, smoothed_path[i+1]);
                current_time_s += segment_dist / current_speed_ms;
                distance_traveled_m += segment_dist;
            }
        }
        cout << "Bird path initialized. Total distance: " << total_distance_m / 1000.0 << " km. Estimated time: " << current_time_s / 3600.0 << " hours.\n\n";
    }

    /***********************************************************************************************
    * Function:       runSimulation
    ***********************************************************************************************/
    void runSimulation(double time_step_s)
    {
        if (flight_trajectory.empty() || bird_trajectory.empty())
        {
            throw logic_error("Cannot run simulation before initializing paths.");
        }

        cout << "--- Starting Simulation (Time Step: " << time_step_s << "s) ---\n\n";
        
        double end_time_s = max(flight_trajectory.back().time_s, bird_trajectory.back().time_s);

        for (double t = 0; t <= end_time_s; t += time_step_s)
        {
            checkApproachAtTime(t);
        }

        cout << "--- Simulation Finished ---\n";
    }
};

// ***************************************************************************************************
//                                        MAIN FUNCTION
// ***************************************************************************************************
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