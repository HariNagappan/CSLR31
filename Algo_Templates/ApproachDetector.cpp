#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <stdexcept>

// Assuming these header files are in the include path
#include "path_perceptor.h"
#include "ascent_and _descent_perceptor.h"
#include "Coordinator.h"

// Use the namespaces from your header files
using namespace Utils;
using namespace FlightProfile;

// A simple struct to hold a complete 4D point in a trajectory
struct PathPoint4D
{
    double time_s;      // Time in seconds from the start
    LatLon pos;         // Latitude and Longitude
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
    std::vector<PathPoint4D> flight_trajectory;
    std::vector<PathPoint4D> bird_trajectory;

    // Detection Parameters
    double detection_range_m;
    double altitude_buffer_m;
    double detection_angle_deg;

    /***********************************************************************************************
    * Function:       getInterpolatedState (private)
    * Description:    Calculates the exact state (position, altitude) of an entity at a given time
    * by linearly interpolating between two points in its 4D trajectory.
    ***********************************************************************************************/
    PathPoint4D getInterpolatedState(const std::vector<PathPoint4D>& trajectory, double current_time_s) const
    {
        // Find the two points in the trajectory that bracket the current time
        for (size_t i = 0; i + 1 < trajectory.size(); ++i)
        {
            const auto& p1 = trajectory[i];
            const auto& p2 = trajectory[i+1];

            if (current_time_s >= p1.time_s && current_time_s <= p2.time_s)
            {
                // Calculate the interpolation factor (0.0 to 1.0)
                double time_diff = p2.time_s - p1.time_s;
                if (time_diff < 1e-6) return p1; // Avoid division by zero
                double factor = (current_time_s - p1.time_s) / time_diff;

                // Linearly interpolate the values
                PathPoint4D interpolated_state;
                interpolated_state.time_s = current_time_s;
                interpolated_state.pos.lat = p1.pos.lat + factor * (p2.pos.lat - p1.pos.lat);
                interpolated_state.pos.lon = p1.pos.lon + factor * (p2.pos.lon - p1.pos.lon);
                interpolated_state.altitude_m = p1.altitude_m + factor * (p2.altitude_m - p1.altitude_m);
                interpolated_state.speed_ms = p1.speed_ms + factor * (p2.speed_ms - p1.speed_ms);

                return interpolated_state;
            }
        }
        // If time is beyond the trajectory, return the last known state
        return trajectory.back();
    }


    /***********************************************************************************************
    * Function:       checkApproachAtTime (private)
    * Description:    Executes the 3-step detection algorithm for a single moment in time.
    ***********************************************************************************************/
    void checkApproachAtTime(double current_time_s)
    {
        // 1. Get the interpolated state for both flight and birds at the current time
        PathPoint4D flight_state = getInterpolatedState(flight_trajectory, current_time_s);
        PathPoint4D bird_state = getInterpolatedState(bird_trajectory, current_time_s);

        // 2. Perform the 3-step approach check
        // STEP A: Altitude Check
        if (std::abs(flight_state.altitude_m - bird_state.altitude_m) > this->altitude_buffer_m)
        {
            return; // Altitudes are too different, no risk
        }

        // STEP B: Proximity Check
        double ground_distance_m = GeoUtils::distance(flight_state.pos, bird_state.pos);
        if (ground_distance_m > this->detection_range_m)
        {
            return; // They are too far apart, no risk
        }

        // STEP C: Angular Check
        double flight_bearing = GeoUtils::bearing(flight_state.pos, getInterpolatedState(flight_trajectory, current_time_s + 1.0).pos);
        double bearing_to_bird = GeoUtils::bearing(flight_state.pos, bird_state.pos);
        double angle_diff = std::abs(flight_bearing - bearing_to_bird);
        if (angle_diff > 180.0) angle_diff = 360.0 - angle_diff; // Handle wraparound

        if (angle_diff > this->detection_angle_deg)
        {
            return; // Birds are not within the flight's forward cone
        }

        // If all three checks pass, an approach is detected!
        std::cout << "[!] APPROACH DETECTED at Time: " << static_cast<int>(current_time_s / 60) << " mins\n"
                  << "    - Distance: " << std::fixed << std::setprecision(2) << ground_distance_m / 1000.0 << " km\n"
                  << "    - Flight Alt: " << static_cast<int>(flight_state.altitude_m) << " m | Bird Alt: " << static_cast<int>(bird_state.altitude_m) << " m\n"
                  << "    - Angle: " << angle_diff << " degrees\n\n";
    }


public:
    /***********************************************************************************************
    * Function:       ApproachDetector (Constructor)
    * Description:    Initializes the detector with the required safety parameters.
    ***********************************************************************************************/
    ApproachDetector(double range_m, double alt_buffer_m, double angle_deg)
        : detection_range_m(range_m), altitude_buffer_m(alt_buffer_m), detection_angle_deg(angle_deg)
    {
        if (range_m <= 0 || alt_buffer_m <= 0 || angle_deg <= 0)
        {
            throw std::invalid_argument("Detection parameters must be positive.");
        }
        std::cout << "Approach Detector Initialized.\n"
                  << " - Detection Range: " << range_m / 1000.0 << " km\n"
                  << " - Altitude Buffer: " << alt_buffer_m << " m\n"
                  << " - Detection Angle: " << angle_deg << " degrees\n\n";
    }

    /***********************************************************************************************
    * Function:       initializeFlightPath
    * Description:    Generates the flight's 4D trajectory using the provided waypoints and
    * the FlightProfile model.
    ***********************************************************************************************/
    void initializeFlightPath(const std::vector<LatLon>& waypoints)
    {
        // Use PathSmoother to create a high-resolution 2D path
        auto smoothed_path = PathSmoother::generatePath(waypoints);
        
        // Calculate total distance for the FlightProfile
        double total_distance_m = 0;
        for (size_t i = 0; i + 1 < smoothed_path.size(); ++i) {
            total_distance_m += GeoUtils::distance(smoothed_path[i], smoothed_path[i+1]);
        }

        // Configure the FlightProfile
        FlightProfile profile;
        profile.setTotalDistance(total_distance_m);

        // Generate the 4D trajectory
        double current_time_s = 0;
        double distance_traveled_m = 0;
        flight_trajectory.clear();

        for (size_t i = 0; i + 1 < smoothed_path.size(); ++i) {
            auto p_start = profile.profileAtDistanceFromStart(distance_traveled_m);
            flight_trajectory.push_back({current_time_s, smoothed_path[i], p_start.altitude_m, p_start.horizontal_speed_m_s});
            
            double segment_dist = GeoUtils::distance(smoothed_path[i], smoothed_path[i+1]);
            double avg_speed_in_segment = (p_start.horizontal_speed_m_s + profile.profileAtDistanceFromStart(distance_traveled_m + segment_dist).horizontal_speed_m_s) / 2.0;
            
            if (avg_speed_in_segment < 1.0) avg_speed_in_segment = 1.0;
            current_time_s += segment_dist / avg_speed_in_segment;
            distance_traveled_m += segment_dist;
        }
        flight_trajectory.push_back({current_time_s, smoothed_path.back(), 0.0, 0.0}); // Add final point
        std::cout << "Flight path initialized. Total distance: " << total_distance_m / 1000.0 << " km. Estimated time: " << current_time_s / 3600.0 << " hours.\n";
    }

    /***********************************************************************************************
    * Function:       initializeBirdPath
    * Description:    Generates the bird flock's 4D trajectory using a trapezoidal speed profile.
    ***********************************************************************************************/
    void initializeBirdPath(const std::vector<LatLon>& waypoints, double avg_speed_ms, double cruise_alt_m)
    {
        auto smoothed_path = PathSmoother::generatePath(waypoints);
        
        double total_distance_m = 0;
        for (size_t i = 0; i + 1 < smoothed_path.size(); ++i) {
            total_distance_m += GeoUtils::distance(smoothed_path[i], smoothed_path[i+1]);
        }

        // Generate 4D trajectory with a simple trapezoidal speed model (10% accel, 80% cruise, 10% decel)
        double current_time_s = 0;
        double distance_traveled_m = 0;
        bird_trajectory.clear();
        
        for (const auto& point : smoothed_path) {
            double current_speed_ms = avg_speed_ms;
            double dist_frac = distance_traveled_m / total_distance_m;

            if (dist_frac < 0.1) current_speed_ms = avg_speed_ms * (dist_frac / 0.1); // Ramp up
            else if (dist_frac > 0.9) current_speed_ms = avg_speed_ms * ((1.0 - dist_frac) / 0.1); // Ramp down
            
            if (current_speed_ms < 1.0) current_speed_ms = 1.0;

            bird_trajectory.push_back({current_time_s, point, cruise_alt_m, current_speed_ms});

            // This is a simplified time calculation; for more accuracy, this loop would be more complex
            if (&point != &smoothed_path.back()) {
                double segment_dist = GeoUtils::distance(point, *(&point + 1));
                current_time_s += segment_dist / current_speed_ms;
                distance_traveled_m += segment_dist;
            }
        }
        std::cout << "Bird path initialized. Total distance: " << total_distance_m / 1000.0 << " km. Estimated time: " << current_time_s / 3600.0 << " hours.\n\n";
    }


    /***********************************************************************************************
    * Function:       runSimulation
    * Description:    The main simulation loop. It iterates through time and calls the
    * detection logic at each step.
    ***********************************************************************************************/
    void runSimulation(double time_step_s)
    {
        if (flight_trajectory.empty() || bird_trajectory.empty())
        {
            throw std::logic_error("Cannot run simulation before initializing paths.");
        }

        std::cout << "--- Starting Simulation (Time Step: " << time_step_s << "s) ---\n\n";
        
        double end_time_s = std::max(flight_trajectory.back().time_s, bird_trajectory.back().time_s);

        for (double t = 0; t <= end_time_s; t += time_step_s)
        {
            // The core detection logic is called for each time step
            checkApproachAtTime(t);
        }

        std::cout << "--- Simulation Finished ---\n";
    }
};

// ***************************************************************************************************
//                                        MAIN FUNCTION
// ***************************************************************************************************
int main()
{
    try
    {
        // 1. Define Paths
        // Flight path from Delhi to Mumbai (approximate)
        std::vector<LatLon> flight_wps = {
            {28.5665, 77.1031}, // Delhi Airport
            {24.0, 75.0},       // Waypoint
            {19.0896, 72.8656}  // Mumbai Airport
        };

        // Bird migration path (e.g., Central Asia to Western India, crossing the flight path)
        std::vector<LatLon> bird_wps = {
            {34.5553, 69.2075}, // Kabul, AF
            {26.5, 73.0},       // Intersecting region
            {22.3039, 70.8022}  // Rajkot, IN
        };

        // 2. Setup the Detector
        // Detect if paths come within 40km, with a 500m altitude buffer, and within a 30-degree cone
        ApproachDetector detector(40000.0, 500.0, 30.0);

        // 3. Initialize Trajectories
        detector.initializeFlightPath(flight_wps);
        
        // Bar-tailed Godwit stats: avg speed ~60 km/h (16.7 m/s), altitude ~2000m
        detector.initializeBirdPath(bird_wps, 16.7, 2000.0);

        // 4. Run the Simulation
        // Check for an approach every 60 seconds
        detector.runSimulation(60.0);
    }
    catch (const std::exception& e)
    {
        // Catch any errors during setup or simulation
        std::cerr << "An error occurred: " << e.what() << '\n';
        return 1;
    }
    return 0;
}