

---

# Code Documentation

This document outlines the classes, structs, and key functions provided in the Algo_template directory.

## `humidity.h`

**File Description:** This file provides a comprehensive implementation for flight safety analysis related to weather, specifically humidity and dew point. It includes utility functions for geographic and thermodynamic calculations. The main class, `WeatherFlightSafetyEnhanced`, recommends safe operational changes (altitude or horizontal position) based on current atmospheric conditions to avoid icing.

### namespace `Utils`

#### struct `LatLon`
A simple structure to represent a geographic coordinate.
* `double lat`: Latitude in degrees.
* `double lon`: Longitude in degrees.

#### struct `GeoUtils`
A utility struct containing static methods for geographic calculations.
* `static double toRad(double d)`: Converts degrees to radians.
* `static double toDeg(double r)`: Converts radians to degrees.
* `static double distance_m(const LatLon &a, const LatLon &b)`: Calculates Haversine distance in meters.
* `static double bearing_deg(const LatLon &from, const LatLon &to)`: Calculates initial bearing in degrees.

#### Thermodynamic & Humidity Helper Functions
A collection of free-standing functions for calculating atmospheric properties.

* `inline double saturationVaporPressure_hPa(double tempC)`: Calculates saturation vapor pressure from temperature.
* `inline double hPa_to_Pa(double h)`: Converts hectopascals to pascals.
* `inline double Pa_to_hPa(double p)`: Converts pascals to hectopascals.
* `inline double pressureAtAltitude_Pa(double altitude_m)`: Estimates atmospheric pressure at a given altitude.
* `inline double vaporPressureFromRH_Pa(double tempC, double rh_percent)`: Calculates vapor pressure from temperature and relative humidity.
* `inline double specificHumidity_from_T_RH_p(double tempC, double rh_percent, double p_Pa)`: Calculates specific humidity.
* `inline double RH_from_T_q_p_percent(double tempC, double q, double p_Pa)`: Calculates relative humidity from specific humidity.
* `inline double dewPointC_from_e_Pa(double e_Pa)`: Calculates dew point from vapor pressure.
* `inline double dewPointC_from_T_RH(double tempC, double rh_percent)`: Calculates dew point from temperature and relative humidity.

#### struct `NearbySample`
Represents a weather data point at a specific geographic location.
* `LatLon loc`: The geographic location.
* `double rh_percent`: Measured relative humidity (0-100).

#### struct `SafetyResult`
A structure to hold the results of a safety analysis.
* `bool altitude_found`: True if a safe altitude was found.
* `double safe_altitude_m`: The recommended safe altitude.
* `bool horizontal_found`: True if a safe horizontal position was found.
* `double distance_km`: Recommended horizontal distance to travel.
* `double bearing_deg`: Bearing for horizontal travel.
* `double final_margin_c`: The T-Td margin at the recommended position.
* `string note`: An explanatory note.

#### class `WeatherFlightSafetyEnhanced`
The main class for performing weather safety analysis.

##### `static SafetyResult recommend(...)`
Analyzes current conditions to find a safer flight level by searching vertically. If unsuccessful, it provides a heuristic horizontal estimate.

##### `static SafetyResult recommend_with_location(...)`
An overload of `recommend` that takes the aircraft's current `LatLon`. This enables a more accurate horizontal search using the provided `samples` list.

##### `static SafetyResult recommend_estimatedT(...)`
A convenience overload for when measured ambient temperature is not available. It estimates the temperature at the current altitude before calling `recommend_with_location` or `recommend`.

---

## `coordinator.h`

**File Description:** This file provides a set of geographic utility functions encapsulated within the `GeoUtils` class. These functions perform common geodetic calculations on a spherical Earth model.

### namespace `Utils`

#### struct `LatLon`
A simple structure to represent a geographic coordinate.
* `double lat`: Latitude in degrees.
* `double lon`: Longitude in degrees.

#### class `GeoUtils`
A utility class containing static methods for geographic calculations.

##### `static double toRad(double deg)`
Converts angles from degrees to radians.

##### `static double toDeg(double rad)`
Converts angles from radians to degrees.

##### `static double distance(const LatLon &a, const LatLon &b)`
Calculates the great-circle distance (Haversine formula) between two coordinates in meters.

##### `static double bearing(const LatLon &from, const LatLon &to)`
Calculates the initial bearing (forward azimuth) from a starting point to an ending point in degrees (0-360).

##### `static LatLon destinationPoint(const LatLon &from, double bearing_deg, double distance_m)`
Calculates the destination coordinate after traveling a specific distance along a constant bearing.

##### `static double angleBetween(const LatLon &A, const LatLon &B, const LatLon &C)`
Calculates the interior angle in degrees [0, 180] formed by the path A-B-C at point B.

##### `static pair<LatLon, double> selectCoordinate(const LatLon &fixed, const LatLon &dest, double angle_deg)`
Finds a coordinate 'E' such that the angle E-fixed-dest is approximately equal to `angle_deg`.

---

## `path_perceptor.h`

**File Description:** This file contains the implementation for the `PathSmoother` class, which generates a smooth, adaptively sampled flight path from a given set of waypoints.

### namespace `Utils`

#### struct `Geo`
A utility struct containing static methods for geographic calculations.

##### `static double distance_m(const LatLon &a, const LatLon &b)`
Calculates the great-circle distance (Haversine formula) between two coordinates in meters.

##### `static double bearing_deg(const LatLon &from, const LatLon &to)`
Calculates the initial bearing (forward azimuth) from a starting point to an ending point in degrees (0-360).

##### `static LatLon destinationPoint(const LatLon &from, double bearing_deg, double distance_m)`
Calculates the destination coordinate after traveling a specific distance along a constant bearing.

#### class `PathSmoother`
The main class for generating a smoothed flight path.

##### struct `Params`
A configuration struct for the path generation algorithm, defining spacing for "near", "mid", and "far" zones.
* `double near_radius_km`
* `double near_spacing_km`
* `double mid_radius_km`
* `double mid_spacing_km`
* `double far_spacing_km`
* `double min_spacing_km`

##### `static vector<LatLon> generatePath(const vector<LatLon> &waypoints, const Params &params)`
Takes a list of waypoints and generates a smoothed, high-resolution path, sampled more densely near the global start and end points.

---

## `ascent_and_descent_perceptor.h`

**File Description:** This file implements the `FlightProfile` class, which generates a simplified averaged flight profile (climb, cruise, descent) and can calculate altitude/speed at any given distance along the path.

### namespace `FlightProfile`

#### struct `ProfilePoint`
A data structure to hold the state of an aircraft at a specific point.
* `double altitude_m`: Altitude in meters.
* `double horizontal_speed_m_s`: Horizontal ground speed in m/s.
* `double vertical_speed_m_s`: Vertical speed in m/s.
* `string phase`: Current flight phase ("climb", "cruise", etc.).

#### class `FlightProfile`
Models a flight's vertical profile based on performance parameters.

##### `FlightProfile(...)` (Constructor)
Initializes the profile with default parameters (cruise altitude, climb/descent rates, speeds).

##### `void setTotalDistance(double total_m)`
Sets the total distance of the flight route, which is required for accurate cruise/descent calculations.

##### `void setCruiseAltitude(double m)`
Setter method for cruise altitude.

##### `void setClimbParams(double vz_m_s, double v_climb_m_s)`
Setter method for climb rate and speed.

##### `void setDescentParams(double vz_m_s, double v_desc_m_s)`
Setter method for descent rate and speed.

##### `void setCruiseSpeed(double v_cruise_m_s)`
Setter method for cruise speed.

##### `ProfilePoint profileAtDistanceFromStart(double s_from_start_m) const`
Calculates the flight profile state (altitude, speed, phase) at a specific distance from the start.

##### `ProfilePoint profileAtDistanceFromDestination(double d_from_dest_m) const`
Calculates the flight profile state at a specific distance from the destination.

##### `vector<pair<double, ProfilePoint>> sampleAlongRoute(int samples) const`
Generates a series of profile points at equally spaced intervals along the entire route.

##### `double climbDistance() const`, `double descentDistance() const`, `double peakAltitudeForTotalRoute() const`
Public getters for profile diagnostics.

---

## `ApproachDetector.cpp`

**File Description:** This file defines the `ApproachDetector` class, which simulates and detects potential approaches between a flight path and a migratory bird path using dynamic 4D trajectories.

#### struct `PathPoint4D`
A simple struct to hold a complete 4D point in a trajectory.
* `double time_s`: Time in seconds from the start.
* `Utils::LatLon pos`: Latitude and Longitude.
* `double altitude_m`: Altitude in meters.
* `double speed_ms`: Horizontal speed in m/s.

#### class `ApproachDetector`
The main class for running the approach simulation.

##### `ApproachDetector(double range_m, double alt_buffer_m, double angle_deg)` (Constructor)
Initializes the detector with detection parameters:
* `range_m`: The horizontal detection range in meters.
* `alt_buffer_m`: The vertical altitude buffer in meters.
* `angle_deg`: The detection angle (relative to flight path) in degrees.

##### `void initializeFlightPath(const vector<Utils::LatLon>& waypoints)`
Generates a complete 4D trajectory for the flight using `PathSmoother` and `FlightProfile` based on a set of waypoints.

##### `void initializeBirdPath(const vector<Utils::LatLon>& waypoints, double avg_speed_ms, double cruise_alt_m)`
Generates a complete 4D trajectory for the bird path using `PathSmoother` and simple speed/altitude parameters.

##### `void runSimulation(double time_step_s)`
Runs the simulation from T=0 until the end of the trajectories, checking for approach criteria at each `time_step_s`.

---

## `open_weather.h`

**File Description:** This program fetches current weather data from the OpenWeatherMap API using libcurl for HTTP requests and jsoncpp for parsing.

### namespace `Utils`

#### `size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp)`
A C-style callback function for libcurl to handle incoming HTTP response data by appending it to a `std::string`.

#### class `WeatherFetcher`
A class to interface with the OpenWeatherMap API.

##### `WeatherFetcher(const string& key)` (Constructor)
Initializes the fetcher with the necessary OpenWeatherMap API key.

##### `void fetchWeatherData(const string& cityName)`
Fetches and displays weather data for a given city name.

##### `void fetchWeatherData(double lat, double lon)`
Fetches and displays weather data for a given set of geographic coordinates.

##### `void fetchWeatherData(int pinCode)`
Fetches and displays weather data for a given postal code.

---

## `graph_map.h`

**File Description:** This file contains the implementation of a `KeyedMatrix` class, which provides a wrapper around an `Eigen::MatrixXd` to allow element access via string-based keys.

### namespace `Matrix`

#### class `KeyedMatrix`
Manages a matrix where elements are mapped to and accessed via string keys.

##### `KeyedMatrix(int rows, int cols)` (Constructor)
Initializes a zero-matrix with the given number of rows and columns.

##### `void addKey(const string& key, int row, int col)`
Adds or updates a mapping between a string `key` and a specific (`row`, `col`) index in the matrix.

##### `void setValue(const string& key, double value)`
Sets the value of a matrix element identified by its string `key`.

##### `double getValue(const string& key) const`
Gets the value of a matrix element by its string `key`. Returns 0.0 if the key is not found.

##### `void resize(int newRows, int newCols)`
Resizes the underlying matrix, preserving existing data where possible and removing any keys that now point to out-of-bounds indices.

##### `void printMatrix() const`
Prints the entire underlying Eigen matrix to the standard output.