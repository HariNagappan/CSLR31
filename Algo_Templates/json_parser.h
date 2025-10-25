#pragma once
#include <string>
#include <vector>
#include "coordinator.h"

using namespace std;
using namespace Utils;

/***************************************
 * Flight waypoint: uses LatLon + altitude
 ***************************************/
typedef struct FlightWaypoint
{
    string id;     // Waypoint ID
    LatLon pos;    // Latitude / Longitude
    double alt_ft; // Altitude in feet
} FlightWaypoint;

/***************************************
 * Flight data
 ***************************************/
typedef struct FlightData
{
    string flight_id;
    vector<FlightWaypoint> waypoints;
    int speed_kts;
    string depart_time; // ISO string
} FlightData;

/***************************************
 * Bird corridor point
 ***************************************/
typedef struct BirdCorridorPoint
{
    string id;
    LatLon pos;
} BirdCorridorPoint;

/***************************************
 * Bird data
 ***************************************/
typedef struct BirdData
{
    string species;
    string season;
    vector<BirdCorridorPoint> corridor;
    double alt_min_ft;
    double alt_max_ft;
    string t_start; // ISO string
    string t_end;   // ISO string
    double lateral_buffer_km;
} BirdData;
