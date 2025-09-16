/***************************************************************************************************
* File:           humidity.cpp (Original: flight_safety_full.cpp)
* Description:    This file provides a comprehensive implementation for flight safety analysis
* related to weather, specifically humidity and dew point. It includes utility
* functions for geographic and thermodynamic calculations. The main class,
* WeatherFlightSafetyEnhanced, recommends safe operational changes (altitude or
* horizontal position) based on current atmospheric conditions to avoid icing.
***************************************************************************************************/
#include <bits/stdc++.h>
using namespace std;

/***************************************************************************************************
* Struct:         LatLon
* Description:    A simple structure to represent a geographic coordinate with latitude and
* longitude values in degrees.
***************************************************************************************************/
struct LatLon {
    double lat; // degrees
    double lon; // degrees
    LatLon() : lat(0), lon(0) {}
    LatLon(double la, double lo) : lat(la), lon(lo) {}
};

/***************************************************************************************************
* Struct:         GeoUtils
* Description:    A utility struct containing static methods for geographic calculations based
* on a spherical Earth model, including Haversine distance and bearing.
***************************************************************************************************/
struct GeoUtils {
    static constexpr double R = 6371000.0; // mean earth radius (m)
    static double toRad(double d) { return d * M_PI / 180.0; }
    static double toDeg(double r) { return r * 180.0 / M_PI; }

    // Haversine distance (meters)
    static double distance_m(const LatLon &a, const LatLon &b) {
        double phi1 = toRad(a.lat), phi2 = toRad(b.lat);
        double dphi = toRad(b.lat - a.lat);
        double dlambda = toRad(b.lon - a.lon);
        double s = sin(dphi/2.0), t = sin(dlambda/2.0);
        double A = s*s + cos(phi1)*cos(phi2)*t*t;
        double C = 2.0 * atan2(sqrt(A), sqrt(max(0.0, 1.0 - A)));
        return R * C;
    }

    // Initial bearing from 'from' to 'to' in degrees 0..360
    static double bearing_deg(const LatLon &from, const LatLon &to) {
        double phi1 = toRad(from.lat), phi2 = toRad(to.lat);
        double lam1 = toRad(from.lon), lam2 = toRad(to.lon);
        double dl = lam2 - lam1;
        double y = sin(dl) * cos(phi2);
        double x = cos(phi1)*sin(phi2) - sin(phi1)*cos(phi2)*cos(dl);
        double th = atan2(y,x);
        double deg = fmod(toDeg(th) + 360.0, 360.0);
        return deg;
    }
};

/***************************************************************************************************
* Section:        Thermodynamic & Humidity Helper Functions
* Description:    A collection of free-standing functions for calculating atmospheric properties
* such as saturation vapor pressure, atmospheric pressure at altitude, specific
* humidity, relative humidity, and dew point based on established physical
* formulas (e.g., Magnus/Tetens, ISA barometric formula).
***************************************************************************************************/

static constexpr double MAG_A = 17.27;
static constexpr double MAG_B = 237.7; // °C

inline double saturationVaporPressure_hPa(double tempC) {
    return 6.112 * exp((MAG_A * tempC) / (MAG_B + tempC));
}

inline double hPa_to_Pa(double h) { return h * 100.0; }
inline double Pa_to_hPa(double p) { return p / 100.0; }

inline double pressureAtAltitude_Pa(double altitude_m) {
    double h = max(0.0, altitude_m);
    double base = 1.0 - (0.0065 * h / 288.15);
    if (base <= 0.0) return 20000.0;
    return 101325.0 * pow(base, 5.255877); // Pa
}

inline double vaporPressureFromRH_Pa(double tempC, double rh_percent) {
    double es_hPa = saturationVaporPressure_hPa(tempC);
    double es_Pa = hPa_to_Pa(es_hPa);
    double e_Pa = es_Pa * (clamp(rh_percent, 0.0, 100.0) / 100.0);
    return e_Pa;
}

inline double specificHumidity_from_T_RH_p(double tempC, double rh_percent, double p_Pa) {
    double e = vaporPressureFromRH_Pa(tempC, rh_percent);
    double q = 0.0;
    double denom = p_Pa - 0.378 * e;
    if (denom <= 0.0) return 0.0;
    q = 0.622 * e / denom;
    return q; // unitless (kg/kg)
}

inline double RH_from_T_q_p_percent(double tempC, double q, double p_Pa) {
    double numerator = q * p_Pa;
    double denom = 0.622 + 0.378 * q;
    if (denom <= 0.0) return 0.0;
    double e = numerator / denom; // Pa
    double es_hPa = saturationVaporPressure_hPa(tempC);
    double es_Pa = hPa_to_Pa(es_hPa);
    if (es_Pa <= 0.0) return 0.0;
    double rh = (e / es_Pa) * 100.0;
    rh = clamp(rh, 0.0, 100.0);
    return rh;
}

inline double dewPointC_from_e_Pa(double e_Pa) {
    double e_hPa = Pa_to_hPa(e_Pa);
    if (e_hPa <= 0.0001) return -273.15;
    double gamma = log(e_hPa / 6.112);
    double td = (MAG_B * gamma) / (MAG_A - gamma);
    return td;
}

inline double dewPointC_from_T_RH(double tempC, double rh_percent) {
    double e = vaporPressureFromRH_Pa(tempC, rh_percent);
    return dewPointC_from_e_Pa(e);
}

/***************************************************************************************************
* Struct:         NearbySample
* Description:    Represents a weather data point at a specific geographic location, containing
* the location and its measured relative humidity.
***************************************************************************************************/
struct NearbySample {
    LatLon loc;
    double rh_percent; // 0..100
};

/***************************************************************************************************
* Struct:         SafetyResult
* Description:    A structure to hold the results of a safety analysis, including whether a safe
* altitude or horizontal position was found, the recommended values, and a note
* explaining the result.
***************************************************************************************************/
struct SafetyResult {
    bool altitude_found;
    double safe_altitude_m;
    bool horizontal_found;
    double distance_km;
    double bearing_deg;
    double final_margin_c;
    string note;
};

/***************************************************************************************************
* Class:          WeatherFlightSafetyEnhanced
* Description:    The main class for performing weather safety analysis. It provides methods to
* recommend operational changes for an aircraft based on its current location,
* altitude, and atmospheric conditions to maintain a safe margin between the
* ambient temperature and the dew point.
***************************************************************************************************/
class WeatherFlightSafetyEnhanced {
public:
    // defaults & tunables
    static constexpr double DEFAULT_MARGIN_C = 2.0;
    static constexpr double DEFAULT_STEP_M = 10.0;
    static constexpr double DEFAULT_MAX_UP_M = 5000.0;
    static constexpr double DEFAULT_MAX_DOWN_M = 9999.0;
    static constexpr double DEFAULT_SEA_LEVEL_TEMP_C = 15.0;

    /***********************************************************************************************
    * Function:       recommend
    * Description:    Analyzes the current conditions (altitude, measured temp, RH) to find a
    * safer flight level. It first searches vertically (up and down) for an
    * altitude that meets the required dew point margin, assuming specific
    * humidity is conserved. If unsuccessful, it provides a heuristic estimate
    * for a horizontal distance to travel to find drier air.
    * Inputs:         current_alt_m (double)      - Current aircraft altitude in meters.
    * current_temp_c (double)     - Measured ambient temperature in Celsius.
    * humidity_pct (double)       - Current relative humidity in percent.
    * wind_ms (double)            - Wind speed in m/s (for heuristics).
    * samples (vector<NearbySample>) - Optional nearby weather data.
    * margin_c (double)           - Required T-Td margin in Celsius.
    * ... other search parameters ...
    * Outputs:        A SafetyResult struct with the recommendation.
    ***********************************************************************************************/
    static SafetyResult recommend(
        double current_alt_m,
        double current_temp_c,
        double humidity_pct,
        double wind_ms,
        const vector<NearbySample> &samples = {},
        double margin_c = DEFAULT_MARGIN_C,
        double max_search_up_m = DEFAULT_MAX_UP_M,
        double max_search_down_m = DEFAULT_MAX_DOWN_M,
        double step_m = DEFAULT_STEP_M
    ) {
        SafetyResult out;
        out.altitude_found = false;
        out.safe_altitude_m = NAN;
        out.horizontal_found = false;
        out.distance_km = NAN;
        out.bearing_deg = NAN;
        out.final_margin_c = NAN;
        out.note = "";

        humidity_pct = clamp(humidity_pct, 0.0, 100.0);

        double p_curr = pressureAtAltitude_Pa(current_alt_m);
        double q_curr = specificHumidity_from_T_RH_p(current_temp_c, humidity_pct, p_curr);

        double e_curr = vaporPressureFromRH_Pa(current_temp_c, humidity_pct);
        double td_curr = dewPointC_from_e_Pa(e_curr);
        double margin_curr = current_temp_c - td_curr;

        if (margin_curr >= margin_c) {
            out.altitude_found = true;
            out.safe_altitude_m = current_alt_m;
            out.final_margin_c = margin_curr;
            out.note = "Current altitude meets the dew-point margin (measured temperature used).";
            return out;
        }

        auto ambientTemp_est = [&](double alt_m)->double {
            double lapse = 6.5; // °C per km
            double dkm = (alt_m - current_alt_m) / 1000.0;
            return current_temp_c - lapse * dkm;
        };

        double up_limit = current_alt_m + max_search_up_m;
        for (double alt = current_alt_m + step_m; alt <= up_limit + 1e-9; alt += step_m) {
            double p_new = pressureAtAltitude_Pa(alt);
            double T_new = ambientTemp_est(alt);
            double rh_new = RH_from_T_q_p_percent(T_new, q_curr, p_new);
            double e_new = vaporPressureFromRH_Pa(T_new, rh_new);
            double td_new = dewPointC_from_e_Pa(e_new);
            double margin_new = T_new - td_new;
            if (margin_new >= margin_c) {
                out.altitude_found = true;
                out.safe_altitude_m = alt;
                out.final_margin_c = margin_new;
                ostringstream ss; ss << "Found safe altitude above current altitude conserving specific humidity (q). RH at that alt ~ "
                                     << fixed << setprecision(2) << rh_new << " %.";
                out.note = ss.str();
                return out;
            }
        }

        double low_limit = max(0.0, current_alt_m - max_search_down_m);
        for (double alt = current_alt_m - step_m; alt >= low_limit - 1e-9; alt -= step_m) {
            double p_new = pressureAtAltitude_Pa(alt);
            double T_new = ambientTemp_est(alt);
            double rh_new = RH_from_T_q_p_percent(T_new, q_curr, p_new);
            double e_new = vaporPressureFromRH_Pa(T_new, rh_new);
            double td_new = dewPointC_from_e_Pa(e_new);
            double margin_new = T_new - td_new;
            if (margin_new >= margin_c) {
                out.altitude_found = true;
                out.safe_altitude_m = alt;
                out.final_margin_c = margin_new;
                ostringstream ss; ss << "Found safe altitude below current altitude conserving specific humidity (q). RH at that alt ~ "
                                     << fixed << setprecision(2) << rh_new << " %.";
                out.note = ss.str();
                return out;
            }
        }

        double td_target = current_temp_c - margin_c;
        double gamma_t = (MAG_A * td_target) / (MAG_B + td_target);
        double e_target_hPa = 6.112 * exp(gamma_t);
        double e_target_Pa = hPa_to_Pa(e_target_hPa);
        double es_curr_hPa = saturationVaporPressure_hPa(current_temp_c);
        double es_curr_Pa = hPa_to_Pa(es_curr_hPa);
        double target_rh_percent = (es_curr_Pa <= 0.0) ? 0.0 : (e_target_Pa / es_curr_Pa) * 100.0;
        target_rh_percent = clamp(target_rh_percent, 0.0, 100.0);

        if (!samples.empty()) {
            out.horizontal_found = false;
            out.distance_km = NAN;
            out.note = "Vertical change didn't help. You provided nearby samples, but current location (lat/lon) was not provided to compute distances. Use recommend_with_location(...) overload to enable map-based horizontal lookup.";
            out.final_margin_c = margin_curr;
            return out;
        }

        double assumed_rh_gradient_pct_per_km = 1.0;
        if (target_rh_percent >= humidity_pct - 1e-9) {
            out.horizontal_found = false;
            out.distance_km = NAN;
            out.final_margin_c = margin_curr;
            out.note = "Vertical change did not fix margin and required RH is not lower than current RH (no horizontal remedy computed).";
            return out;
        }
        double delta_rh_needed = humidity_pct - target_rh_percent;
        double dist_km = delta_rh_needed / assumed_rh_gradient_pct_per_km;
        double wind_multiplier = 1.0 + (wind_ms / 50.0);
        dist_km *= wind_multiplier;

        out.horizontal_found = false;
        out.distance_km = dist_km;
        out.final_margin_c = margin_curr;
        {
            ostringstream ss;
            ss << "Vertical change (conserving specific humidity) up to limits did not achieve safety. "
               << "Estimated required RH at current location: " << fixed << setprecision(2) << target_rh_percent << " %. "
               << "Using default horizontal gradient " << assumed_rh_gradient_pct_per_km << " %/km and wind multiplier "
               << fixed << setprecision(2) << wind_multiplier << ", estimated distance: " << dist_km << " km.";
            out.note = ss.str();
        }
        return out;
    }

    /***********************************************************************************************
    * Function:       recommend_with_location
    * Description:    An overload of `recommend` that takes the aircraft's current LatLon. This
    * enables a more accurate horizontal search by using a provided list of
    * nearby weather samples to find the closest safe location and provide an
    * exact distance and bearing.
    * Inputs:         current_loc (LatLon) - Current aircraft location.
    * ... plus all other parameters from `recommend` ...
    * Outputs:        A SafetyResult struct with the recommendation.
    ***********************************************************************************************/
    static SafetyResult recommend_with_location(
        const LatLon &current_loc,
        double current_alt_m,
        double current_temp_c,
        double humidity_pct,
        double wind_ms,
        const vector<NearbySample> &samples,
        double margin_c = DEFAULT_MARGIN_C,
        double max_search_up_m = DEFAULT_MAX_UP_M,
        double max_search_down_m = DEFAULT_MAX_DOWN_M,
        double step_m = DEFAULT_STEP_M
    ) {
        SafetyResult verticalResult = recommend(current_alt_m, current_temp_c, humidity_pct, wind_ms, {}, margin_c, max_search_up_m, max_search_down_m, step_m);

        if (verticalResult.altitude_found) return verticalResult;

        double td_target = current_temp_c - margin_c;
        double gamma_t = (MAG_A * td_target) / (MAG_B + td_target);
        double e_target_hPa = 6.112 * exp(gamma_t);
        double e_target_Pa = hPa_to_Pa(e_target_hPa);
        double es_curr_hPa = saturationVaporPressure_hPa(current_temp_c);
        double es_curr_Pa = hPa_to_Pa(es_curr_hPa);
        double target_rh_percent = (es_curr_Pa <= 0.0) ? 0.0 : (e_target_Pa / es_curr_Pa) * 100.0;
        target_rh_percent = clamp(target_rh_percent, 0.0, 100.0);

        double bestDist = 1e18;
        int bestIdx = -1;
        for (size_t i = 0; i < samples.size(); ++i) {
            if (samples[i].rh_percent <= target_rh_percent + 1e-9) {
                double d = GeoUtils::distance_m(current_loc, samples[i].loc);
                if (d < bestDist) { bestDist = d; bestIdx = (int)i; }
            }
        }

        SafetyResult out = verticalResult;
        if (bestIdx >= 0) {
            out.horizontal_found = true;
            out.distance_km = bestDist / 1000.0;
            out.bearing_deg = GeoUtils::bearing_deg(current_loc, samples[bestIdx].loc);
            out.note = "Found nearby sample meeting target RH; distance and bearing returned.";
            return out;
        }

        int driestIdx = -1;
        double driestRH = 1e9;
        for (size_t i = 0; i < samples.size(); ++i) {
            if (samples[i].rh_percent < driestRH) { driestRH = samples[i].rh_percent; driestIdx = (int)i; }
        }
        if (driestIdx >= 0 && driestRH < humidity_pct - 1e-9) {
            double d_to_driest = GeoUtils::distance_m(current_loc, samples[driestIdx].loc) / 1000.0;
            double proportion = (humidity_pct - target_rh_percent) / (humidity_pct - driestRH);
            proportion = clamp(proportion, 0.0, 10.0);
            double est_dist_km = d_to_driest * proportion;
            out.horizontal_found = false;
            out.distance_km = est_dist_km;
            out.bearing_deg = GeoUtils::bearing_deg(current_loc, samples[driestIdx].loc);
            {
                ostringstream ss;
                ss << "No nearby sample already meets the target RH. Driest sample RH=" << fixed << setprecision(2)
                   << driestRH << " % at distance " << fixed << setprecision(2) << d_to_driest << " km. "
                   << "Estimating required distance to reach target RH by proportional scaling: " << fixed << setprecision(2) << est_dist_km << " km.";
                out.note = ss.str();
            }
            return out;
        }

        double assumed_rh_gradient_pct_per_km = 1.0;
        if (target_rh_percent >= humidity_pct - 1e-9) {
            out.horizontal_found = false;
            out.distance_km = NAN;
            out.note = "Unable to find or estimate horizontal remedy from provided samples.";
            return out;
        }
        double delta_rh_needed = humidity_pct - target_rh_percent;
        double dist_km = delta_rh_needed / assumed_rh_gradient_pct_per_km;
        double wind_multiplier = 1.0 + (wind_ms / 50.0);
        dist_km *= wind_multiplier;
        out.horizontal_found = false;
        out.distance_km = dist_km;
        {
            ostringstream ss;
            ss << "No suitable sample found; fallback heuristic used. Estimated horizontal distance: " << fixed << setprecision(2) << dist_km << " km.";
            out.note = ss.str();
        }
        return out;
    }

    /***********************************************************************************************
    * Function:       recommend_estimatedT
    * Description:    A convenience overload for when measured ambient temperature is not
    * available. It first estimates the temperature at the current altitude
    * using a standard lapse rate from a sea-level baseline, then calls the
    * appropriate recommendation method.
    * Inputs:         ... same as `recommend_with_location` but without `current_temp_c` ...
    * sea_level_temp_c (double) - Assumed temperature at sea level.
    * Outputs:        A SafetyResult struct with the recommendation.
    ***********************************************************************************************/
    static SafetyResult recommend_estimatedT(
        const LatLon &current_loc,
        double current_alt_m,
        double humidity_pct,
        double wind_ms,
        const vector<NearbySample> &samples = {},
        double sea_level_temp_c = DEFAULT_SEA_LEVEL_TEMP_C,
        double margin_c = DEFAULT_MARGIN_C,
        double max_search_up_m = DEFAULT_MAX_UP_M,
        double max_search_down_m = DEFAULT_MAX_DOWN_M,
        double step_m = DEFAULT_STEP_M
    ) {
        double lapse = 6.5; // °C/km
        double T_curr = sea_level_temp_c - lapse * (current_alt_m / 1000.0);
        if (!samples.empty()) {
            return recommend_with_location(current_loc, current_alt_m, T_curr, humidity_pct, wind_ms, samples, margin_c, max_search_up_m, max_search_down_m, step_m);
        } else {
            return recommend(current_alt_m, T_curr, humidity_pct, wind_ms, {}, margin_c, max_search_up_m, max_search_down_m, step_m);
        }
    }
};

/***************************************************************************************************
* Function:       main
* Description:    Entry point and demonstration of the WeatherFlightSafetyEnhanced class.
* It runs two scenarios: one using a measured temperature and another using an
* estimated temperature, printing the safety recommendations for each.
***************************************************************************************************/
int main() {
    cout << fixed << setprecision(2);

    // Example 1: measured temperature available
    LatLon currentLoc(28.6139, 77.2090); // New Delhi
    double curAltM = 2000.0;             // meters
    double curTempC = 5.0;               // measured ambient temp at that altitude
    double curRH = 85.0;                 // %
    double wind_ms = 5.0;

    // Example nearby samples (lat,lon,RH%)
    vector<NearbySample> samples = {
        { LatLon(28.70, 77.20), 90.0 },
        { LatLon(28.50, 77.10), 70.0 },
        { LatLon(29.00, 77.50), 60.0 }
    };

    auto res1 = WeatherFlightSafetyEnhanced::recommend_with_location(
        currentLoc, curAltM, curTempC, curRH, wind_ms, samples, 2.0, 5000.0, 2000.0, 10.0
    );

    if (res1.altitude_found) {
        cout << "[Measured T] Safe altitude found: " << res1.safe_altitude_m << " m (margin " << res1.final_margin_c << " °C)\n";
    } else {
        if (res1.horizontal_found) {
            cout << "[Measured T] Move " << res1.distance_km << " km at bearing " << res1.bearing_deg << "° to reach drier sample.\n";
        } else {
            cout << "[Measured T] Estimated horizontal distance (heuristic): " << res1.distance_km << " km. Note: " << res1.note << "\n";
        }
    }

    // Example 2: no measured temperature -> estimate temperature from sea-level baseline
    LatLon curr2(19.0760, 72.8777); // Mumbai
    double alt2 = 50.0;             // near sea-level
    double rh2 = 90.0;
    double wind2 = 3.0;
    vector<NearbySample> samples2 = {
        { LatLon(19.10, 72.90), 88.0 },
        { LatLon(19.30, 73.00), 60.0 },
    };

    auto res2 = WeatherFlightSafetyEnhanced::recommend_estimatedT(curr2, alt2, rh2, wind2, samples2, 30.0 /*sea-level temp*/);
    if (res2.altitude_found) {
        cout << "[Estimated T] Safe altitude: " << res2.safe_altitude_m << " m\n";
    } else {
        if (res2.horizontal_found) {
            cout << "[Estimated T] Move " << res2.distance_km << " km at bearing " << res2.bearing_deg << "° to a drier spot.\n";
        } else {
            cout << "[Estimated T] Estimated horizontal distance (heuristic): " << res2.distance_km << " km. Note: " << res2.note << "\n";
        }
    }

    return 0;
}