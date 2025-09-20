/***************************************************************************************************
* File:           open_weather.cpp
* Description:    This program fetches current weather data from the OpenWeatherMap API.
* It uses libcurl for making HTTP requests and jsoncpp for parsing the JSON
* response. The main class, WeatherFetcher, supports fetching data by city name,
* geographic coordinates, or postal code.
***************************************************************************************************/

#ifndef CURL_STATICLIB // Include Guard for static linking
#define CURL_STATICLIB

#include <iostream>
#include <string>
#include <curl/curl.h>
#include <json/json.h>
#include <sstream>

/***************************************************************************************************
* Function:       WriteCallback
* Description:    A callback function for libcurl to handle incoming data from an HTTP request.
* It appends the received data chunk to a std::string buffer.
* Inputs:         contents - Pointer to the data received.
* size     - Size of each data item.
* nmemb    - Number of data items.
* userp    - User-provided pointer, expected to be a std::string*.
* Outputs:        The total number of bytes handled (size * nmemb).
***************************************************************************************************/
size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) 
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

/***************************************************************************************************
* Class:          WeatherFetcher
* Description:    A class designed to interface with the OpenWeatherMap API. It encapsulates
* the logic for building API request URLs, fetching data via cURL, and
* parsing the resulting JSON to display weather information.
***************************************************************************************************/
class WeatherFetcher 
{
private:
    std::string apiKey;
    
    /***********************************************************************************************
    * Function:       fetchDataFromAPI (private)
    * Description:    Performs the HTTP GET request to the specified URL using libcurl.
    * It collects the response into a string.
    * Inputs:         url - The full URL for the API request.
    * Outputs:        A std::string containing the JSON response from the server, or an empty
    * string if an error occurred.
    ***********************************************************************************************/
    std::string fetchDataFromAPI(const std::string& url) 
    {
        CURL* curl;
        CURLcode res;
        std::string readBuffer;

        // Initialize cURL
        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl = curl_easy_init();
        
        if (curl) 
        {
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
            
            res = curl_easy_perform(curl);
            if (res != CURLE_OK) 
            {
                std::cerr << "cURL error: " << curl_easy_strerror(res) << std::endl;
                curl_easy_cleanup(curl);
                curl_global_cleanup();
                return "";
            }
            curl_easy_cleanup(curl);
        }

        curl_global_cleanup();
        return readBuffer;
    }

    /***********************************************************************************************
    * Function:       parseWeatherData (private)
    * Description:    Parses a JSON string containing weather data, extracts key information
    * (city name, temperature, description, humidity), and prints it.
    * Inputs:         jsonResponse - The raw JSON string from the API.
    * Outputs:        None.
    ***********************************************************************************************/
    void parseWeatherData(const std::string& jsonResponse) 
    {
        Json::Value root;
        Json::CharReaderBuilder builder;
        std::string errs;

        std::istringstream sstream(jsonResponse);
        if (Json::parseFromStream(builder, sstream, &root, &errs)) 
        {
            std::string city_name = root["name"].asString();
            double temperature = root["main"]["temp"].asDouble();
            std::string weather_desc = root["weather"][0]["description"].asString();
            double humidity = root["main"]["humidity"].asDouble();
            
            std::cout << "Weather in " << city_name << ":\n";
            std::cout << "Temperature: " << temperature << "°C\n";
            std::cout << "Description: " << weather_desc << "\n";
            std::cout << "Humidity: " << humidity << "%\n";
        } 
        else 
        {
            std::cerr << "Failed to parse the response JSON.\n";
        }
    }

public:
    /***********************************************************************************************
    * Function:       WeatherFetcher (Constructor)
    * Description:    Initializes the WeatherFetcher with the necessary API key.
    * Inputs:         key - The OpenWeatherMap API key as a std::string.
    * Outputs:        None.
    ***********************************************************************************************/
    WeatherFetcher(const std::string& key) : apiKey(key) {}

    /***********************************************************************************************
    * Function:       fetchWeatherData (by City Name)
    * Description:    Fetches and displays weather data for a given city name.
    * Inputs:         cityName - The name of the city.
    * Outputs:        None.
    ***********************************************************************************************/
    void fetchWeatherData(const std::string& cityName) 
    {
        std::string url = "http://api.openweathermap.org/data/2.5/weather?q=" + cityName +
                          "&appid=" + apiKey + "&units=metric";
        std::string response = fetchDataFromAPI(url);
        if (!response.empty()) 
        {
            parseWeatherData(response);
        }
    }

    /***********************************************************************************************
    * Function:       fetchWeatherData (by Latitude and Longitude)
    * Description:    Fetches and displays weather data for a given set of coordinates.
    * Inputs:         lat - The latitude.
    * lon - The longitude.
    * Outputs:        None.
    ***********************************************************************************************/
    void fetchWeatherData(double lat, double lon) 
    {
        std::string url = "http://api.openweathermap.org/data/2.5/weather?lat=" + std::to_string(lat) +
                          "&lon=" + std::to_string(lon) + "&appid=" + apiKey + "&units=metric";
        std::string response = fetchDataFromAPI(url);
        if (!response.empty()) 
        {
            parseWeatherData(response);
        }
    }

    /***********************************************************************************************
    * Function:       fetchWeatherData (by Postal Code)
    * Description:    Fetches and displays weather data for a given postal code.
    * Inputs:         pinCode - The postal code (e.g., ZIP code).
    * Outputs:        None.
    ***********************************************************************************************/
    void fetchWeatherData(int pinCode) 
    {
        std::string url = "http://api.openweathermap.org/data/2.5/weather?zip=" + std::to_string(pinCode) +
                          "&appid=" + apiKey + "&units=metric";
        std::string response = fetchDataFromAPI(url);
        if (!response.empty()) 
        {
            parseWeatherData(response);
        }
    }
};

// ***************************************************************************************************

/***************************************************************************************************
* Function:       main
* Description:    Entry point of the program. Initializes the WeatherFetcher with an API key
* and demonstrates fetching weather data using all three available methods:
* by city name, by coordinates, and by postal code.
***************************************************************************************************/


/*

int main() 
{
    std::string apiKey = "bd5e378503939ddaee76f12ad7a97608"; // Replace with your OpenWeather API key
    WeatherFetcher weatherFetcher(apiKey);

    // Example 1: Fetch weather by City Name
    std::cout << "Fetching weather for city 'London':\n";
    weatherFetcher.fetchWeatherData("London");

    std::cout << "\n";

    // Example 2: Fetch weather by Coordinates (Latitude and Longitude)
    std::cout << "Fetching weather for coordinates (Latitude: 51.5074, Longitude: -0.1278):\n";
    weatherFetcher.fetchWeatherData(51.5074, -0.1278);

    std::cout << "\n";

    // Example 3: Fetch weather by Postal Code (Pin Code)
    std::cout << "Fetching weather for Pin Code '94040':\n";
    weatherFetcher.fetchWeatherData(94040);

    return 0;
}

*/

#endif // CURL_STATICLIB