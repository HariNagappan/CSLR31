###Use pkg-config to automatically add the necessary flags for libjsoncpp and libcurl. The command would be:
g++ -o open_weather open_weather.cpp $(pkg-config --cflags --libs jsoncpp) $(pkg-config --cflags --libs libcurl)


###After installation, you can just use g++ without any extra flags for standard libraries. The system should automatically know where to find jsoncpp and libcurl.
g++ -o open_weather open_weather.cpp -ljsoncpp -lcurl
