#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

#define LOG(x) std::cout << x << std::endl;

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;    
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below.
    float start_x, start_y, goal_x, goal_y;
    do {
        std::cout << "Enter X coordinate of the start: " << std::endl;
        std::cin >> start_x;
    }while(start_x < 0.0 || start_x > 100.0);
    do {
        std::cout << "Enter Y coordinate of the start: " << std::endl;
        std::cin >> start_y;
    }while(start_y < 0.0 || start_y > 100.0);
    do {
        std::cout << "Enter X coordinate of the goal: " << std::endl;
        std::cin >> goal_x;
    }while(goal_x < 0.0 || goal_x > 100.0);
    do {
        std::cout << "Enter Y coordinate of the goal: " << std::endl;
        std::cin >> goal_y;
    }while(goal_y < 0.0 || goal_y > 100.0);
    // Build Model.
    RouteModel model{osm_data};

    // Perform search and render results.
    RoutePlanner route_planner{model, start_x, start_y, goal_x, goal_y};
  	route_planner.AStarSearch();
  	std::cout << "Distance to go: " << route_planner.GetDistance() << " meters." << std::endl;
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
