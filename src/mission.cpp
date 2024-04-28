#include "mission.hpp"
#include <fstream>
#include <string>
#include "json.hpp"

void Mission::init() {
  
  std::ifstream i("mission.json");
  nlohmann::json data = nlohmann::json::parse(i);

  nlohmann::json Coordinates = data["Coordinates"];
  nlohmann::json Target = data["Target"];
  nlohmann::json Atmosphere = data["Atmosphere"];
  nlohmann::json Gun = data["Gun"];

  std::string coord_type = Coordinates["Type"];
  std::string coord_unit = Coordinates["Unit"];
  bool lat = Coordinates["Latitude"];
  bool lon = Coordinates["Longitude"];
  bool alt = Coordinates["Altitude"];
  

  

}