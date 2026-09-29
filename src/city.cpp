#include "../include/city.h"
#include "../include/growing_city.h"
#include "../include/parallax_city.h"

std::unique_ptr<City> makeCity(CityType _type) {
  switch (_type) {
  case CityType::GROWING:
    return std::make_unique<GrowingCity>();
  case CityType::PARALLAX:
  default:
    return std::make_unique<ParallaxCity>();
  }
}
