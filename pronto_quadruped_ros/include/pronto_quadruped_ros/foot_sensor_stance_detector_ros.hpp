#pragma once

#include <pronto_msgs/msg/quadruped_stance.hpp>
#include <pronto_quadruped/FootSensorStanceDetector.hpp>

namespace pronto {
namespace quadruped {

class FootSensorStanceDetectorROS : public FootSensorStanceDetector {
public:
  FootSensorStanceDetectorROS() = default;
};

}  // namespace quadruped
}  // namespace pronto
