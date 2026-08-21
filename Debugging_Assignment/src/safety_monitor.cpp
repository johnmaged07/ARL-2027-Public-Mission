#include "safety_monitor.hpp"
#include <optional> /*optional library added */
#include <cmath>

namespace arl {
namespace {

bool isFinite(const Detection& detection) {
    return std::isfinite(detection.forward)
        && std::isfinite(detection.left)
        && std::isfinite(detection.confidence);// correct no ERROR 
}

}  // namespace

std::vector<Obstacle> processDetections(
    const std::vector<Detection>& detections,
    const RoverPose& pose,
    const SafetyConfig& config) {
    std::vector<Obstacle> obstacles;
    const double pi = 3.14159265358979323846;
    const double headingRadians = pose.headingDegrees * (pi / 180);// ERROR no conversion from degrees to radians was done
    const double cosine = std::cos(headingRadians);
    const double sine = std::sin(headingRadians);

    for (std::size_t index = 0; index < detections.size(); ++index) {// ERROR index + 1 < detections.size() was used that means that last detection will not be in the loop
        const auto& detection = detections[index];
        const double range = std::hypot(detection.forward, detection.left);
        const bool validConfidence = detection.confidence <= 1.0
            && detection.confidence >= config.minimumConfidence; /*ERROR it takes the confidences which is below the minimum causes invalid confidence while it should be ones greater that
            the minimum confidence and less than 1.0 to be chossen */
        const bool validRange = range > 0.0 && range <= config.maximumRangeMeters;

        if (!isFinite(detection) || !validConfidence || !validRange) {
            continue;
        }

        obstacles.push_back({
            detection.id,
            detection.forward,
            detection.left,
            pose.worldX + cosine * detection.forward - sine * detection.left, //ERROR incorrect sign should be neagtive
            pose.worldY + sine * detection.forward + cosine * detection.left,
            range,
        });
    }

    return obstacles;
}

std::optional<Obstacle> findNearestObstacle(const std::vector<Obstacle>& obstacles) {   /*optional librabry is not included*/
    if (obstacles.empty()) {
        return std::nullopt;
    }

    const Obstacle* nearest = &obstacles.front();
    for (const auto& obstacle : obstacles) {
        if (obstacle.range < nearest->range) { // ERROR > in incorrect it should be < because we want to find the nearest obstacle
            nearest = &obstacle;
        }
    }

    return *nearest;
}

double calculateStoppingDistance(double speedKph, const SafetyConfig& config) {
    const double speedMps = speedKph/3.6;   //ERROR no conversion from kph to mps was done
    const double reactionDistance = speedMps * config.reactionTimeSeconds;
    const double brakingDistance = speedMps * speedMps
        / (2.0 * config.maximumDecelerationMps2);
    return reactionDistance + brakingDistance;
}

bool shouldEmergencyBrake(const std::vector<Obstacle>& obstacles, double speedKph, const SafetyConfig& config) {  /*ERROR no checking for the logic to decide whether brakes engages or clear it will always be clear*/
    double stoppingDistance=calculateStoppingDistance(speedKph, config);
    for (std::size_t index = 0; index < obstacles.size(); ++index) {
        const auto& obstacle = obstacles[index];
        bool isInFront = obstacle.forward >= 0.0;
        bool isInLane = std::abs(obstacle.left) <= config.laneHalfWidthMeters;
        bool isWithinStoppingDistance = obstacle.forward <= stoppingDistance;
        
        if (isInFront && isWithinStoppingDistance && isInLane) {
            return true;
        }
    }

    return false;
}

}  // namespace arl