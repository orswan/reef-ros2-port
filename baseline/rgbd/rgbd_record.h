// Adaptation V3 (baseline/README.md): force-included before the converter's
// sources. Declares a record_published overload for reef_msgs::DeltaToVel
// (found by argument-dependent lookup from the stand-in Publisher::publish),
// so the harness can record the init-frame message as published, before the
// original overwrites the shared message object (VISION.md Q6).
#pragma once
#include <reef_msgs/DeltaToVel.h>
namespace reef_msgs {
void record_published(const std::string& topic, const DeltaToVel& m);
}
