#include "ExVectrSensor/gnss.hpp"

namespace VCTR
{

    Core::Topic<Core::Timestamped<SNSR::GNSSData>> &SNSR::GNSS::getGNSSTopic() {
        return gnssTopic_;
    }

}
