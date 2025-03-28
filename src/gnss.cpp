#include "ExVectrSensor/gnss.hpp"

namespace VCTR
{

    Core::Topic<Core::Timestamped<Data::GNSSData>> &SNSR::GNSS::getGNSSTopic() {
        return gnssTopic_;
    }

}
