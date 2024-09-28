#include "ExVectrSensor/hygrometer.hpp"

namespace VCTR
{

    Core::Topic<Core::Timestamped<Data::ValueCov<float, 1>>> &SNSR::Hygrometer::getHygroTopic() {
        return hygroTopic_;
    }

}
