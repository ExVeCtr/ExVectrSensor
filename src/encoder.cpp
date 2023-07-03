#include "ExVectrSensor/encoder.hpp"

namespace VCTR
{

    Core::Topic<Core::Timestamped<Data::ValueCov<int32_t, 1>>> &SNSR::Encoder::getEncoderTopic() {
        return encoderTopic_;
    }

}
