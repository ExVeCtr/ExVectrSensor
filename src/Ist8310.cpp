#include "ExVectrCore/print.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/timestamped.hpp"

#include "ExVectrDSP/value_covariance.hpp"

#include "ExVectrMath/matrix_base.hpp"

#include "ExVectrHAL/io_params.hpp"
#include "ExVectrHAL/io_types.hpp"

#include "ExVectrSensor/Sensors/Ist8310.hpp"

namespace VCTR::sensor::sensors {

Ist8310Driver::Ist8310Driver(HAL::DigitalIO &ioBus)
    : Ist8310(ioBus), Task_Periodic("IST8310 Driver", 20 * Core::MILLISECONDS) {
  Core::getSystemScheduler().addTask(*this);
  // setPriority(1000);
}

Ist8310Driver::Ist8310Driver(HAL::DigitalIO &ioBus, Core::Scheduler &scheduler)
    : Ist8310(ioBus), Task_Periodic("IST8310 Driver", 20 * Core::MILLISECONDS) {
  scheduler.addTask(*this);
  // setPriority(1000);
}

void Ist8310Driver::taskInit() {
  if (!initSensor()) {
    LOG_MSG("failed to init sensor!\n");
    setInitialised(false);
    setRelease(Core::NOW() + Core::MILLISECONDS * 1000);
    return;
  } else {
    LOG_MSG("sensor start successful!\n");
  }
}

void Ist8310Driver::taskThread() {
  if (!initialised) {
    LOG_MSG("sensor is not initialised!\n");
    setInitialised(false);
    setRelease(Core::NOW() + Core::MILLISECONDS * 1000);
    return;
  }
  readMag();
  if (Core::NOW() - lastSensorData > 500 * Core::MILLISECONDS) {
    LOG_MSG("no data available for more than 500ms! Something has failed! "
            "Restarting sensor!\n");
    setInitialised(false);
  }
  // triggerRead();
  setDeadline(Core::NOW() + 10 * Core::MILLISECONDS);
  setRelease(Core::NOW() + 5 * Core::MILLISECONDS);
}

Ist8310::Ist8310(HAL::DigitalIO &ioBus) : busDevice(ioBus) {}

bool Ist8310::readMag() {

  if (!initialised) {
    LOG_MSG("sensor not yet initialised!\n");
    return false;
  }

  if (!dataAvailable()) {
    if (Core::NOW() - lastSensorData > 100 * Core::MILLISECONDS) {
      triggerRead();
      LOG_MSG("data not available to read! Triggering new measurement!\n");
    }
    return false;
  }

  int64_t time = lastSensorData = Core::NOW();
  uint8_t buffer[6];
  if (!busDevice.readReg(IST8310_X_LSB, buffer, 6)) {
    LOG_MSG("failed to read from IST8310_X_LSB register!");
    return false;
  }

  int16_t x =
      static_cast<int16_t>(buffer[0]) | (static_cast<int16_t>(buffer[1]) << 8);
  int16_t y =
      static_cast<int16_t>(buffer[2]) | (static_cast<int16_t>(buffer[3]) << 8);
  int16_t z =
      static_cast<int16_t>(buffer[4]) | (static_cast<int16_t>(buffer[5]) << 8);

  Core::Timestamped<DSP::ValueCov<float, 3>> buf;
  buf.data.val[0][0] = (float)x;
  buf.data.val[1][0] = (float)y;
  buf.data.val[2][0] = -(float)z;
  buf.data.cov = Math::Matrix<float, 3, 3>::eye(cov);
  buf.timestamp = time;

  LOG_MSG("mag data: %f, %f, %f\n", buf.data.val(0), buf.data.val(1),
          buf.data.val(2));

  magTopic_.publish(buf);

  return true;
}

void Ist8310::triggerRead() {
  busDevice.writeReg(IST8310_CONTROL1, 0b00000001);
}

bool Ist8310::dataAvailable() {

  if (!initialised) {
    LOG_MSG("sensor not yet initialised!\n");
    return false;
  }

  uint8_t byte = 0;
  if (busDevice.readReg(IST8310_CONTROL2, &byte, 1) != 1) {
    LOG_MSG("failed to read from IST8310_CONTROL2 register!\n");
    return false;
  }

  return byte & 0b00001000;
}

bool Ist8310::initSensor() {

  lastSensorData = Core::NOW();

  uint16_t writeError = false;

  uint8_t b = 0;
  writeError |= !busDevice.readReg(IST8310_WHOAMI, &b);

  if (b != IST8310_WHOAMI_VALUE) {
    LOG_MSG("chip ID was wrong, Usually a connection or setting error! ID was: "
            "%d\n",
            b);
    // return false;
  }

  writeError |= !busDevice.writeReg(IST8310_CONTROL2, 0xFF);

  Core::delay(10 * Core::MILLISECONDS);

  writeError |= !busDevice.writeReg(IST8310_CONTROL2, 0b00001100);

  if (writeError) {
    LOG_MSG("Init write failed! Code: %d\n", writeError);
    return false;
  }

  initialised = true;

  // Begin measurement
  triggerRead();
  lastSensorData = Core::NOW();

  return true;
}

} // namespace VCTR::sensor::sensors
