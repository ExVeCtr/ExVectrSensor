#ifndef EXVECTRSENSOR_IST8310_HPP
#define EXVECTRSENSOR_IST8310_HPP

#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrHAL/bus_device.hpp"
#include "ExVectrHAL/digital_io.hpp"

#include "../magnetometer.hpp"

namespace VCTR::sensor::sensors {

/**
 * @brief This class implements control for the IST8310 3 axis magnetometer.
 */
class Ist8310 : public VCTR::sensor::Magnetometer {
protected:
  HAL::BusDevice busDevice;

  bool initialised = false;

  int64_t lastSensorData = 0;

  // Sensor covariance
  float cov = 0.05f;

  static constexpr uint8_t IST8310_WHOAMI_VALUE = 0x10;

  static constexpr uint8_t IST8310_WHOAMI = 0x00;
  static constexpr uint8_t IST8310_STATUS1 = 0x02;
  static constexpr uint8_t IST8310_X_LSB = 0x03;
  static constexpr uint8_t IST8310_X_MSB = 0x04;
  static constexpr uint8_t IST8310_Y_LSB = 0x05;
  static constexpr uint8_t IST8310_Y_MSB = 0x06;
  static constexpr uint8_t IST8310_Z_LSB = 0x07;
  static constexpr uint8_t IST8310_Z_MSB = 0x08;
  static constexpr uint8_t IST8310_STATUS2 = 0x09;
  static constexpr uint8_t IST8310_CONTROL1 = 0x0A;
  static constexpr uint8_t IST8310_CONTROL2 = 0x0B;
  static constexpr uint8_t IST8310_TEMP_LSB = 0x1C;
  static constexpr uint8_t IST8310_TEMP_MSB = 0x1D;

public:
  Ist8310(HAL::DigitalIO &ioBus);

  /**
   * @brief Initialises and sets sensors settings.
   * @param ioBus Which bus to use for communications.
   * @return true if successfull and sensor is running, false otherwise.
   */
  bool initSensor();

  /**
   * @brief Makes the sensor read the values and publish them to the topic.
   * @note call triggerRead after to take another measurement.
   * @return true if reading was successfull. False otherwise.
   */
  bool readMag() override;

  /**
   * @brief Triggers the sensor to take a measurement.
   * @note Measurement will be ready in 5ms.
   */
  void triggerRead();

private:
  /**
   * @brief Checks if data is currently available to read.
   * @return true if available
   */
  bool dataAvailable();
};

/**
 * @brief This class uses tasks to automatically init and read the sensor.
 */
class Ist8310Driver : public Ist8310, public Core::Task_Periodic {
public:
  /**
   * @brief Constructor that uses the standard system scheduler.
   * @param ioBus The bus connection with sensor.
   */
  Ist8310Driver(HAL::DigitalIO &ioBus);

  /**
   * @brief Constructor that uses the given scheduler.
   * @param ioBus The bus connection with sensor.
   * @param scheduler Scheduler to use for this driver.
   */
  Ist8310Driver(HAL::DigitalIO &ioBus, Core::Scheduler &scheduler);

  /**
   * @brief Initialises sensor and expected to be called once at start by
   * scheduler
   */
  void taskInit() override;

  /**
   * @brief main task thread that reads all sensor data and publishes it. To be
   * called by scheduler.
   */
  void taskThread() override;
};

} // namespace VCTR::sensor::sensors

#endif