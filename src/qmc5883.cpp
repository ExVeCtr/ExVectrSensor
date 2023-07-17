#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/print.hpp"

#include "ExVectrData/value_covariance.hpp"

#include "ExVectrMath/matrix_base.hpp"

#include "ExVectrHAL/io_types.hpp"
#include "ExVectrHAL/io_params.hpp"

#include "ExVectrSensor/Sensors/qmc5883.hpp"

using namespace VCTR;

SNSR::QMC5883Driver::QMC5883Driver(HAL::DigitalIO &ioBus) : Task_Periodic("QMC5883 Driver", 20 * Core::MILLISECONDS)
{
    ioBus_ = &ioBus;
    Core::getSystemScheduler().addTask(*this);
    // setPriority(1000);
}

SNSR::QMC5883Driver::QMC5883Driver(HAL::DigitalIO &ioBus, Core::Scheduler &scheduler) : Task_Periodic("QMC5883 Driver", 20 * Core::MILLISECONDS)
{
    ioBus_ = &ioBus;
    scheduler.addTask(*this);
    // setPriority(1000);
}

void SNSR::QMC5883Driver::taskInit()
{
    if (ioBus_ == nullptr)
    {
        Core::printE("QMC5883 Driver taskInit(): ioBus is a nullptr. Give the constructor the iobus connected with the sensor!\n");
        return;
    }
    if (!initSensor(*ioBus_))
    {
        Core::printE("QMC5883 Driver taskInit(): failed to init sensor!\n");
        return;
    }
    else
    {
        Core::printD("QMC5883 Driver taskInit(): sensor start successful!\n");
    }
}

void SNSR::QMC5883Driver::taskThread()
{
    if (!initialised_)
    {
        Core::printE("QMC5883 Driver taskThread(): sensor is not initialised!\n");
        return;
    }
    readMag();
}

bool SNSR::QMC5883::readMag()
{

    if (!initialised_)
    {
        Core::printW("QMC5883 readMag(): sensor not yet initialised!\n");
        return false;
    }

    if (!dataAvailable())
    {

        if (Core::NOW() - lastSensorData_ > 500 * Core::MILLISECONDS)
        {
            Core::printW("QMC5883 readMag(): no data available for more than 100ms! Something has failed!\n");
        }

        return false;
    }

    int64_t time = lastSensorData_ = Core::NOW();
    uint8_t buffer[6];

    !ioBus_->writeByte(QMC5883L_X_LSB, false);
    if (ioBus_->readData(buffer, 6) != 6)
    {

        Core::printE("QMC5883 readMag(): failed to read from QMC5883L_X_LSB register!\n");
        return false;
    }

    int16_t x = static_cast<int16_t>(buffer[0]) | (static_cast<int16_t>(buffer[1]) << 8);
    int16_t y = static_cast<int16_t>(buffer[2]) | (static_cast<int16_t>(buffer[3]) << 8);
    int16_t z = static_cast<int16_t>(buffer[4]) | (static_cast<int16_t>(buffer[5]) << 8);

    Core::Timestamped<Data::ValueCov<float, 3>> buf;
    buf.data.val[0][0] = (float)x * 8.0f / 32767.0f;
    buf.data.val[1][0] = (float)y * 8.0f / 32767.0f;
    buf.data.val[2][0] = (float)z * 8.0f / 32767.0f;
    buf.data.cov = Math::Matrix<float, 3, 3>::eye(cov_);
    buf.timestamp = time;

    magTopic_.publish(buf);

    return true;
}

bool SNSR::QMC5883::dataAvailable()
{

    if (!initialised_)
    {
        Core::printW("QMC5883 dataAvailable(): sensor not yet initialised!\n");
        return false;
    }

    uint8_t byte = 0;
    !ioBus_->writeByte(QMC5883L_STATUS, false);
    if (!ioBus_->readByte(byte))
    {
        Core::printE("QMC5883 dataAvailable(): failed to read from QMC5883L_STATUS register!\n");
        return false;
    }

    return (byte & 0b00000001) == 0b00000001;
}

bool SNSR::QMC5883::initSensor(HAL::DigitalIO &ioBus)
{

    lastSensorData_ = Core::NOW();

    if (ioBus.getInputType() != HAL::IO_TYPE_t::BUS_I2C)
    {
        VCTR::Core::printE("QMC5883 given incorrect input type. Must be I2C. Given type: %d.\n", ioBus.getInputType());
        return false;
    }

    if (ioBus.getOutputType() != HAL::IO_TYPE_t::BUS_I2C)
    {
        VCTR::Core::printE("QMC5883 given incorrect output type. Must be I2C. Given type: %d.\n", ioBus.getOutputType());
        return false;
    }

    ioBus_ = &ioBus;

    uint16_t writeError = false;

    uint8_t b = 0;
    writeError |= !ioBus_->writeByte(QMC5883L_CHIP_ID, false); // Read chip ID
    writeError |= !ioBus_->readByte(b) << 1;

    if (b != 0xFF)
    {
        Core::printE("QMC5883 initSensor(): chip ID was wrong, Usually a connection or setting error! ID was: %d\n", b);
        return false;
    }

    writeError |= !ioBus_->writeByte(QMC5883L_CONFIG2, false) << 2; // Do software reset
    writeError |= !ioBus_->writeByte(0b10000000) << 3;

    Core::delay(10 * Core::MILLISECONDS);

    writeError |= !ioBus_->writeByte(QMC5883L_RESET, false) << 4; 
    writeError |= !ioBus_->writeByte(0x01) << 5;

    writeError |= !ioBus_->writeByte(QMC5883L_CONFIG, false) << 6;
    writeError |= !ioBus_->writeByte(0b00011101) << 7; 

    if (writeError)
    {
        Core::printE("BME280: Init write failed! Code: %d\n", writeError);
        return false;
    }

    initialised_ = true;

    return true;
}
