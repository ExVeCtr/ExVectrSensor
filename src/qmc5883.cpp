#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/print.hpp"

#include "ExVectrData/value_covariance.hpp"

#include "ExVectrMath/matrix_base.hpp"

#include "ExVectrHAL/io_types.hpp"
#include "ExVectrHAL/io_params.hpp"

#include "ExVectrSensor/Sensors/qmc5883.hpp"

using namespace VCTR;

bool SNSR::QMC5883::readMag()
{

    if (!initialised_)
    {
        Core::printW("QMC5883 readMag(): sensor not yet initialised!\n");
        return false;
    }

    if (!dataAvailable())
        return false;

    int64_t time = Core::NOW();
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
    buf.data.cov = Math::Matrix<float, 3, 3>::eye(0.006);
    buf.timestamp = time;

    magTopic_.publish(buf);

    return true;
}

int64_t SNSR::QMC5883::getMagInterval() const
{
    return 10 * Core::MILLISECONDS;
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

bool SNSR::QMC5883::initSensor(HAL::IO &ioBus)
{

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

    // uint8_t byte;
    // if (!readByte(address_, QMC5883Registers::QMC5883L_CHIP_ID, &byte)) failed = true;

    ioBus_ = &ioBus;

    uint16_t writeError = false;

    uint8_t b = 0;
    writeError |= !ioBus_->writeByte(QMC5883L_CHIP_ID);
    writeError |= !ioBus_->readByte(b)<<1;

    if (b != 0xFF)
    {
        Core::printE("QMC5883 initSensor(): chip ID was wrong, Usually a connection or setting error! ID was: %d\n", b);
        return false;
    }

    writeError |= !ioBus_->writeByte(QMC5883L_CONFIG2, false)<<2;
    writeError |= !ioBus_->writeByte(0b10000000)<<3;

    Core::delay(10 * Core::MILLISECONDS);

    writeError |= !ioBus_->writeByte(QMC5883L_RESET, false)<<4;
    writeError |= !ioBus_->writeByte(0x01)<<5;

    writeError |= !ioBus_->writeByte(QMC5883L_CONFIG, false)<<6;
    writeError |= !ioBus_->writeByte(0b00011101)<<7;

    writeError |= !ioBus_->writeByte(QMC5883L_CONFIG2, false)<<8;
    writeError |= !ioBus_->writeByte(0b00000000)<<9;

    if (writeError) {
        Core::printE("BME280: Init write failed! Code: %d\n", writeError);
        return false;
    }

    initialised_ = true;

    return true;
}
