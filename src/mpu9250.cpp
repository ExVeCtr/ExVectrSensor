
/**
 * Credit for original implementation to:
 *
 * Brian R Taylor
 * brian.taylor@bolderflight.com
 * Copyright (c) 2021 Bolder Flight Systems Inc
 *
 * Original implementation: https://github.com/bolderflight/invensense-imu
 *
 * Heavily modified for use with the ExVectrSensor library with extra features.
 *
 *
 */

#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/print.hpp"

#include "ExVectrHAL/io_types.hpp"
#include "ExVectrHAL/io_params.hpp"

#include "ExVectrSensor/Sensors/mpu9250.hpp"

namespace VCTR
{

    // ### Below is MPU9250Driver Implementation ###

    SNSR::MPU9250Driver::MPU9250Driver(HAL::IO &ioBus, bool disableMag) : Task_Periodic("MPU9250 Driver", 1.01 * Core::MILLISECONDS)
    {
        disableMag_ = disableMag;
        ioBus_ = &ioBus;
        Core::getSystemScheduler().addTask(*this);
        setPriority(1000);
    }

    SNSR::MPU9250Driver::MPU9250Driver(HAL::IO &ioBus, Core::Scheduler &scheduler, bool disableMag) : Task_Periodic("MPU9250 Driver", 1.01 * Core::MILLISECONDS)
    {
        disableMag_ = disableMag;
        ioBus_ = &ioBus;
        scheduler.addTask(*this);
        setPriority(1000);
    }

    void SNSR::MPU9250Driver::taskInit()
    {
        if (ioBus_ == nullptr)
        {
            Core::printE("MPU9250 taskInit(): ioBus is a nullptr. Give the constructor the iobus connected with the sensor!\n");
            return;
        }
        if (!initSensor(*ioBus_))
        {
            Core::printE("MPU9250 taskInit(): failed to init sensor!\n");
            return;
        }
    }

    void SNSR::MPU9250Driver::taskThread()
    {
        if (!initialised_)
        {
            Core::printE("MPU9250 taskThread(): sensor is not initialised!\n");
            return;
        }
        readAll();
    }

    // ### Below is MPU9250 Implementation ###

    SNSR::MPU9250::MPU9250(bool disableMag = false)
    {
        disableMag_ = disableMag;
    }

    bool SNSR::MPU9250::initSensor(HAL::IO &ioBus)
    {

        if (ioBus.getInputType() != HAL::IO_TYPE_t::BUS_I2C && ioBus.getInputType() != HAL::IO_TYPE_t::BUS_SPI)
        {
            VCTR::Core::printE("MPU9250 Connected to incorrect input type. Must be I2C or SPI. Given type: %d.\n", ioBus.getInputType());
            return false;
        }

        if (ioBus.getOutputType() != HAL::IO_TYPE_t::BUS_I2C && ioBus.getOutputType() != HAL::IO_TYPE_t::BUS_SPI)
        {
            VCTR::Core::printE("MPU9250 Connected to incorrect output type. Must be I2C or SPI. Given type: %d.\n", ioBus.getOutputType());
            return false;
        }

        ioBus_ = &ioBus;

        if (ioBus_->getInputType() == HAL::IO_TYPE_t::BUS_I2C)
        {
            ioBus_->setInputParam(HAL::IO_PARAM_t::PARAM_SPEED, 400000);
        }
        else
        {
            ioBus_->setOutputParam(HAL::IO_PARAM_t::PARAM_SPEED, spi_clock_);
            ioBus_->setOutputParam(HAL::IO_PARAM_t::PARAM_MSBFIRST, true);
            ioBus_->setOutputParam(HAL::IO_PARAM_t::PARAM_SPIMODE, 3);
        }

        if (ioBus_->getOutputType() == HAL::IO_TYPE_t::BUS_I2C)
        {
            ioBus_->setOutputParam(HAL::IO_PARAM_t::PARAM_SPEED, 400000);
        }
        else
        {
            ioBus_->setOutputParam(HAL::IO_PARAM_t::PARAM_SPEED, spi_clock_);
            ioBus_->setOutputParam(HAL::IO_PARAM_t::PARAM_MSBFIRST, true);
            ioBus_->setOutputParam(HAL::IO_PARAM_t::PARAM_SPIMODE, 3);
        }

        if (disableMag_)
            VCTR::Core::printD("MPU9250 initSensor(): Configured to disable mag!\n");

        if (!Begin(disableMag_))
        {
            VCTR::Core::printW("MPU9250 Failed to start!\n");
            return false;
        }

        EnableDrdyInt();
        ConfigAccelRange(AccelRange::ACCEL_RANGE_16G);
        ConfigGyroRange(GyroRange::GYRO_RANGE_2000DPS);
        ConfigDlpf(DlpfBandwidth::DLPF_BANDWIDTH_184HZ);
        ConfigSrd(0);

        initialised_ = true; // Keep at end of function!

        return true;
    }

    bool SNSR::MPU9250::readGyro()
    {
        if (!initialised_)
        {
            VCTR::Core::printE("MPU9250 readGyro(): class not yet initialised!\n");
            return false;
        }

        int64_t time = VCTR::Core::NOW();

        if (!Read())
        {
            VCTR::Core::printW("MPU9250 readGyro(): Something went wrong reading sensor values!\n");
            return false;
        }

        Data::ValueCov<float, 3> gyroVals;
        gyroVals.val(0) = gyro_x_radps();
        gyroVals.val(1) = gyro_y_radps();
        gyroVals.val(2) = gyro_z_radps();

        gyroVals.cov = gyroVariance_;

        gyroTopic_.publish(Core::Timestamped<Data::ValueCov<float, 3>>(gyroVals, time));

        return true;
    }

    bool SNSR::MPU9250::readAccel()
    {
        if (!initialised_)
        {
            VCTR::Core::printE("MPU9250 readAccel(): class not yet initialised!\n");
            return false;
        }

        int64_t time = VCTR::Core::NOW();

        if (!Read())
        {
            VCTR::Core::printW("MPU9250 readAccel(): Something went wrong reading sensor values!\n");
            return false;
        }

        Data::ValueCov<float, 3> accelVals;
        accelVals.val(0) = accel_x_mps2();
        accelVals.val(1) = accel_y_mps2();
        accelVals.val(2) = accel_z_mps2();

        accelVals.cov = accelVariance_;

        accelTopic_.publish(Core::Timestamped<Data::ValueCov<float, 3>>(accelVals, time));

        return true;
    }

    bool SNSR::MPU9250::readMag()
    {
        if (!initialised_)
        {
            VCTR::Core::printE("MPU9250 readMag(): class not yet initialised!\n");
            return false;
        }
        if (akFailure_)
        {
            VCTR::Core::printE("MPU9250 readMag(): Mag failed!\n");
            return false;
        }
        if (disableMag_)
        {
            return false;
        }

        int64_t time = VCTR::Core::NOW();

        if (!Read())
        {
            VCTR::Core::printW("MPU9250 readMag(): Something went wrong reading sensor values!\n");
            return false;
        }

        Data::ValueCov<float, 3> magVals;
        magVals.val(0) = mag_x_ut();
        magVals.val(1) = mag_y_ut();
        magVals.val(2) = mag_z_ut();

        magVals.cov = magVariance_;

        magTopic_.publish(Core::Timestamped<Data::ValueCov<float, 3>>(magVals, time));

        return true;
    }

    bool SNSR::MPU9250::readAll()
    {

        if (!initialised_)
        {
            VCTR::Core::printE("MPU9250 readAll(): class not yet initialised!\n");
            return false;
        }

        int64_t time = VCTR::Core::NOW();

        if (!Read())
        {
            VCTR::Core::printW("MPU9250 readAll(): Something went wrong reading sensor values!\n");
            return false;
        }

        Data::ValueCov<float, 3> gyroVals;
        gyroVals.val(0) = gyro_x_radps();
        gyroVals.val(1) = gyro_y_radps();
        gyroVals.val(2) = gyro_z_radps();
        gyroVals.cov = gyroVariance_;

        gyroTopic_.publish(Core::Timestamped<Data::ValueCov<float, 3>>(gyroVals, time));

        Data::ValueCov<float, 3> accelVals;
        accelVals.val(0) = accel_x_mps2();
        accelVals.val(1) = accel_y_mps2();
        accelVals.val(2) = accel_z_mps2();
        accelVals.cov = accelVariance_;

        accelTopic_.publish(Core::Timestamped<Data::ValueCov<float, 3>>(accelVals, time));

        if (disableMag_)
            return true;

        if (akFailure_)
        {
            VCTR::Core::printE("MPU9250 readMag(): Mag failed!\n");
            return false;
        }

        Data::ValueCov<float, 3> magVals;
        magVals.val(0) = mag_x_ut();
        magVals.val(1) = mag_y_ut();
        magVals.val(2) = mag_z_ut();
        magVals.cov = magVariance_;

        magTopic_.publish(Core::Timestamped<Data::ValueCov<float, 3>>(magVals, time));

        return true;
    }

    int64_t SNSR::MPU9250::getMagInterval() const
    {
        return 10 * Core::MILLISECONDS;
    }

    int64_t SNSR::MPU9250::getAccelInterval() const
    {
        return 1 * Core::MILLISECONDS;
    }

    int64_t SNSR::MPU9250::getGyroInterval() const
    {
        return 1 * Core::MILLISECONDS;
    }

    /**
     * Below are implementations from original implementions but modified for ExVectr.
     */

    bool SNSR::MPU9250::Begin(bool disableMag)
    {

        akFailure_ = disableMag; // ####################### Reset in case it works again
        mpuFailure_ = false;     // ####################### Reset in case it works again

        spi_clock_ = 1000000;
        /* Select clock source to gyro */
        VCTR::Core::printD("MPU9250 selecting clock source to gyro.\n");
        if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_))
        {
            akFailure_ = true;
            mpuFailure_ = true;
            VCTR::Core::printE("MPU9250 failed to select clock source to gyro!\n");
            return false;
        }
        VCTR::Core::printD("MPU9250 enabling master mode.\n");
        /* Enable I2C master mode */
        if (!WriteRegister(USER_CTRL_, I2C_MST_EN_))
        {
            akFailure_ = true;
            mpuFailure_ = true;
            VCTR::Core::printE("MPU9250 failed to enable I2C master mode!\n");
            return false;
        }
        VCTR::Core::printD("MPU9250 setting I2C speed.\n");
        /* Set the I2C bus speed to 400 kHz */
        if (!WriteRegister(I2C_MST_CTRL_, I2C_MST_CLK_))
        {
            akFailure_ = true;
            mpuFailure_ = true;
            VCTR::Core::printE("MPU9250 failed to set I2C bus speed!\n");
            return false;
        }
        /* Set AK8963 to power down */
        VCTR::Core::printD("MPU9250 powering down mag.\n");
        if (!akFailure_)
            WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);
        /* Reset the MPU9250 */
        VCTR::Core::printD("MPU9250 resetting.\n");
        WriteRegister(PWR_MGMNT_1_, H_RESET_);
        /* Wait for MPU-9250 to come back up */
        VCTR::Core::delay(VCTR::Core::MILLISECONDS * 10); // ###################### increased from 1ms to 10ms
        /* Reset the AK8963 */
        VCTR::Core::printD("MPU9250 resetting mag.\n");
        if (!akFailure_)
            WriteAk8963Register(AK8963_CNTL2_, AK8963_RESET_);
        /* Select clock source to gyro */
        VCTR::Core::printD("MPU9250 selecting clock source to gyro.\n");
        if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_))
        {
            VCTR::Core::printE("MPU9250 failed to select gyro clock source!\n");
            return false;
        }
        /* Check the WHO AM I byte */
        VCTR::Core::printD("MPU9250 reading whoami.\n");
        uint8_t who_am_i;
        if (!ReadRegisters(WHOAMI_, sizeof(who_am_i), &who_am_i))
        {
            akFailure_ = true;
            mpuFailure_ = true;
            VCTR::Core::printE("MPU9250 WHOAMI read failed!\n");
            return false;
        }
        VCTR::Core::printD("MPU9250 checking whoami.\n");
        if ((who_am_i != WHOAMI_MPU9250_) && (who_am_i != WHOAMI_MPU9255_))
        {
            akFailure_ = true;
            mpuFailure_ = true;
            VCTR::Core::printE("MPU9250 WHOAMI check failed. WHOAMI: %d!\n", who_am_i);
            return false;
        }
        /* Enable I2C master mode */
        VCTR::Core::printD("MPU9250 enabling i2c master mode.\n");
        if (!WriteRegister(USER_CTRL_, I2C_MST_EN_))
        {
            akFailure_ = true;
            mpuFailure_ = true;
            VCTR::Core::printE("MPU9250 failed to set I2C master mode (2nd set)!\n");
            return false;
        }
        /* Set the I2C bus speed to 400 kHz */
        VCTR::Core::printD("MPU9250 setting i2c bus speed.\n");
        if (!WriteRegister(I2C_MST_CTRL_, I2C_MST_CLK_))
        {
            akFailure_ = true;
            mpuFailure_ = true;
            VCTR::Core::printE("MPU9250 failed to set I2C speed (2nd set)!\n");
            return false;
        }
        VCTR::Core::delay(VCTR::Core::MILLISECONDS * 10);
        if (!akFailure_)
        {
            /* Check the AK8963 WHOAMI */
            VCTR::Core::printD("MPU9250 reading mag whoami.\n");
            if (!ReadAk8963Registers(AK8963_WHOAMI_, sizeof(who_am_i), &who_am_i))
            {
                akFailure_ = true;
                mpuFailure_ = true;
                VCTR::Core::printE("MPU9250 failed to read mag WHOAMI!\n");
                return false;
            }
            VCTR::Core::printD("MPU9250 checking mag whoami.\n");
            if (who_am_i != WHOAMI_AK8963_)
            {
                akFailure_ = true;
                VCTR::Core::printE("MPU9250 mag WHOAMI check failed. WHOAMI: %d! Disabling mag!\n", who_am_i);
            }
        }
        /* Skip if mag failed */
        if (!akFailure_)
        {
            /* Get the magnetometer calibration */
            /* Set AK8963 to power down */
            VCTR::Core::printD("MPU9250 power down mag.\n");
            if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_))
            {
                akFailure_ = true;
                mpuFailure_ = true;
                VCTR::Core::printE("MPU9250 failed to power down mag!\n");
                return false;
            }
            VCTR::Core::delay(VCTR::Core::MILLISECONDS * 100); // long wait between AK8963 mode changes
            /* Set AK8963 to FUSE ROM access */
            VCTR::Core::printD("MPU9250 set FUSE ROM acecss.\n");
            if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_FUSE_ROM_))
            {
                akFailure_ = true;
                mpuFailure_ = true;
                VCTR::Core::printE("MPU9250 failed to set FUSE rom access!\n");
                return false;
            }
            VCTR::Core::delay(VCTR::Core::MILLISECONDS * 100); // long wait between AK8963 mode changes
            /* Read the AK8963 ASA registers and compute magnetometer scale factors */
            uint8_t asa_buff[3];
            VCTR::Core::printD("MPU9250 reading mag ASA registers.\n");
            if (!ReadAk8963Registers(AK8963_ASA_, sizeof(asa_buff), asa_buff))
            {
                akFailure_ = true;
                mpuFailure_ = true;
                VCTR::Core::printE("MPU9250 failed to read ASA registers!\n");
                return false;
            }
            mag_scale_[0] = ((static_cast<float>(asa_buff[0]) - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f;
            mag_scale_[1] = ((static_cast<float>(asa_buff[1]) - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f;
            mag_scale_[2] = ((static_cast<float>(asa_buff[2]) - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f;
            /* Set AK8963 to power down */
            VCTR::Core::printD("MPU9250 power mag down.\n");
            if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_))
            {
                akFailure_ = true;
                mpuFailure_ = true;
                VCTR::Core::printE("MPU9250 failed to set mag power down!\n");
                return false;
            }
            /* Set AK8963 to 16 bit resolution, 100 Hz update rate */
            VCTR::Core::printD("MPU9250 set mag to 16 bit and 100Hz.\n");
            if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_CNT_MEAS2_))
            {
                akFailure_ = true;
                mpuFailure_ = true;
                VCTR::Core::printE("MPU9250 failed to set mag resolution!\n");
                return false;
            }
            VCTR::Core::delay(VCTR::Core::MILLISECONDS * 100); // long wait between AK8963 mode changes
        }
        /* Select clock source to gyro */
        VCTR::Core::printD("MPU9250 set gyro clock source.\n");
        if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_))
        {
            akFailure_ = true;
            mpuFailure_ = true;
            VCTR::Core::printE("MPU9250 failed to set gyro clock source!\n");
            return false;
        }
        /* Instruct the MPU9250 to get 7 bytes from the AK8963 at the sample rate */
        /* Skip if mag failed */
        if (!akFailure_)
        {
            uint8_t mag_data[7];
            VCTR::Core::printD("MPU9250 instruct mpu to get 7 bytes from mag.\n");
            if (!ReadAk8963Registers(AK8963_HXL_, sizeof(mag_data), mag_data))
            {
                akFailure_ = true;
                mpuFailure_ = true;
                VCTR::Core::printE("MPU9250 failed to get 7 bytes from mag!\n");
                return false;
            }
        }
        /* Set the accel range to 16G by default */
        VCTR::Core::printD("MPU9250 set accel range to 16g at startup.\n");
        if (!ConfigAccelRange(ACCEL_RANGE_16G))
        {
            akFailure_ = true;
            mpuFailure_ = true;
            VCTR::Core::printE("MPU9250 failed to set accel range at startup!\n");
            return false;
        }
        /* Set the gyro range to 2000DPS by default*/
        VCTR::Core::printD("MPU9250 set gyro range to 2000dps at startup.\n");
        if (!ConfigGyroRange(GYRO_RANGE_2000DPS))
        {
            akFailure_ = true;
            mpuFailure_ = true;
            VCTR::Core::printE("MPU9250 failed to set gyro range at startup!\n");
            return false;
        }

        /* Set the DLPF to 20HZ by default */
        VCTR::Core::printD("MPU9250 set dlpf to 20Hz at startup.\n");
        if (!ConfigDlpf(DLPF_BANDWIDTH_20HZ))
        {
            akFailure_ = true;
            mpuFailure_ = true;
            VCTR::Core::printE("MPU9250 failed to set DLPF at startup!\n");
            return false;
        }
        /* Set the SRD to 0 by default */
        VCTR::Core::printD("MPU9250 set srd to 0 at startup.\n");
        if (!ConfigSrd(0))
        {
            akFailure_ = true;
            mpuFailure_ = true;
            VCTR::Core::printE("MPU9250 failed to set SRD at startup!\n");
            return false;
        }
        return true;
    }
    bool SNSR::MPU9250::EnableDrdyInt()
    {
        spi_clock_ = 1000000;
        if (!WriteRegister(INT_PIN_CFG_, INT_PULSE_50US_))
        {
            VCTR::Core::printE("MPU9250 failed to configure drdyInt!\n");
            return false;
        }
        if (!WriteRegister(INT_ENABLE_, INT_RAW_RDY_EN_))
        {
            VCTR::Core::printE("MPU9250 failed to enable drdyInt!\n");
            return false;
        }
        return true;
    }
    bool SNSR::MPU9250::DisableDrdyInt()
    {
        spi_clock_ = 1000000;
        if (!WriteRegister(INT_ENABLE_, INT_DISABLE_))
        {
            VCTR::Core::printE("MPU9250 failed to disable drdyInt!\n");
            return false;
        }
        return true;
    }
    bool SNSR::MPU9250::ConfigAccelRange(const AccelRange range)
    {
        AccelRange requested_range;
        float requested_scale;
        spi_clock_ = 1000000;
        /* Check input is valid and set requested range and scale */
        switch (range)
        {
        case ACCEL_RANGE_2G:
        {
            requested_range = range;
            requested_scale = 2.0f / 32767.5f;
            break;
        }
        case ACCEL_RANGE_4G:
        {
            requested_range = range;
            requested_scale = 4.0f / 32767.5f;
            break;
        }
        case ACCEL_RANGE_8G:
        {
            requested_range = range;
            requested_scale = 8.0f / 32767.5f;
            break;
        }
        case ACCEL_RANGE_16G:
        {
            requested_range = range;
            requested_scale = 16.0f / 32767.5f;
            break;
        }
        default:
        {
            VCTR::Core::printW("MPU9250 got incorrect Accel range command.\n");
            return false;
        }
        }
        /* Try setting the requested range */
        if (!WriteRegister(ACCEL_CONFIG_, requested_range))
        {
            VCTR::Core::printE("MPU9250 could not set requested accel range!\n");
            return false;
        }
        /* Update stored range and scale */
        accel_range_ = requested_range;
        accel_scale_ = requested_scale;
        return true;
    }
    bool SNSR::MPU9250::ConfigGyroRange(const GyroRange range)
    {
        GyroRange requested_range;
        float requested_scale;
        spi_clock_ = 1000000;
        /* Check input is valid and set requested range and scale */
        switch (range)
        {
        case GYRO_RANGE_250DPS:
        {
            requested_range = range;
            requested_scale = 250.0f / 32767.5f;
            break;
        }
        case GYRO_RANGE_500DPS:
        {
            requested_range = range;
            requested_scale = 500.0f / 32767.5f;
            break;
        }
        case GYRO_RANGE_1000DPS:
        {
            requested_range = range;
            requested_scale = 1000.0f / 32767.5f;
            break;
        }
        case GYRO_RANGE_2000DPS:
        {
            requested_range = range;
            requested_scale = 2000.0f / 32767.5f;
            break;
        }
        default:
        {
            VCTR::Core::printW("MPU9250 got incorrect gyro range command.\n");
            return false;
        }
        }
        /* Try setting the requested range */
        if (!WriteRegister(GYRO_CONFIG_, requested_range))
        {
            VCTR::Core::printE("MPU9250 could not set requested gyro range!\n");
            return false;
        }
        /* Update stored range and scale */
        gyro_range_ = requested_range;
        gyro_scale_ = requested_scale;
        return true;
    }
    bool SNSR::MPU9250::ConfigSrd(const uint8_t srd)
    {
        spi_clock_ = 1000000;
        /* Changing the SRD to allow us to set the magnetometer successfully */
        if (!WriteRegister(SMPLRT_DIV_, 19))
        {
            VCTR::Core::printE("MPU9250 could not configure Srd!\n");
            return false;
        }
        /* Set the magnetometer sample rate */
        if (srd > 9 && !akFailure_)
        {
            /* Set AK8963 to power down */
            WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);
            VCTR::Core::delay(VCTR::Core::MILLISECONDS * 100); // long wait between AK8963 mode changes
            /* Set AK8963 to 16 bit resolution, 8 Hz update rate */
            if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_CNT_MEAS1_))
            {
                return false;
            }
            VCTR::Core::delay(VCTR::Core::MILLISECONDS * 100); // long wait between AK8963 mode changes
            /* Instruct the MPU9250 to get 7 bytes from the AK8963 at the sample rate */
            uint8_t mag_data[7];
            if (!ReadAk8963Registers(AK8963_HXL_, sizeof(mag_data), mag_data))
            {
                return false;
            }
        }
        else if (!akFailure_)
        {
            /* Set AK8963 to power down */
            if (!akFailure_)
                WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);
            VCTR::Core::delay(VCTR::Core::MILLISECONDS * 100); // long wait between AK8963 mode changes
            /* Set AK8963 to 16 bit resolution, 100 Hz update rate */
            if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_CNT_MEAS2_))
            {
                return false;
            }
            VCTR::Core::delay(VCTR::Core::MILLISECONDS * 100); // long wait between AK8963 mode changes
            /* Instruct the MPU9250 to get 7 bytes from the AK8963 at the sample rate */
            uint8_t mag_data[7];
            if (!ReadAk8963Registers(AK8963_HXL_, sizeof(mag_data), mag_data))
            {
                return false;
            }
        }
        /* Set the IMU sample rate */
        if (!WriteRegister(SMPLRT_DIV_, srd))
        {
            return false;
        }
        srd_ = srd;
        return true;
    }
    bool SNSR::MPU9250::ConfigDlpf(const DlpfBandwidth dlpf)
    {
        DlpfBandwidth requested_dlpf;
        spi_clock_ = 1000000;
        /* Check input is valid and set requested dlpf */
        switch (dlpf)
        {
        case DLPF_BANDWIDTH_250HZ_4kHz:
        {
            requested_dlpf = dlpf;
            break;
        }
        case DLPF_BANDWIDTH_184HZ:
        {
            requested_dlpf = dlpf;
            break;
        }
        case DLPF_BANDWIDTH_92HZ:
        {
            requested_dlpf = dlpf;
            break;
        }
        case DLPF_BANDWIDTH_41HZ:
        {
            requested_dlpf = dlpf;
            break;
        }
        case DLPF_BANDWIDTH_20HZ:
        {
            requested_dlpf = dlpf;
            break;
        }
        case DLPF_BANDWIDTH_10HZ:
        {
            requested_dlpf = dlpf;
            break;
        }
        case DLPF_BANDWIDTH_5HZ:
        {
            requested_dlpf = dlpf;
            break;
        }
        case DLPF_BANDWIDTH_DISABLE_32kHz:
        {
            requested_dlpf = dlpf;
            break;
        }
        default:
        {
            return false;
        }
        }

        if (dlpf == DLPF_BANDWIDTH_DISABLE_32kHz)
        {

            if (!WriteRegister(ACCEL_CONFIG2_, 0b00001100))
            {
                return false;
            }
            uint8_t reg;
            if (!ReadRegisters(GYRO_CONFIG_, 1, &reg))
            {
                return false;
            }
            reg &= 0b11111100;
            reg |= 0b00000011;
            if (!WriteRegister(GYRO_CONFIG_, reg))
            {
                return false;
            }
        }
        else
        {
            /* Try setting the dlpf */
            if (!WriteRegister(ACCEL_CONFIG2_, requested_dlpf))
            {
                return false;
            }
            if (!WriteRegister(CONFIG_, requested_dlpf))
            {
                return false;
            }
        }
        /* Update stored dlpf */
        dlpf_bandwidth_ = requested_dlpf;
        return true;
    }
    bool SNSR::MPU9250::Read()
    {
        spi_clock_ = 20000000;
        /* Read the data registers */
        uint8_t data_buff[22];
        if (!ReadRegisters(INT_STATUS_, sizeof(data_buff), data_buff))
        {
            return false;
        }
        /* Check if data is ready */
        bool data_ready = (data_buff[0] & RAW_DATA_RDY_INT_);
        if (!data_ready)
        {
            return false;
        }
        /* Unpack the buffer */
        int16_t accel_counts[3], gyro_counts[3], temp_counts, mag_counts[3];
        accel_counts[0] = static_cast<int16_t>(data_buff[1]) << 8 | data_buff[2];
        accel_counts[1] = static_cast<int16_t>(data_buff[3]) << 8 | data_buff[4];
        accel_counts[2] = static_cast<int16_t>(data_buff[5]) << 8 | data_buff[6];
        temp_counts = static_cast<int16_t>(data_buff[7]) << 8 | data_buff[8];
        gyro_counts[0] = static_cast<int16_t>(data_buff[9]) << 8 | data_buff[10];
        gyro_counts[1] = static_cast<int16_t>(data_buff[11]) << 8 | data_buff[12];
        gyro_counts[2] = static_cast<int16_t>(data_buff[13]) << 8 | data_buff[14];
        mag_counts[0] = static_cast<int16_t>(data_buff[16]) << 8 | data_buff[15];
        mag_counts[1] = static_cast<int16_t>(data_buff[18]) << 8 | data_buff[17];
        mag_counts[2] = static_cast<int16_t>(data_buff[20]) << 8 | data_buff[19];
        /* Convert to float values and rotate the accel / gyro axis */
        accel_mps2_[0] = static_cast<float>(accel_counts[1]) * accel_scale_ *
                         9.80665f;
        accel_mps2_[2] = static_cast<float>(accel_counts[2]) * accel_scale_ *
                         -9.80665f;
        accel_mps2_[1] = static_cast<float>(accel_counts[0]) * accel_scale_ *
                         9.80665f;
        die_temperature_c_ = (static_cast<float>(temp_counts) - 21.0f) / temp_scale_ + 21.0f;
        gyro_radps_[1] = static_cast<float>(gyro_counts[0]) * gyro_scale_ *
                         3.14159265358979323846f / 180.0f;
        gyro_radps_[0] = static_cast<float>(gyro_counts[1]) * gyro_scale_ *
                         3.14159265358979323846f / 180.0f;
        gyro_radps_[2] = static_cast<float>(gyro_counts[2]) * gyro_scale_ *
                         -1.0f * 3.14159265358979323846f / 180.0f;
        mag_ut_[0] = static_cast<float>(mag_counts[0]) * mag_scale_[0];
        mag_ut_[1] = static_cast<float>(mag_counts[1]) * mag_scale_[1];
        mag_ut_[2] = static_cast<float>(mag_counts[2]) * mag_scale_[2];
        return true;
    }
    bool SNSR::MPU9250::WriteRegister(uint8_t reg, uint8_t data)
    {
        uint8_t ret_val;

        ioBus_->writeByte(reg, false);
        ioBus_->writeByte(data, true);

        VCTR::Core::delay(VCTR::Core::MILLISECONDS * 10);
        ReadRegisters(reg, sizeof(ret_val), &ret_val);
        VCTR::Core::delay(VCTR::Core::MILLISECONDS * 10);

        if (data == ret_val)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    bool SNSR::MPU9250::ReadRegisters(uint8_t reg, uint8_t count, uint8_t *data)
    {
        if (ioBus_->getInputType() == HAL::IO_TYPE_t::BUS_I2C)
        {
            ioBus_->writeByte(reg, true);
        }
        else
        {

            ioBus_->writeByte(reg | SPI_READ_, false);
        }

        return ioBus_->readData(data, count, true) == count;
    }
    bool SNSR::MPU9250::WriteAk8963Register(uint8_t reg, uint8_t data)
    {
        uint8_t ret_val;
        if (!WriteRegister(I2C_SLV0_ADDR_, AK8963_I2C_ADDR_))
        {
            return false;
        }
        if (!WriteRegister(I2C_SLV0_REG_, reg))
        {
            return false;
        }
        if (!WriteRegister(I2C_SLV0_DO_, data))
        {
            return false;
        }
        if (!WriteRegister(I2C_SLV0_CTRL_, I2C_SLV0_EN_ | sizeof(data)))
        {
            return false;
        }
        if (!ReadAk8963Registers(reg, sizeof(ret_val), &ret_val))
        {
            return false;
        }
        if (data == ret_val)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    bool SNSR::MPU9250::ReadAk8963Registers(uint8_t reg, uint8_t count, uint8_t *data)
    {
        if (!WriteRegister(I2C_SLV0_ADDR_, AK8963_I2C_ADDR_ | I2C_READ_FLAG_))
        {
            return false;
        }
        if (!WriteRegister(I2C_SLV0_REG_, reg))
        {
            return false;
        }
        if (!WriteRegister(I2C_SLV0_CTRL_, I2C_SLV0_EN_ | count))
        {
            return false;
        }
        VCTR::Core::delay(VCTR::Core::MILLISECONDS * 1);
        return ReadRegisters(EXT_SENS_DATA_00_, count, data);
    }

}
