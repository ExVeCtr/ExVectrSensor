
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


#ifndef EXVECTRSENSOR_MPU9250_H
#define EXVECTRSENSOR_MPU9250_H

#include "ExVectrCore/print.hpp"

#include "ExVectrHAL/io_types.hpp"
#include "ExVectrHAL/io_params.hpp"
#include "ExVectrHAL/io.hpp"

#include "../gyroscope.hpp"
#include "../accelerometer.hpp"
#include "../magnetometer.hpp"

namespace VCTR
{

    namespace SNSR
    {

        /**
         * @brief A class implementing control for the MPU9250 9dof sensor. The MPU9250 has a Gyroscope, accelerometer and magnetometer.
         * @note Currently the magnetometer is not implemented.
         */
        class MPU9250 : public Gyroscope, public Accelerometer, public Magnetometer
        {
        private:

            HAL::IO* ioBus_ = nullptr;

            bool initialised_ = false;

        public:

            enum DlpfBandwidth : uint8_t {
                DLPF_BANDWIDTH_250HZ_4kHz = 0x00,
                DLPF_BANDWIDTH_184HZ = 0x01,
                DLPF_BANDWIDTH_92HZ = 0x02,
                DLPF_BANDWIDTH_41HZ = 0x03,
                DLPF_BANDWIDTH_20HZ = 0x04,
                DLPF_BANDWIDTH_10HZ = 0x05,
                DLPF_BANDWIDTH_5HZ = 0x06,
                DLPF_BANDWIDTH_DISABLE_32kHz = 0xAA
            };
            enum AccelRange : uint8_t {
                ACCEL_RANGE_2G = 0x00,
                ACCEL_RANGE_4G = 0x08,
                ACCEL_RANGE_8G = 0x10,
                ACCEL_RANGE_16G = 0x18
            };
            enum GyroRange : uint8_t {
                GYRO_RANGE_250DPS = 0x00,
                GYRO_RANGE_500DPS = 0x08,
                GYRO_RANGE_1000DPS = 0x10,
                GYRO_RANGE_2000DPS = 0x18
            };

            /**
             * @brief Initialises sensor on the given IO bus.
             * @param ioBus Input/Output bus the sensor is connected to. Should be a single connection not true bus.
             * @return true if initialisation successfull.
             */
            bool initSensor(HAL::IO &ioBus);

            /**
             * @brief Implements reading gyro values and publishing them to the topic.
             * @return true if successful, false otherwise.
             */
            bool readGyro() override;

            /**
             * @brief Implements reading accel values and publishing them to the topic.
             * @return true if successful, false otherwise.
             */
            bool readAccel() override;

            /**
             * @brief Implements reading mag values and publishing them to the topic.
             * @note Currently not implemented.
             * @return true if successful, false otherwise.
             */
            bool readMag() override;

            /**
             * @brief Makes sensor read and publish gyro, accel and magnetometer values.
             * @returns true if all could be read, false if one or more failed.
             */
            bool readAll();

        private:

            /**
             * Below are all registers and extra functions.
            */
            bool Begin(bool disableMag = false);
            bool MagnetometerFailed() {return akFailure_;}
            bool IMUFailed() {return mpuFailure_ || akFailure_;}
            bool EnableDrdyInt();
            bool DisableDrdyInt();
            bool ConfigAccelRange(const AccelRange range);
            inline AccelRange accel_range() const {return accel_range_;}
            bool ConfigGyroRange(const GyroRange range);
            inline GyroRange gyro_range() const {return gyro_range_;}
            bool ConfigSrd(const uint8_t srd);
            inline uint8_t srd() const {return srd_;}
            bool ConfigDlpf(const DlpfBandwidth dlpf);
            inline DlpfBandwidth dlpf() const {return dlpf_bandwidth_;}
            bool Read();
            inline float accel_x_mps2() const {return accel_mps2_[0];}
            inline float accel_y_mps2() const {return accel_mps2_[1];}
            inline float accel_z_mps2() const {return accel_mps2_[2];}
            inline float gyro_x_radps() const {return gyro_radps_[0];}
            inline float gyro_y_radps() const {return gyro_radps_[1];}
            inline float gyro_z_radps() const {return gyro_radps_[2];}
            inline float mag_x_ut() const {return mag_ut_[0];}
            inline float mag_y_ut() const {return mag_ut_[1];}
            inline float mag_z_ut() const {return mag_ut_[2];}
            inline float die_temperature_c() const {return die_temperature_c_;}

            /* Configuration */
            static constexpr uint8_t SPI_READ_ = 0x80;
            size_t spi_clock_ = 1000000;
            AccelRange accel_range_;
            GyroRange gyro_range_;
            DlpfBandwidth dlpf_bandwidth_;
            uint8_t srd_;
            bool akFailure_ = false;                               //################ Keep track of the AK8963 in case something fails.
            bool mpuFailure_ = false;                               //################ Keep track of the MPU6500 in case something fails.
            static constexpr uint8_t WHOAMI_MPU9250_ = 0x71;
            static constexpr uint8_t WHOAMI_MPU9255_ = 0x73;
            static constexpr uint8_t WHOAMI_AK8963_ = 0x48;
            /* Data */
            float accel_scale_, gyro_scale_, mag_scale_[3];
            float temp_scale_ = 333.87f;
            float accel_mps2_[3];
            float gyro_radps_[3];
            float mag_ut_[3];
            float die_temperature_c_;
            /* Registers */
            static constexpr uint8_t PWR_MGMNT_1_ = 0x6B;
            static constexpr uint8_t H_RESET_ = 0x80;
            static constexpr uint8_t CLKSEL_PLL_ = 0x01;
            static constexpr uint8_t WHOAMI_ = 0x75;
            static constexpr uint8_t ACCEL_CONFIG_ = 0x1C;
            static constexpr uint8_t GYRO_CONFIG_ = 0x1B;
            static constexpr uint8_t ACCEL_CONFIG2_ = 0x1D;
            static constexpr uint8_t CONFIG_ = 0x1A;
            static constexpr uint8_t SMPLRT_DIV_ = 0x19;
            static constexpr uint8_t INT_PIN_CFG_ = 0x37;
            static constexpr uint8_t INT_ENABLE_ = 0x38;
            static constexpr uint8_t INT_DISABLE_ = 0x00;
            static constexpr uint8_t INT_PULSE_50US_ = 0x00;
            static constexpr uint8_t INT_RAW_RDY_EN_ = 0x01;
            static constexpr uint8_t INT_STATUS_ = 0x3A;
            static constexpr uint8_t RAW_DATA_RDY_INT_ = 0x01;
            static constexpr uint8_t USER_CTRL_ = 0x6A;
            static constexpr uint8_t I2C_MST_EN_ = 0x20;
            static constexpr uint8_t I2C_MST_CLK_ = 0x0D;
            static constexpr uint8_t I2C_MST_CTRL_ = 0x24;
            static constexpr uint8_t I2C_SLV0_ADDR_ = 0x25;
            static constexpr uint8_t I2C_SLV0_REG_ = 0x26;
            static constexpr uint8_t I2C_SLV0_CTRL_ = 0x27;
            static constexpr uint8_t I2C_SLV0_DO_ = 0x63;
            static constexpr uint8_t I2C_READ_FLAG_ = 0x80;
            static constexpr uint8_t I2C_SLV0_EN_ = 0x80;
            static constexpr uint8_t EXT_SENS_DATA_00_ = 0x49;
            /* AK8963 registers */
            static constexpr uint8_t AK8963_I2C_ADDR_ = 0x0C;
            static constexpr uint8_t AK8963_HXL_ = 0x03;
            static constexpr uint8_t AK8963_CNTL1_ = 0x0A;
            static constexpr uint8_t AK8963_PWR_DOWN_ = 0x00;
            static constexpr uint8_t AK8963_CNT_MEAS1_ = 0x12;
            static constexpr uint8_t AK8963_CNT_MEAS2_ = 0x16;
            static constexpr uint8_t AK8963_FUSE_ROM_ = 0x0F;
            static constexpr uint8_t AK8963_CNTL2_ = 0x0B;
            static constexpr uint8_t AK8963_RESET_ = 0x01;
            static constexpr uint8_t AK8963_ASA_ = 0x10;
            static constexpr uint8_t AK8963_WHOAMI_ = 0x00;
            bool WriteRegister(uint8_t reg, uint8_t data);
            bool ReadRegisters(uint8_t reg, uint8_t count, uint8_t *data);
            bool WriteAk8963Register(uint8_t reg, uint8_t data);
            bool ReadAk8963Registers(uint8_t reg, uint8_t count, uint8_t *data);

        };

    }

}

#endif