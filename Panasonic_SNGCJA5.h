/* mbed Microcontroller Library for 
 * Panasonic SN-GCJA5 Particular Matter Sensor
 * Copyright (c) 2022 C Gerrish (Gerrikoio)
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef I2C_SN_GCJA5_H
#define I2C_SN_GCJA5_H

#include "mbed.h"

/** The base class for the Panasonic SN-GCJA5 PM sensor driver using I2C. */

#define SNGCJA5_ADDRESS                 (0x33u)
#define SNGCJA5_ALL                     (0x00u)
#define SNGCJA5_PM1                     (0x00u)
#define SNGCJA5_PM25                    (0x04u)
#define SNGCJA5_PM10                    (0x08u)
#define SNGCJA5_CNTA                    (0x0Cu)
#define SNGCJA5_CNTB                    (0x14u)
#define SNGCJA5_REG1                    (0x0Cu)              ///< 0.3um Particle Count
#define SNGCJA5_REG2                    (0x0Eu)              ///< 0.5um Particle Count
#define SNGCJA5_REG3                    (0x10u)              ///< 1.0um Particle Count
#define SNGCJA5_REG4                    (0x14u)              ///< 2.5um Particle Count
#define SNGCJA5_REG5                    (0x16u)              ///< 5.0um Particle Count
#define SNGCJA5_REG6                    (0x18u)              ///< 7.5um Particle Count

#define SNGCJA5_STATUS                  (0x26u)
#define TIME2FIRSTREAD                  8s
#define UNSTABLECOUNTER                 (20u)

/**! Structure holding Panasonic's PM Sensor payload data **/
// Includes mass-density (μg/m 3 ) values and counts based on size of particle
typedef struct SNGCJA5_datastruct {
  uint32_t 
      pm10_mdv,             ///< Standard PM1.0
      pm25_mdv,             ///< Standard PM2.5
      pm100_mdv;            ///< Standard PM10.0
  uint16_t 
      reg1_pc,              ///< 0.3um Particle Count
      reg2_pc,              ///< 0.5um Particle Count
      reg3_pc,              ///< 1.0um Particle Count
      reg4_pc,              ///< 2.5um Particle Count
      reg5_pc,              ///< 5.0um Particle Count
      reg6_pc;              ///< 7.5um Particle Count
} PM_MDVPC_Data;



class Panasonic_SNGCJA5
{
public:
	Panasonic_SNGCJA5(I2C &i2c, uint8_t i2cAddress):
    _i2c(i2c), _i2cAddress(i2cAddress << 1) {};

    ~Panasonic_SNGCJA5();
    
	int getData(char* reg, uint8_t *buff, uint8_t ds) {
        int readAck = 2;
        readAck = _i2c.write(_i2cAddress, reg, 1, true);
        if (readAck != 2) {                 // 2 = timeout
            wait_us(600);
            readAck = _i2c.read(_i2cAddress, (char*)buff, ds, false);

        }
        return readAck;
    }

    int getData_NonBlocking (char* reg, uint8_t *buff, uint8_t ds, const event_callback_t &cb) {
        int readAck = 2;            // Returns zero if the transfer has started, or -1 if I2C peripheral is busy
        // With this sensor only 1 byte is ever transferred before data is returned
        readAck = _i2c.transfer(_i2cAddress, reg, 1, (char*)buff, ds, cb);
        return readAck;
    }

    uint32_t convert4byte(uint8_t buff[4]) {
        uint32_t Val = (buff[0] | buff[1] <<8 | buff[2] <<16 | buff[3] <<24);
        return Val;
    }

    uint16_t convert2byte(uint8_t buff[2]) {
        uint32_t Val = (buff[0] | buff[1] <<8);
        return Val;
    }

    uint16_t calcAveCount(uint8_t buff[6]) {
        uint8_t cntArray[2] = {'\0'};
        uint16_t cntTotal = 0;
        for (uint8_t i = 0; i < 6; i+=2) {
            memcpy(cntArray, buff+i, 2);
            cntTotal += convert2byte(cntArray);
        }
        return cntTotal/3;
    }

    PM_MDVPC_Data convert2struct(uint8_t PMbuffer[26]) {
        PM_MDVPC_Data pmdata;
        pmdata.pm10_mdv = (PMbuffer[0] | PMbuffer[1]<< 8 | PMbuffer[2]<< 16 | PMbuffer[3]<< 24);
        pmdata.pm25_mdv = (PMbuffer[4] | PMbuffer[5]<< 8 | PMbuffer[6]<< 16 | PMbuffer[7]<< 24);
        pmdata.pm100_mdv = (PMbuffer[8] | PMbuffer[9]<< 8 | PMbuffer[10]<< 16 | PMbuffer[11]<< 24);
        pmdata.reg1_pc = (PMbuffer[12] | PMbuffer[13]<< 8);
        pmdata.reg2_pc = (PMbuffer[14] | PMbuffer[15]<< 8);
        pmdata.reg3_pc = (PMbuffer[16] | PMbuffer[17]<< 8);
        //gap
        pmdata.reg4_pc = (PMbuffer[20] | PMbuffer[21]<< 8);
        pmdata.reg5_pc = (PMbuffer[22] | PMbuffer[23]<< 8);
        pmdata.reg6_pc = (PMbuffer[24] | PMbuffer[25]<< 8);

        return pmdata;
    }
	
    
protected:
	// the memory buffer for the sensor
    I2C &_i2c;
    uint8_t _i2cAddress;
};

#endif // I2C_SN_GCJA5_H

