/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
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
#include "i2c_api.h"

#include "cmsis.h"
#include "pinmap.h"
#include "error.h"

static const PinMap PinMap_I2C_SDA[] = {
    {NC  ,  NC   , 0}
};

static const PinMap PinMap_I2C_SCL[] = {
    {NC  ,  NC,    0}
};

static const uint16_t ICR[0x40] = {
      20,   22,   24,   26,   28,
      30,   34,   40,   28,   32,
      36,   40,   44,   48,   56,
      68,   48,   56,   64,   72,
      80,   88,  104,  128,   80,
      96,  112,  128,  144,  160,
      192,  240,  160,  192,  224,
      256,  288,  320,  384,  480,
      320,  384,  448,  512,  576,
      640,  768,  960,  640,  768,
      896, 1024, 1152, 1280, 1536,
      1920, 1280, 1536, 1792, 2048,
      2304, 2560, 3072, 3840
};

static uint8_t first_read;


void i2c_init(i2c_t *obj, PinName sda, PinName scl) {

}

int i2c_start(i2c_t *obj) {
    return 1;
}

int i2c_stop(i2c_t *obj) {
    return 1;
}

static int timeout_status_poll(i2c_t *obj, uint32_t mask) {
    return 1;
}

// this function waits the end of a tx transfer and return the status of the transaction:
//    0: OK ack received
//    1: OK ack not received
//    2: failure
static int i2c_wait_end_tx_transfer(i2c_t *obj) {

}

// this function waits the end of a rx transfer and return the status of the transaction:
//    0: OK
//    1: failure
static int i2c_wait_end_rx_transfer(i2c_t *obj) {
    return 1;
}

static void i2c_send_nack(i2c_t *obj) {

}

static void i2c_send_ack(i2c_t *obj) {

}

static int i2c_do_write(i2c_t *obj, int value) {
    return 1;
}

static int i2c_do_read(i2c_t *obj, char * data, int last) {
    return 1;
}

void i2c_frequency(i2c_t *obj, int hz) {

}

int i2c_read(i2c_t *obj, int address, char *data, int length, int stop) {
    return 1;
}

int i2c_write(i2c_t *obj, int address, const char *data, int length, int stop) {
    return 1;
}

void i2c_reset(i2c_t *obj) {
    i2c_stop(obj);
}

int i2c_byte_read(i2c_t *obj, int last) {
    return 1;
}

int i2c_byte_write(i2c_t *obj, int data) {
    return 1;
}


#if DEVICE_I2CSLAVE
void i2c_slave_mode(i2c_t *obj, int enable_slave) {

}

int i2c_slave_receive(i2c_t *obj) {
    return 1;
}

int i2c_slave_read(i2c_t *obj, char *data, int length) {
    return 1;
}

int i2c_slave_write(i2c_t *obj, const char *data, int length) {
    return 1;
}

void i2c_slave_address(i2c_t *obj, int idx, uint32_t address, uint32_t mask) {

}
#endif

