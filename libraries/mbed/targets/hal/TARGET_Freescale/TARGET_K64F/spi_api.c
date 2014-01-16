/* mbed Microcontroller Library
 * Copyright (c) 2013 ARM Limited
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
#include "spi_api.h"

#include <math.h>

#include "cmsis.h"
#include "pinmap.h"
#include "error.h"

static const PinMap PinMap_SPI_SCLK[] = {

    {NC  , NC   , 0}
};

static const PinMap PinMap_SPI_MOSI[] = {

    {NC  , NC   , 0}
};

static const PinMap PinMap_SPI_MISO[] = {

    {NC  , NC   , 0}
};

static const PinMap PinMap_SPI_SSEL[] = {

    {NC  , NC   , 0}
};

void spi_init(spi_t *obj, PinName mosi, PinName miso, PinName sclk, PinName ssel) {

}

void spi_free(spi_t *obj) {
    // [TODO]
}
void spi_format(spi_t *obj, int bits, int mode, int slave) {

}

void spi_frequency(spi_t *obj, int hz) {

}

static inline int spi_writeable(spi_t * obj) {
}

static inline int spi_readable(spi_t * obj) {
}

int spi_master_write(spi_t *obj, int value) {

}

int spi_slave_receive(spi_t *obj) {

}

int spi_slave_read(spi_t *obj) {

}

void spi_slave_write(spi_t *obj, int value) {

}
