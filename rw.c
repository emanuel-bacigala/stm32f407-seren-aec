/*
 * Copyright (C) 2013, 2014 Giorgio Vazzana
 *
 * These functions has been taken form Seren.
 *
 * Seren is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Seren is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdint.h>

uint16_t read_be16(const uint8_t *p) {
    return (uint16_t)(((uint32_t)p[0] << 8) | (uint32_t)p[1]);
}

void write_be16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >>  8);
    p[1] = (uint8_t)(v      );
}


uint32_t read_be32(const uint8_t *p)
{
    return (((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3]);
}

void write_be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >>  8);
    p[3] = (uint8_t)(v      );
}
