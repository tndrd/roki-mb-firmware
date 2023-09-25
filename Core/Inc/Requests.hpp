/*
 * Requests.hpp
 *
 *  Created on: Sep 22, 2023
 *      Author: tndrd
 */

#ifndef INC_REQUESTS_HPP_
#define INC_REQUESTS_HPP_

namespace BodyMessages {

static constexpr uint8_t KondoNack[] = { 0x4, 0xFE, 0x15, 0x17 };
static constexpr uint8_t ServoPosRequest[] = {0xA, 0, 0x20, 0, 0, 0, 0x70, 0, 0x1E, 0xB8};
static constexpr uint8_t ServoPosResponceSize = 33;

}

#endif /* INC_REQUESTS_HPP_ */
