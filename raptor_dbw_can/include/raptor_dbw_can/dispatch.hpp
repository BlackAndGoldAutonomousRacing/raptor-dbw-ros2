// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RAPTOR_DBW_CAN__DISPATCH_HPP_
#define RAPTOR_DBW_CAN__DISPATCH_HPP_

#include <stdint.h>

#include "raptor_dbw_can/canid_enum.h"

namespace raptor_dbw_can
{
/** \brief Enumeration of VIN multiplex control values */
typedef enum
{
  VIN_MUX_VIN0  = 0x00,   /**< VIN mux value = 0 */
  VIN_MUX_VIN1  = 0x01,   /**< VIN mux value = 1 */
  VIN_MUX_VIN2  = 0x02,   /**< VIN mux value = 2 */
} VinMux;

/** \brief Enumeration of wheel speed multiplex control values */
typedef enum
{
  WHEEL_SPEED_MUX0  = 0x00,   /**< Wheel speed mux value = 0 */
  WHEEL_SPEED_MUX1  = 0x01,   /**< Wheel speed mux value = 1 */
  WHEEL_SPEED_MUX2  = 0x02,   /**< Wheel speed mux value = 2 */
} WheelSpeedMux;

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__DISPATCH_HPP_
