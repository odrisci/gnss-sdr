/*!
 * \file gnss_time_converter.h
 * \brief Time Conversion Utilities in GNSS-SDR
 * \author Cillian O'Driscoll, cillian.odriscoll(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_TIME_CONVERTER_H_
#define GNSS_SDR_TIME_CONVERTER_H_

#include "gnss_time.h"

/// Forward declaration:
class TimeConverterImpl;

class TimeConverter
{
private:
    // Disable visual studio warning C4251 about
    // making the definition of TimeConverterImpl public
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4251)
#endif
    /// Private implementation class
    std::unique_ptr<TimeConverterImpl> mImpl;
#ifdef _MSC_VER
#pragma warning(pop)
#endif
public:
    TimeConverter();
    ~TimeConverter();
    std::pair<bool, TimePoint> Convert(TimePoint in, ClockID out_type);

    bool AddLeapSecondsAt(TimePoint leap_epoch, int num_leaps);

    void SetReceiverEpoch(uint32_t rx_id, TimePoint epoch);
};

#endif

