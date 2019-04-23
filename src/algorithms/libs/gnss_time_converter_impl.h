/*!
 * \file gnss_time_converter_impl.h
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

#ifndef GNSS_SDR_TIME_CONVERTER_IMPL_H_
#define GNSS_SDR_TIME_CONVERTER_IMPL_H_

#include "gnss_time_converter.h"
#include <vector>

class TimeConverterImpl
{
private:
    struct leap_second_entry_t
    {
        TimeInterval ntp_epoch;
        int num_leap_seconds;
        leap_second_entry_t(TimeInterval ntp_epoch_, int num_leap_seconds_)
            : ntp_epoch(ntp_epoch_), num_leap_seconds(num_leap_seconds_)
        {
        }
    };

    struct epoch_offset_entry_t
    {
        ClockID sys;
        TimeInterval offset;

        epoch_offset_entry_t(ClockID sys_, TimeInterval offset_)
            : sys(sys_),
              offset(offset_)
        {
        }
    };

    std::vector<leap_second_entry_t> d_leap_second_table;
    std::vector<epoch_offset_entry_t> d_epoch_offset_table;

    std::pair<bool, TimePoint> ConvertNoLeaps(TimePoint in, ClockID out_type);

public:
    TimeConverterImpl();
    std::pair<bool, TimePoint> Convert(TimePoint in, ClockID out_type);

    bool AddLeapSecondsAt(TimePoint leap_epoch, int num_leaps);

    void SetReceiverEpoch(uint32_t rx_id, TimePoint epoch);
};

#endif

