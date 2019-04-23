/*!
 * \file gnss_time_converter.cc
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

#include "gnss_time_converter_impl.h"
#include <algorithm>
#include <iostream>

TimeConverter::TimeConverter() : mImpl(new TimeConverterImpl()) {}

TimeConverter::~TimeConverter() = default;

std::pair<bool, TimePoint> TimeConverter::Convert(TimePoint in, ClockID out_type)
{
    return mImpl->Convert( in, out_type );
}

bool TimeConverter::AddLeapSecondsAt(TimePoint leap_epoch, int num_leaps)
{
    return mImpl->AddLeapSecondsAt( leap_epoch, num_leaps );
}

void TimeConverter::SetReceiverEpoch(uint32_t rx_id, TimePoint epoch)
{
    mImpl->SetReceiverEpoch( rx_id, epoch );
}

TimeConverterImpl::TimeConverterImpl()
{
    // Create the leap second table
    // NTP epoch is 1900-JAN-01 00:00:00 UTC
    // Unix epoch is 1970-JAN-01 00:00:00 UTC : 70 years of which 17 are leap years later
    static const TimeInterval unix_epoch_offset = TimeInterval::Years(70) + TimeInterval::Days(17);
    // GPS epoch is 1980-JAN-06 00:00:00 UTC : 10 years of which 2 are leap years and 5 days later + 19 leap seconds
    // Glonass epoch is ill defined, using GPS epoch
    static const TimeInterval gps_epoch_offset = unix_epoch_offset + TimeInterval::Years(10) + TimeInterval::Days(2)
        + TimeInterval::Days(5) + TimeInterval::Seconds(19);
    // Galileo epoch is 1024 weeks after GPS
    static const TimeInterval gal_epoch_offset = gps_epoch_offset + TimeInterval::Weeks(1024);
    // BeiDou epoch is 2006-JAN-01 00:00:00 UTC : 26 years - 5 days + 14 extra leap seconds after GPS
    static const TimeInterval beidou_epoch_offset = gps_epoch_offset + TimeInterval::Years(26)
        + TimeInterval::Days(7) - TimeInterval::Days(5) + TimeInterval::Seconds(14);

    // Create the epoch offset table (all relaitve to GPS)
    d_epoch_offset_table.push_back(
        epoch_offset_entry_t(ClockID::MakeGnss(GnssSystem::kGps),
            gps_epoch_offset));
    // galileo epoch is 1024 weeks after GPS, but occurs at the GPS weekly epoch,
    // not the UTC weekly epoch (13 seconds difference)
    d_epoch_offset_table.push_back(
        epoch_offset_entry_t(ClockID::MakeGnss(GnssSystem::kGalileo),
            gal_epoch_offset));
    d_epoch_offset_table.push_back(
        epoch_offset_entry_t(ClockID::MakeGnss(GnssSystem::kGlonass),
            gps_epoch_offset));
    d_epoch_offset_table.push_back(
        epoch_offset_entry_t(ClockID::MakeGnss(GnssSystem::kBeiDou),
            beidou_epoch_offset));

    // Non-GNSS times-scales:
    // Unix Epoch is 01-Jan-1970
    d_epoch_offset_table.push_back(
        epoch_offset_entry_t(ClockID::MakeUnix(),
            unix_epoch_offset));
    //
    // Treat UTC Epoch as same as Unix
    d_epoch_offset_table.push_back(
        epoch_offset_entry_t(ClockID::MakeUTC(),
            unix_epoch_offset));

    // NTP Epoch is Jan 1 1900
    d_epoch_offset_table.push_back(
        epoch_offset_entry_t(ClockID::MakeNTP(),
            TimeInterval::Seconds(0.0) ));
    
    // Treat TAI like NTP
    d_epoch_offset_table.push_back(
        epoch_offset_entry_t(ClockID::MakeTAI(),
            TimeInterval::Seconds(0.0) ));

    int64_t leap_epochs[] = {
        2272060800LL,
        2287785600LL,
        2303683200LL,
        2335219200LL,
        2366755200LL,
        2398291200LL,
        2429913600LL,
        2461449600LL,
        2492985600LL,
        2524521600LL,
        2571782400LL,
        2603318400LL,
        2634854400LL,
        2698012800LL,
        2776982400LL,
        2840140800LL,
        2871676800LL,
        2918937600LL,
        2950473600LL,
        2982009600LL,
        3029443200LL,
        3076704000LL,
        3124137600LL,
        3345062400ll,
        3439756800ll,
        3550089600ll,
        3644697600ll,
        3692217600ll};

    int num_leaps = 10;

    for (int i = 0; i < sizeof(leap_epochs)/sizeof(leap_epochs[0]); ++i)
        {
            AddLeapSecondsAt(
                TimePoint( ClockID::MakeNTP(), 
                    TimeInterval::Seconds(leap_epochs[i])),
                num_leaps++);
        }
}

std::pair<bool, TimePoint> TimeConverterImpl::ConvertNoLeaps(TimePoint in, ClockID out_sys)
{
    TimePoint ret( out_sys, in.TimeSinceEpoch() );
    ClockID in_sys = in.GetClockID();

    if (in_sys == out_sys)
        {
            return std::make_pair(true, in);
        }

    // First thing we do is add the relative offset of the epoch if we have that:
    auto ip_offset_itr = std::find_if(d_epoch_offset_table.begin(),
        d_epoch_offset_table.end(),
        [in_sys](epoch_offset_entry_t v) { return v.sys == in_sys; });

    auto op_offset_itr = std::find_if(d_epoch_offset_table.begin(),
        d_epoch_offset_table.end(),
        [out_sys](epoch_offset_entry_t v) {
            return v.sys == out_sys;
        });

    if (ip_offset_itr == d_epoch_offset_table.end() ||
        op_offset_itr == d_epoch_offset_table.end())
        {
            return std::make_pair(false, ret);
        }

    TimeInterval epoch_delta = ip_offset_itr->offset - op_offset_itr->offset;

    return std::make_pair(true, ret + epoch_delta);
}

std::pair<bool, TimePoint> TimeConverterImpl::Convert(TimePoint in, ClockID out_sys)
{
    std::pair<bool, TimePoint> ret_pair = ConvertNoLeaps(in, out_sys);

    ClockID in_sys = in.GetClockID();

    if (ret_pair.first == false ||
        (in_sys.KeepsLeapSeconds() == out_sys.KeepsLeapSeconds()))
        {
            return ret_pair;
        }

    // If we get to here then we have converted but we need to update for
    // the difference in leap seconds between in_sys and out_sys
    // Need to find the number of leap seconds at the epoch

    std::pair<bool, TimePoint> ntp_pair = ConvertNoLeaps(in, ClockID::MakeNTP());
    if (ntp_pair.first == false)
        {
            ret_pair.first = false;
            return ret_pair;
        }

    TimeInterval seconds_since_ntp_epoch = ntp_pair.second.TimeSinceEpoch();
    auto leap_itr = std::find_if(
        d_leap_second_table.begin(),
        d_leap_second_table.end(),
        [seconds_since_ntp_epoch](auto v) {
            return v.ntp_epoch < seconds_since_ntp_epoch;
        });

    TimeInterval leaps_at_epoch = TimeInterval::Seconds(0);
    if( leap_itr != d_leap_second_table.end() )
    {
        leaps_at_epoch = TimeInterval::Seconds(leap_itr->num_leap_seconds);

        if( out_sys.KeepsLeapSeconds() )
        {
            ret_pair.second -= leaps_at_epoch;
        }
        else
        {
            ret_pair.second += leaps_at_epoch;
        }
    }
    return ret_pair;

}

bool TimeConverterImpl::AddLeapSecondsAt(TimePoint leap_epoch, int num_leaps)
{
    std::pair< bool, TimePoint > ntp_pair = Convert( leap_epoch, ClockID::MakeNTP() );

    if( not ntp_pair.first )
    {
        return false;
    }

    TimeInterval ntp_epoch = ntp_pair.second.TimeSinceEpoch();

    if( d_leap_second_table.size() > 0 && ntp_epoch <= d_leap_second_table.front().ntp_epoch )
    {
        // Not adding this leap second, can only add later leap seconds
        return false;
    }
    d_leap_second_table.insert( d_leap_second_table.begin(),
            leap_second_entry_t( ntp_epoch, num_leaps ) );

    return true;

}

void TimeConverterImpl::SetReceiverEpoch(uint32_t rx_id, TimePoint epoch)
{
    ClockID rx_sys = ClockID::MakeReceiver( rx_id );

    std::pair< bool, TimePoint > tai_epoch_pair = Convert( epoch, ClockID::MakeTAI() );

    if( tai_epoch_pair.first )
    {
        auto offset_entry = epoch_offset_entry_t( rx_sys, 
                tai_epoch_pair.second.TimeSinceEpoch() );

        auto epoch_offset_iter = std::find_if(
                d_epoch_offset_table.begin(),
                d_epoch_offset_table.end(),
                [rx_sys]( auto v )
                {
                    return v.sys == rx_sys;
                });

        if( epoch_offset_iter == d_epoch_offset_table.end() )
        {
            d_epoch_offset_table.push_back( offset_entry );
        }
        else
        {
            *epoch_offset_iter = offset_entry;
        }
    }
}
