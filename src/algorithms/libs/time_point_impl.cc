/*!
 * \file TimePoint_impl.cc
 * \brief Private implementation of the TimePoint class
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

#include "time_point_impl.h"
#include "gnss_time.h"
#include "gnss_time_converter.h"
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>

TimePointImpl::TimePointImpl(ClockID const &clock_id)
    : mTimeSinceEpoch(TimeInterval::Weeks(0)), mClockID(clock_id)
{
}

TimePointImpl::TimePointImpl(ClockID const &clock_id,
    TimeInterval const &timeInterval)
    : mTimeSinceEpoch(timeInterval), mClockID(clock_id)
{
}

ClockID const &TimePointImpl::GetClockID(void) const { return mClockID; }

int TimePointImpl::Week(void) const { return mTimeSinceEpoch.AsWeeks(); }

TimeInterval TimePointImpl::TOW(void) const
{
    // Return the number of seconds since the start of this week:
    return (mTimeSinceEpoch - TimeInterval::Weeks(Week()));
}

TimeInterval TimePointImpl::TimeSinceEpoch(void) const
{
    return mTimeSinceEpoch;
}

TimePointImpl &TimePointImpl::operator=(TimePointImpl const &rhs)
{
    mClockID = rhs.mClockID;
    mTimeSinceEpoch = rhs.mTimeSinceEpoch;
    return *this;
}

TimePointImpl &TimePointImpl::operator+=(TimeInterval const &rhs)
{
    mTimeSinceEpoch += rhs;
    return *this;
}

TimePointImpl &TimePointImpl::operator-=(TimeInterval const &rhs)
{
    mTimeSinceEpoch -= rhs;
    return *this;
}

TimeInterval operator-(TimePointImpl const &lhs, TimePointImpl const &rhs)
{
    /// TODO: cillian Find a better way of doing this
    if (lhs.mClockID != rhs.mClockID)
        {
            throw "Incompatible clock_idtems";
        }
    return lhs.mTimeSinceEpoch - rhs.mTimeSinceEpoch;
}

TimePointImpl operator-(TimePointImpl lhs, TimeInterval const &rhs)
{
    lhs -= rhs;
    return lhs;
}

TimePointImpl operator+(TimePointImpl lhs, TimeInterval const &rhs)
{
    lhs += rhs;
    return lhs;
}

bool operator==(TimePointImpl const &lhs, TimePointImpl const &rhs)
{
    /// TODO: Find a better way of doing this - use a clock_idtem comparator? [ default
    /// assumes no clock drift between clock_idtem?]
    return (lhs.mClockID == rhs.mClockID &&
            lhs.mTimeSinceEpoch == rhs.mTimeSinceEpoch);
}

bool operator!=(TimePointImpl const &lhs, TimePointImpl const &rhs)
{
    return !(lhs == rhs);
}

bool operator<(TimePointImpl const &lhs, TimePointImpl const &rhs)
{
    /// TODO: Find a better way of doing this:
    return (lhs.mClockID == rhs.mClockID &&
            lhs.mTimeSinceEpoch < rhs.mTimeSinceEpoch);
}

bool operator>(TimePointImpl const &lhs, TimePointImpl const &rhs)
{
    return rhs < lhs;
}

bool operator<=(TimePointImpl const &lhs, TimePointImpl const &rhs)
{
    return !(lhs > rhs);
}

bool operator>=(TimePointImpl const &lhs, TimePointImpl const &rhs)
{
    return !(rhs > lhs);
}

TimePoint::TimePoint() : mImpl(new TimePointImpl(ClockID::MakeReceiver())) {}

TimePoint::TimePoint(ClockID const &clock_id) : mImpl(new TimePointImpl(clock_id)) {}

TimePoint::TimePoint(ClockID const &clock_id, TimeInterval const &timeInterval)
    : mImpl(new TimePointImpl(clock_id, timeInterval))
{
}

// This copy constructor ensures we do a deep copy of the mImpl member
TimePoint::TimePoint(const TimePoint &rhs)
    : mImpl(std::unique_ptr<TimePointImpl>(new TimePointImpl(*(rhs.mImpl))))
{
    // Handled in initialiser
}

#if defined(__cplusplus) && (__cplusplus >= 201103L)
// This copy constructor works with temporary variables, allowing us to
// simply re-use the mImpl created in the temporary variable
TimePoint::TimePoint(TimePoint &&rhs) : mImpl(std::move(rhs.mImpl))
{
    // Handled in initialiser
}
#endif
TimePoint::~TimePoint() = default;

ClockID TimePoint::GetClockID(void) const { return mImpl->GetClockID(); }

int TimePoint::Week(void) const { return mImpl->Week(); }

TimeInterval TimePoint::TOW(void) const { return mImpl->TOW(); }

TimeInterval TimePoint::TimeSinceEpoch(void) const { return mImpl->TimeSinceEpoch(); }

TimePoint TimePoint::MakeGnss(GnssSystem sys, TimeInterval time_interval)
{
    ClockID clockID = ClockID::MakeGnss(sys);

    return TimePoint{clockID, time_interval};
}

TimePoint TimePoint::MakeGnss(GnssSystem sys, int week_number, double TOW)
{
    return TimePoint::MakeGnss(sys, TimeInterval::Weeks(week_number) +
                                        TimeInterval::Seconds(TOW));
}

TimePoint TimePoint::MakeReceiver(TimeInterval time_interval, uint32_t r_id)
{
    ClockID clockID = ClockID::MakeReceiver(r_id);
    return TimePoint(clockID, time_interval);
}

TimePoint TimePoint::MakeReceiver(int64_t sample_count, double sample_rate, uint32_t r_id)
{
    return TimePoint::MakeReceiver(TimeInterval::Ticks(sample_count, sample_rate), r_id);
}

TimePoint TimePoint::GetCurrentUnix(void)
{
    ClockID clk_id = ClockID::MakeUnix();

    auto now = std::chrono::high_resolution_clock::now();

    auto t_seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
    auto t_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>((now - t_seconds).time_since_epoch());

    return TimePoint(clk_id, TimeInterval::Seconds(t_seconds.count()) + TimeInterval::NanoSeconds(t_nanoseconds.count()));
}


time_t mktime_utc( std::tm * tm_utc )
{
    static std::tm tm0 = { 0, 0, 0, 1, 0, 80, 0, 0, 0 };
    static time_t utc_off = std::mktime(&tm0) - 315532800;

    return std::mktime(tm_utc) - utc_off;
}

TimePoint TimePoint::GetCurrentUTC(void)
{
    std::time_t raw_time;
    std::time( &raw_time );

    std::tm *ptm = gmtime( &raw_time );

    return MakeUTC( ptm->tm_year + 1900, static_cast<eMonth>(ptm->tm_mon),
            ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec );
}

TimePoint TimePoint::MakeUTC( int year, eMonth month, int day, int hour, 
        int minute, int seconds )
{
    std::tm time_as_tm;
    time_as_tm.tm_year = year - 1900;
    time_as_tm.tm_mon = static_cast<int>(month);
    time_as_tm.tm_mday = day;
    time_as_tm.tm_hour = hour;
    time_as_tm.tm_min = minute;
    time_as_tm.tm_sec = seconds;
    time_as_tm.tm_isdst = 0;

    std::time_t ct = mktime_utc( &time_as_tm );

    return TimePoint( ClockID::MakeUTC(), TimeInterval::Seconds(ct) );
}

TimeConverter &TimePoint::GetConverter(void)
{
    static TimeConverter converter;
    return converter;
}

TimePoint &TimePoint::operator=(TimePoint const &rhs)
{
    *mImpl = *(rhs.mImpl);
    return *this;
}

TimePoint &TimePoint::operator+=(TimeInterval const &rhs)
{
    *mImpl += rhs;
    return *this;
}

TimePoint &TimePoint::operator-=(TimeInterval const &rhs)
{
    *mImpl -= rhs;
    return *this;
}

TimeInterval operator-(TimePoint lhs, TimePoint const &rhs)
{
    return *(lhs.mImpl) - *(rhs.mImpl);
}

TimePoint operator-(TimePoint lhs, TimeInterval const &rhs)
{
    lhs -= rhs;
    return lhs;
}

TimePoint operator+(TimePoint lhs, TimeInterval const &rhs)
{
    lhs += rhs;
    return lhs;
}

bool operator==(TimePoint const &lhs, TimePoint const &rhs)
{
    return *(lhs.mImpl) == *(rhs.mImpl);
}

bool operator!=(TimePoint const &lhs, TimePoint const &rhs)
{
    return !(lhs == rhs);
}

bool operator<(TimePoint const &lhs, TimePoint const &rhs)
{
    return *(lhs.mImpl) < *(rhs.mImpl);
}

bool operator>(TimePoint const &lhs, TimePoint const &rhs)
{
    return rhs < lhs;
}

bool operator<=(TimePoint const &lhs, TimePoint const &rhs)
{
    return !(lhs > rhs);
}

bool operator>=(TimePoint const &lhs, TimePoint const &rhs)
{
    return !(rhs > lhs);
}

std::ostream &operator<<(std::ostream &os, TimePoint const &rhs)
{
    os << rhs.GetClockID() << " ";
    if( rhs.GetClockID().IsGnss() )
    {
        os << "Week: " << rhs.Week();
        os << " TOW: " << rhs.TOW();
    }
    else if( rhs.GetClockID().GetSystem() == ClockID::eClockSystem::kReceiver )
    {
        os << rhs.TimeSinceEpoch();
    }
    else
    {
        // We have a non-GNSS non-receiver time system
        // Convert to Unix time
        //
        TimeConverter &converter = TimePoint::GetConverter();
        std::pair< bool, TimePoint > unix_time_pair = converter.Convert( rhs,
                ClockID::MakeUnix() );

        if( unix_time_pair.first )
        {
            int64_t seconds_since_epoch = static_cast<int64_t>( 
                    unix_time_pair.second.TimeSinceEpoch().AsSeconds()
                    );

            std::chrono::system_clock::duration d = std::chrono::seconds(seconds_since_epoch); 

            std::chrono::system_clock::time_point ctp(d);
            std::time_t ctp_c = std::chrono::system_clock::to_time_t( ctp );
            os << std::put_time( std::gmtime( &ctp_c ), "%F %T" );
        }
    }

    return os;
}

