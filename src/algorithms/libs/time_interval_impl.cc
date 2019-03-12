/*!
 * \file TimeInterval_impl.cc
 * \brief Private implementation of the TimeInterval class
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

#include "time_interval_impl.h"
#include "gnss_time.h"
#include <cmath>

namespace Constants
{
constexpr int64_t kSecondsPerHour = 3600;
constexpr int64_t kSecondsPerDay = 24 * kSecondsPerHour;
constexpr int64_t kSecondsPerWeek = 7 * kSecondsPerDay;

constexpr int64_t kOneFemtoSecondInternal = 0x01LL;
constexpr int64_t kOnePicoSecondInternal = kOneFemtoSecondInternal * 1000LL;
constexpr int64_t kOneNanoSecondInternal = kOnePicoSecondInternal * 1000LL;
constexpr int64_t kOneMicroSecondInternal = kOneNanoSecondInternal * 1000LL;
constexpr int64_t kOneMilliSecondInternal = kOneMicroSecondInternal * 1000LL;
constexpr int64_t kOneSecondInternal = kOneMilliSecondInternal * 1000LL;
constexpr int64_t kResolution = kOneFemtoSecondInternal;

}  // namespace Constants

TimeIntervalImpl::TimeIntervalImpl(int32_t weeks, int64_t seconds,
    double fractionalSeconds)
    : mSeconds(seconds), mFractionalSeconds(0)
{
    mSeconds += weeks * Constants::kSecondsPerWeek;
    mSeconds += std::floor(fractionalSeconds);
    mFractionalSeconds = static_cast<int64_t>(
        (fractionalSeconds - std::floor(fractionalSeconds)) *
        static_cast<double>(Constants::kOneSecondInternal));

    Normalise();
}

void TimeIntervalImpl::Normalise(void)
{
    int64_t extraSeconds = mFractionalSeconds / Constants::kOneSecondInternal;

    mSeconds += extraSeconds;
    mFractionalSeconds -= extraSeconds * Constants::kOneSecondInternal;
}

double TimeIntervalImpl::AsSeconds(void) const
{
    double seconds = static_cast<double>(mSeconds);

    seconds += static_cast<double>(mFractionalSeconds) / static_cast<double>(Constants::kOneSecondInternal);

    return seconds;
}

int TimeIntervalImpl::AsWeeks(void) const { return mSeconds / Constants::kSecondsPerWeek; }

TimeIntervalImpl& TimeIntervalImpl::operator=(TimeIntervalImpl const& rhs)
{
    mSeconds = rhs.mSeconds;
    mFractionalSeconds = rhs.mFractionalSeconds;
    return *this;
}

TimeIntervalImpl& TimeIntervalImpl::operator+=(TimeIntervalImpl const& rhs)
{
    mSeconds += rhs.mSeconds;
    mFractionalSeconds += rhs.mFractionalSeconds;
    Normalise();
    return *this;
}

TimeIntervalImpl& TimeIntervalImpl::operator-=(TimeIntervalImpl const& rhs)
{
    mSeconds -= rhs.mSeconds;
    mFractionalSeconds -= rhs.mFractionalSeconds;
    Normalise();
    return *this;
}

TimeIntervalImpl& TimeIntervalImpl::operator*=(int64_t n)
{
    mSeconds *= n;
    // To multiply N by the fractional part:
    // b = a*N : both a and N are 64 bit integers
    // b = ( aU * 2^32 + aL )*( NU*2^32 + NL )
    //
    // b =  aU*NU*2^64 + (aU*NL + aL*NU)*2^32 + aL *NL
    //   =   c * 2^64  + ( dU * 2^32  + dL )*2^32 + e
    //   = ( c + dU )*2^64 + dL * 2^32 + e
    //
    //  where c = aU*NU
    //        d = aU*NL + aL*NU;
    //        dU = d >> 32;
    //        dL = d & 0xFFFFFFFF
    //        e = aL * NL;
    //
    // Now 2^64 "fractional" seconds =
    //    2^64/__ONE_SECOND__ seconds
    //  = (2^63/__ONE_SECOND__)*2 seconds
    //
    //  b = (c+dU) * 2 * (2^63/__ONE_SECOND__) seconds
    //    + dL*2^32 + e fractional seconds
    //

    int64_t afU = mFractionalSeconds >> 32;
    int64_t afL = mFractionalSeconds & 0xFFFFFFFF;

    int64_t NU = n >> 32;
    int64_t NL = n & 0xFFFFFFFF;

    int64_t c = afU * NU;
    int64_t d = afL * NU + afU * NL;
    int64_t e = afL * NL;

    int64_t dU = d >> 32;
    int64_t dL = d & 0xFFFFFFFF;

    mSeconds += (c + dU) * 2 * ((1LL << 63) / Constants::kOneSecondInternal);
    mFractionalSeconds = (dL << 32) + e;
    Normalise();
    return *this;
}

TimeIntervalImpl operator+(TimeIntervalImpl lhs, TimeIntervalImpl const& rhs)
{
    lhs += rhs;
    return lhs;
}

TimeIntervalImpl operator-(TimeIntervalImpl lhs, TimeIntervalImpl const& rhs)
{
    lhs -= rhs;
    return lhs;
}

TimeIntervalImpl operator*(TimeIntervalImpl lhs, int64_t n)
{
    lhs *= n;
    return lhs;
}

TimeIntervalImpl operator*(int64_t n, TimeIntervalImpl rhs)
{
    rhs *= n;
    return rhs;
}

bool operator==(TimeIntervalImpl const& lhs, TimeIntervalImpl const& rhs)
{
    return (lhs.mSeconds == rhs.mSeconds &&
            lhs.mFractionalSeconds == rhs.mFractionalSeconds);
}

bool operator!=(TimeIntervalImpl const& lhs, TimeIntervalImpl const& rhs)
{
    return !(lhs == rhs);
}

bool operator<(TimeIntervalImpl const& lhs, TimeIntervalImpl const& rhs)
{
    if (lhs.mSeconds < rhs.mSeconds)
        {
            return true;
        }

    if (lhs.mSeconds > rhs.mSeconds)
        {
            return false;
        }

    // Here weeks and integer seconds are the same:
    return lhs.mFractionalSeconds < rhs.mFractionalSeconds;
}

bool operator>(TimeIntervalImpl const& lhs, TimeIntervalImpl const& rhs)
{
    return rhs < lhs;
}

bool operator<=(TimeIntervalImpl const& lhs, TimeIntervalImpl const& rhs)
{
    return !(lhs > rhs);
}

bool operator>=(TimeIntervalImpl const& lhs, TimeIntervalImpl const& rhs)
{
    return !(lhs < rhs);
}

std::ostream& operator<<(std::ostream& os, TimeIntervalImpl const& rhs)
{
    int weeks = rhs.AsWeeks();
    if (weeks > 0)
        {
            os << weeks << " Week" << (weeks > 1 ? "s " : " ");
        }

    double tow = static_cast<double>(rhs.mSeconds % Constants::kSecondsPerWeek) + static_cast<double>(rhs.mFractionalSeconds) / static_cast<double>(Constants::kOneSecondInternal);
    os << tow << " s";

    return os;
}

TimeInterval::TimeInterval(std::unique_ptr<TimeIntervalImpl> impl)
    : mImpl(std::move(impl)){};

// This copy constructor ensures we do a deep copy of the mImpl member
TimeInterval::TimeInterval(const TimeInterval& rhs)
    : mImpl(
          std::unique_ptr<TimeIntervalImpl>(new TimeIntervalImpl(*(rhs.mImpl))))
{
    // Handled in initialiser
}

#if defined(__cplusplus) && (__cplusplus >= 201103L)
// This copy constructor works with temporary variables, allowing us to
// simply re-use the mImpl created in the temporary variable
TimeInterval::TimeInterval(TimeInterval&& rhs) : mImpl(std::move(rhs.mImpl))
{
    // Handled in initialiser
}
#endif

TimeInterval::~TimeInterval() = default;

double TimeInterval::AsSeconds(void) const { return mImpl->AsSeconds(); }

int TimeInterval::AsWeeks(void) const { return mImpl->AsWeeks(); }

TimeInterval TimeInterval::Weeks(int numWeeks)
{
    return TimeInterval(std::unique_ptr<TimeIntervalImpl>(
        new TimeIntervalImpl(numWeeks, 0, 0.0)));
}

TimeInterval TimeInterval::Days(int numDays)
{
    return TimeInterval(std::unique_ptr<TimeIntervalImpl>(
        new TimeIntervalImpl(0, numDays * Constants::kSecondsPerDay, 0.0)));
}

TimeInterval TimeInterval::Hours(int numHours)
{
    return TimeInterval(std::unique_ptr<TimeIntervalImpl>(
        new TimeIntervalImpl(0, numHours * Constants::kSecondsPerHour, 0.0)));
}

TimeInterval TimeInterval::Seconds(double numSeconds)
{
    return TimeInterval(std::unique_ptr<TimeIntervalImpl>(
        new TimeIntervalImpl(0, 0, numSeconds)));
}

TimeInterval TimeInterval::MilliSeconds(double numMS)
{
    return TimeInterval(std::unique_ptr<TimeIntervalImpl>(
        new TimeIntervalImpl(0, 0, numMS * 1e-3)));
}

TimeInterval TimeInterval::MicroSeconds(double numUS)
{
    return TimeInterval(std::unique_ptr<TimeIntervalImpl>(
        new TimeIntervalImpl(0, 0, numUS * 1e-6)));
}

TimeInterval TimeInterval::NanoSeconds(double numNS)
{
    return TimeInterval(std::unique_ptr<TimeIntervalImpl>(
        new TimeIntervalImpl(0, 0, numNS * 1e-9)));
}

TimeInterval TimeInterval::Ticks(int64_t num_ticks, double tick_rate)
{
    // Assumes tick_rate is actually an integer:
    int64_t integer_ticks_per_second = static_cast<int64_t>(tick_rate);
    TimeInterval T_seconds = TimeInterval::Seconds( num_ticks / integer_ticks_per_second );
    TimeInterval T_fractional_seconds = TimeInterval::Seconds( 
            static_cast<double>(num_ticks % integer_ticks_per_second ) /
            static_cast<double>(integer_ticks_per_second) );

    return T_seconds + T_fractional_seconds;
}


TimeInterval& TimeInterval::operator=(TimeInterval rhs)
{
    std::swap(mImpl, rhs.mImpl);
    return *this;
}

TimeInterval& TimeInterval::operator+=(TimeInterval const& rhs)
{
    *(this->mImpl) += *(rhs.mImpl);
    return *this;
}

TimeInterval& TimeInterval::operator-=(TimeInterval const& rhs)
{
    *(this->mImpl) -= *(rhs.mImpl);
    return *this;
}

TimeInterval& TimeInterval::operator*=(int64_t n)
{
    *(this->mImpl) *= n;
    return *this;
}

TimeInterval operator+(TimeInterval lhs, TimeInterval const& rhs)
{
    lhs += rhs;
    return lhs;
}

TimeInterval operator-(TimeInterval lhs, TimeInterval const& rhs)
{
    lhs -= rhs;
    return lhs;
}

TimeInterval operator*(TimeInterval lhs, int64_t n)
{
    lhs *= n;
    return lhs;
}

TimeInterval operator*(int64_t n, TimeInterval rhs)
{
    rhs *= n;
    return rhs;
}

bool operator==(TimeInterval const& lhs, TimeInterval const& rhs)
{
    return *(lhs.mImpl) == *(rhs.mImpl);
}

bool operator!=(TimeInterval const& lhs, TimeInterval const& rhs)
{
    return !(lhs == rhs);
}

bool operator<(TimeInterval const& lhs, TimeInterval const& rhs)
{
    return *(lhs.mImpl) < *(rhs.mImpl);
}

bool operator>(TimeInterval const& lhs, TimeInterval const& rhs)
{
    return rhs < lhs;
}

bool operator<=(TimeInterval const& lhs, TimeInterval const& rhs)
{
    return !(lhs > rhs);
}

bool operator>=(TimeInterval const& lhs, TimeInterval const& rhs)
{
    return !(lhs < rhs);
}

std::ostream& operator<<(std::ostream& os, TimeInterval const& rhs)
{
    os << *rhs.mImpl;
    return os;
}

