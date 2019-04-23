/*!
 * \file gnss_time.h
 * \brief Time Utilities in GNSS-SDR
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

#ifndef GNSS_SDR_TIME_H_
#define GNSS_SDR_TIME_H_

#include <memory>
#include <limits>
#include <ostream>

/// Forward declaration:
class TimeInterval;
class TimePointImpl;
class TimeConverter;

enum class GnssSystem : char
{
    kBeiDou = 'C',
    kGalileo = 'E',
    kGps = 'G',
    kIrnss = 'I',
    kQzss = 'J',
    kGlonass = 'R',
    kSbas = 'S'
};

/*!
 *  Class encapsulating a Clock identifier
 *
 *  Each clock identifier has a system in which the clock measures time (such as UTC,
 *  GPS, receiver time, etc) and an optional identifier for the specific clock considered.
 *
 *  The system clock (UTC, GPS, etc) is identified by a special identifier
 */

class ClockID
{
public:
    enum class eClockSystem : uint32_t
    {
        kReceiver = 0,
        kGnssStart = 1024,
        kGps,
        kGalileo,
        kGlonass,
        kBeiDou,
        kIrnss,
        kQzss,
        kSbas,
        kGnssEnd,
        kUniStart = 2048,
        kUTC,
        kUnix,
        kNTP,
        kTAI,
        kUniEnd = 4096
    };


    static constexpr uint32_t kSystemClockID = std::numeric_limits<uint32_t>::max();
    ClockID() = delete;

    static ClockID MakeGnss( GnssSystem sys, uint32_t id = kSystemClockID );
    static ClockID MakeUTC( uint32_t id = kSystemClockID );
    static ClockID MakeUnix( uint32_t id = kSystemClockID );
    static ClockID MakeNTP( uint32_t id = kSystemClockID );
    static ClockID MakeTAI( uint32_t id = kSystemClockID );
    static ClockID MakeReceiver( uint32_t id = 0 );

    bool IsCompatibleWith( ClockID rhs ) const;
    bool IsGnss(void) const;
    bool IsSystemClock(void) const;
    eClockSystem GetSystem(void) const;
    uint32_t GetId( void ) const;

    bool KeepsLeapSeconds(void) const;

    friend bool operator==(ClockID const &lhs, ClockID const &rhs);
    friend bool operator!=(ClockID const &lhs, ClockID const &rhs);
    friend std::ostream &operator<<(std::ostream &os, ClockID const &rhs);


private:
    eClockSystem d_sys;
    uint32_t d_id;

    ClockID( eClockSystem sys, uint32_t id ) : d_sys(sys), d_id(id) {};
};

/*!
 * Simple class to capture the range of time scales needed in GNSS
 *
 * Includes a system identifier, a week number and time stored as the number
 * of seconds into the week as an integer plus a floating point representation
 * of the time sub one second.
 *
 * Sample usage:
 *
 *  int weekNumber = 1884;
 *  int tow = 345600;
 *  double fracTow = 0.0786;
 *
 *  TimePoint currTime TimePoint::MakeGnss( Systems::GPS,
 *      TimeInterval::Weeks( weekNumber ) +
 *      TimeInterval::Seconds( tow ) +
 *      TimeInterval::Seconds( fracTow ) );
 */
class TimePoint
{
private:
    // Disable visual studio warning C4251 about
    // making the definition of TimePointImpl public
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4251)
#endif
    /// Private implementation class
    std::unique_ptr<TimePointImpl> mImpl;
#ifdef _MSC_VER
#pragma warning(pop)
#endif

public:
    TimePoint();
    TimePoint(ClockID const &clockID);
    TimePoint(ClockID const &clockID, TimeInterval const &timeInterval);

    /// Copy Constructors
    /// Ensure Deep copy of implementation
    TimePoint(const TimePoint &rhs);
#if defined(__cplusplus) && (__cplusplus >= 201103L)
    TimePoint(TimePoint &&rhs);
#endif
    ~TimePoint();

    //! Get the system:
    ClockID GetClockID(void) const;

    //! Get the week number
    int Week(void) const;

    //! Get the time of Week as a time interval
    TimeInterval TOW(void) const;

    //! Get the time since the clock epoch as a TimeInterval
    TimeInterval TimeSinceEpoch(void) const;

    //! Make a GNSS TimePoint
    static TimePoint MakeGnss(GnssSystem sys, int week_number, double TOW);
    static TimePoint MakeGnss(GnssSystem sys, TimeInterval time_interval);

    //! Make a receiver time point
    static TimePoint MakeReceiver(TimeInterval time_interval, uint32_t r_id = 0);
    static TimePoint MakeReceiver(int64_t sample_count, double sample_rate, uint32_t r_id = 0);

    //! Get unix time
    static TimePoint GetCurrentUnix(void);
    //! UTC time utilities
    enum class eMonth : int
    {
        January = 0, February, March, April, May, June, July, August, September,
        October, November, December
    };

    static TimePoint GetCurrentUTC(void);
    static TimePoint MakeUTC( int year, eMonth month, int day, int hour = 0, 
            int minute = 0, int seconds = 0 );

    static TimeConverter &GetConverter(void);

    TimePoint &operator=(TimePoint const &otherTime);
    TimePoint &operator+=(TimeInterval const &dT);
    TimePoint &operator-=(TimeInterval const &dT);

    friend TimeInterval operator-(TimePoint lhs, TimePoint const &rhs);
    friend TimePoint operator-(TimePoint lhs, TimeInterval const &rhs);
    friend TimePoint operator+(TimePoint lhs, TimeInterval const &rhs);

    friend bool operator==(TimePoint const &lhs, TimePoint const &rhs);
    friend bool operator!=(TimePoint const &lhs, TimePoint const &rhs);

    friend bool operator<(TimePoint const &lhs, TimePoint const &rhs);
    friend bool operator>(TimePoint const &lhs, TimePoint const &rhs);
    friend bool operator<=(TimePoint const &lhs, TimePoint const &rhs);
    friend bool operator>=(TimePoint const &lhs, TimePoint const &rhs);

    friend std::ostream &operator<<(std::ostream &os, TimePoint const &rhs);
};

/// Forward declaration of the TimeIntervalImpl
class TimeIntervalImpl;

/*!
 * Simple class to capture the range of time scales needed in GNSS
 */
class TimeInterval
{
private:
    // Prevent unitialised construction
    TimeInterval() = delete;

    // Disable visual studio warning C4251 about
    // making the definition of TimePointImpl public
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4251)
#endif
    /// Private implementation class
    // Store a pointer to the implementation:
    std::unique_ptr<TimeIntervalImpl> mImpl;
#ifdef _MSC_VER
#pragma warning(pop)
#endif

    // Allow the static members to construct objects of this class
    // when then create an implementation object:
    TimeInterval(std::unique_ptr<TimeIntervalImpl> impl);

public:
    /// Copy Constructors
    /// Ensure Deep copy of implementation
    TimeInterval(const TimeInterval &rhs);
#if defined(__cplusplus) && (__cplusplus >= 201103L)
    TimeInterval(TimeInterval &&rhs);
#endif
    ~TimeInterval();

    /// Accessors:
    // @{

    //!
    // Get the time interval in seconds.
    // WARNING: this may cause overflow if the interval is large:
    double AsSeconds(void) const;

    int64_t IntegerSeconds(void) const;
    TimeInterval RemainderMod(TimeInterval modulus) const;

    //!
    // Get the time interval in weeks
    // This returns the whole integer number of weeks in the time interval
    int AsWeeks(void) const;

    // @}

    /// Static member functions to create time intervals:
    // @{
    static TimeInterval Years(int numYears);
    static TimeInterval Weeks(int numWeeks);

    static TimeInterval Days(int numDays);
    static TimeInterval Hours(int numHours);

    static TimeInterval Seconds(double numSeconds);

    static TimeInterval MilliSeconds(double numMs);
    static TimeInterval MicroSeconds(double numUs);
    static TimeInterval NanoSeconds(double numNs);

    static TimeInterval Ticks(int64_t num_ticks, double tick_rate);

    // @}

    TimeInterval &operator=(TimeInterval otherInterval);
    TimeInterval &operator+=(TimeInterval const &dT);
    TimeInterval &operator-=(TimeInterval const &dT);
    TimeInterval &operator*=(int64_t n);
    TimeInterval& operator/=(int64_t n);

    friend TimeInterval operator+(TimeInterval lhs, TimeInterval const &rhs);
    friend TimeInterval operator-(TimeInterval lhs, TimeInterval const &rhs);

    friend TimeInterval operator*(TimeInterval lhs, int64_t n);
    friend TimeInterval operator*(int64_t n, TimeInterval rhs);

    friend TimeInterval operator/(TimeInterval lhs, int64_t n);

    friend bool operator==(TimeInterval const &lhs, TimeInterval const &rhs);
    friend bool operator!=(TimeInterval const &lhs, TimeInterval const &rhs);

    friend bool operator<(TimeInterval const &lhs, TimeInterval const &rhs);
    friend bool operator>(TimeInterval const &lhs, TimeInterval const &rhs);
    friend bool operator<=(TimeInterval const &lhs, TimeInterval const &rhs);
    friend bool operator>=(TimeInterval const &lhs, TimeInterval const &rhs);

    friend std::ostream &operator<<(std::ostream &os, TimeInterval const &rhs);
};

#endif

