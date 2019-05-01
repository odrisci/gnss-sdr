/*!
 * \file TimeInterval_impl.h
 * \brief Private interface for the TimeInterval class
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

#ifndef GNSS_SDR_TIME_INTERVAL_H_
#define GNSS_SDR_TIME_INTERVAL_H_

#include "gnss_time.h"

class TimeIntervalImpl
{
 private:
  // Prevent unitialised construction
  TimeIntervalImpl(){};

  int64_t mSeconds;

  int64_t mFractionalSeconds;

  ///! Normalise: ensure that all numbers are within their repsective ranges:
  void Normalise(void);

 public:
  //! Initialising constructor
  TimeIntervalImpl(int32_t mWeeks, int64_t mSeconds, double mFractionalSeconds);

  //!
  // Get the time interval in seconds.
  // WARNING: this may cause overflow if the interval is large:
  double AsSeconds(void) const;

  TimeIntervalImpl RemainderMod( TimeIntervalImpl modulus ) const;
  int64_t AsTicks( double sample_rate ) const;
  //!
  // Get the time interval in weeks
  // This returns the whole integer number of weeks in the time interval
  int AsWeeks(void) const;

  TimeIntervalImpl& operator=(TimeIntervalImpl const& otherInterval);
  TimeIntervalImpl& operator+=(TimeIntervalImpl const& dT);
  TimeIntervalImpl& operator-=(TimeIntervalImpl const& dT);
  TimeIntervalImpl& operator*=(int64_t n);
  TimeIntervalImpl& operator/=(int64_t n);

  friend TimeIntervalImpl operator+(TimeIntervalImpl lhs,
                                    TimeIntervalImpl const& rhs);

  friend TimeIntervalImpl operator-(TimeIntervalImpl lhs,
                                    TimeIntervalImpl const& rhs);

  friend TimeIntervalImpl operator*(TimeIntervalImpl lhs,
                                    int64_t n);

  friend TimeIntervalImpl operator*(int64_t n,
                                    TimeIntervalImpl rhs);

  friend TimeIntervalImpl operator/(TimeIntervalImpl lhs,
                                    int64_t n);

  friend bool operator==(TimeIntervalImpl const& lhs,
                         TimeIntervalImpl const& rhs);
  friend bool operator!=(TimeIntervalImpl const& lhs,
                         TimeIntervalImpl const& rhs);

  friend bool operator<(TimeIntervalImpl const& lhs,
                        TimeIntervalImpl const& rhs);
  friend bool operator>(TimeIntervalImpl const& lhs,
                        TimeIntervalImpl const& rhs);
  friend bool operator<=(TimeIntervalImpl const& lhs,
                         TimeIntervalImpl const& rhs);
  friend bool operator>=(TimeIntervalImpl const& lhs,
                         TimeIntervalImpl const& rhs);

  friend std::ostream& operator<<(std::ostream& os,
                                  TimeIntervalImpl const& rhs);

};  // TimeIntervalImpl

#endif
