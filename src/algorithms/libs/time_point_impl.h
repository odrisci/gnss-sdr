/*!
 * \file TimePoint_impl.h
 * \brief Private interface for the TimePoint class
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

#ifndef GNSS_SDR_TIME_POINT_H_
#define GNSS_SDR_TIME_POINT_H_

#include "gnss_time.h"

class TimePointImpl
{
 private:
  TimeInterval mTimeSinceEpoch;

  ClockID mClockID;

 public:
  TimePointImpl(ClockID const &clockID);

  TimePointImpl(ClockID const &clockID, TimeInterval const &timeInterval);

  //! Get the Clock ID:
  ClockID const &GetClockID(void) const;

  int Week(void) const;

  TimeInterval TOW(void) const;

  //! Get the time since the clock epoch as a TimeInterval
  TimeInterval TimeSinceEpoch(void ) const;

  TimePointImpl &operator=(TimePointImpl const &rhs);
  TimePointImpl &operator+=(TimeInterval const &rhs);
  TimePointImpl &operator-=(TimeInterval const &rhs);

  friend TimeInterval operator-(TimePointImpl const &lhs,
                                TimePointImpl const &rhs);
  friend TimePointImpl operator-(TimePointImpl lhs, TimeInterval const &rhs);
  friend TimePointImpl operator+(TimePointImpl lhs, TimeInterval const &rhs);

  friend bool operator==(TimePointImpl const &lhs, TimePointImpl const &rhs);
  friend bool operator!=(TimePointImpl const &lhs, TimePointImpl const &rhs);

  friend bool operator<(TimePointImpl const &lhs, TimePointImpl const &rhs);
  friend bool operator>(TimePointImpl const &lhs, TimePointImpl const &rhs);
  friend bool operator<=(TimePointImpl const &lhs, TimePointImpl const &rhs);
  friend bool operator>=(TimePointImpl const &lhs, TimePointImpl const &rhs);

  friend std::ostream &operator<<(std::ostream &os, TimePointImpl const &rhs);

};  // TimePointImpl

#endif
