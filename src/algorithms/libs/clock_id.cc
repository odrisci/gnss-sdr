/*!
 * \file clock_id.cc
 * \brief ClockID utility class for high resoultion time points
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

#include "gnss_time.h"

ClockID ClockID::MakeGnss(GnssSystem sys, uint32_t id)
{
    eClockSystem clk_sys = eClockSystem::kReceiver;
    switch( sys )
    {
        case GnssSystem::kGalileo:
            clk_sys = eClockSystem::kGalileo;
            break;
        case GnssSystem::kGps:
            clk_sys = eClockSystem::kGps;
            break;
        case GnssSystem::kBeiDou:
            clk_sys = eClockSystem::kBeiDou;
            break;
        case GnssSystem::kGlonass:
            clk_sys = eClockSystem::kGlonass;
            break;
        case GnssSystem::kIrnss:
            clk_sys = eClockSystem::kIrnss;
            break;
        case GnssSystem::kQzss:
            clk_sys = eClockSystem::kQzss;
            break;
        case GnssSystem::kSbas:
            clk_sys = eClockSystem::kSbas;
            break;
    }
    ClockID ret( clk_sys, id );
    return ret;
}

ClockID ClockID::MakeUTC(uint32_t id)
{
    ClockID ret( eClockSystem::kUTC, id );
    return ret;
}

ClockID ClockID::MakeUnix(uint32_t id)
{
    ClockID ret( eClockSystem::kUnix, id );
    return ret;
}

ClockID ClockID::MakeNTP(uint32_t id)
{
    ClockID ret( eClockSystem::kNTP, id );
    return ret;
}

ClockID ClockID::MakeTAI(uint32_t id)
{
    ClockID ret( eClockSystem::kTAI, id );
    return ret;
}

ClockID ClockID::MakeReceiver( uint32_t id)
{
    ClockID ret( eClockSystem::kReceiver, id );
    return ret;
}

bool ClockID::IsCompatibleWith( ClockID rhs ) const
{
    return d_sys == rhs.d_sys;
}

bool ClockID::IsGnss(void) const
{
    return d_sys > eClockSystem::kGnssStart && d_sys < eClockSystem::kGnssEnd;
}

bool ClockID::IsSystemClock(void) const
{
    return d_id == kSystemClockID;
}

ClockID::eClockSystem ClockID::GetSystem(void) const
{
    return d_sys;
}

uint32_t ClockID::GetId( void ) const
{
    return d_id;
}

bool ClockID::KeepsLeapSeconds(void) const
{
    bool ret = true;
    switch( d_sys )
    {
        case eClockSystem::kReceiver:
        case eClockSystem::kGps:
        case eClockSystem::kGalileo:
        case eClockSystem::kBeiDou:
        case eClockSystem::kQzss:
        case eClockSystem::kIrnss:
        case eClockSystem::kSbas:
        case eClockSystem::kTAI:
            ret = false;
            break;
        case eClockSystem::kGlonass:
        case eClockSystem::kUTC:
        case eClockSystem::kUnix:
        case eClockSystem::kNTP:
            ret = true;
    }

    return ret;
}

bool operator==(ClockID const &lhs, ClockID const &rhs)
{
    return (lhs.d_sys == rhs.d_sys) && (lhs.d_id == rhs.d_id );
}

bool operator!=(ClockID const &lhs, ClockID const &rhs)
{
    return ! (lhs == rhs);
}

std::ostream &operator<<(std::ostream &os, ClockID const &rhs)
{
    switch( rhs.d_sys )
    {
        case ClockID::eClockSystem::kReceiver:
            os << "Rx.";
            break;
        case ClockID::eClockSystem::kGps:
            os << "GPS";
            break;
        case ClockID::eClockSystem::kGalileo:
            os << "Galileo";
            break;
        case ClockID::eClockSystem::kGlonass:
            os << "Glonass";
            break;
        case ClockID::eClockSystem::kBeiDou:
            os << "BeiDou";
            break;
        case ClockID::eClockSystem::kIrnss:
            os << "IRNSS";
            break;
        case ClockID::eClockSystem::kQzss:
            os << "QZSS";
            break;
        case ClockID::eClockSystem::kSbas:
            os << "SBAS";
            break;
        case ClockID::eClockSystem::kUTC:
            os << "UTC";
            break;
        case ClockID::eClockSystem::kUnix:
            os << "Unix";
            break;
        case ClockID::eClockSystem::kNTP:
            os << "NTP";
            break;
        default:
            os << "UNKNOWN";
    }

    if( not rhs.IsSystemClock() )
    {
        os << " " << rhs.d_id;
    }

    return os;
}
