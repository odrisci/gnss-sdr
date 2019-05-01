/*!
 * \file time_test.cc
 * \brief Unit tests for the gnss_time classes
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
#include "gnss_time_converter.h"
#include <gtest/gtest.h>

TEST(TimeIntervalTest, CanCreateDifferentUnits)
{
    TimeInterval dt = TimeInterval::Weeks(1);

    ASSERT_EQ(dt, TimeInterval::Seconds(604800));

    dt = TimeInterval::Days(1);
    ASSERT_EQ(dt, TimeInterval::Seconds(24 * 3600));

    dt = TimeInterval::Hours(1);
    ASSERT_EQ(dt, TimeInterval::Seconds(3600));

    dt = TimeInterval::MilliSeconds(1000);
    ASSERT_EQ(dt, TimeInterval::Seconds(1));

    dt = TimeInterval::MicroSeconds(1000000);
    ASSERT_EQ(dt, TimeInterval::Seconds(1));

    dt = TimeInterval::NanoSeconds(1000000000);
    ASSERT_EQ(dt, TimeInterval::Seconds(1));
}

TEST(TimeIntervalTest, CanHandleLargeDifferences)
{
    TimeInterval dt1 = TimeInterval::Weeks(2048);
    TimeInterval dt2 = TimeInterval::NanoSeconds(1);

    TimeInterval dt3 = dt1 + dt2;

    ASSERT_EQ(dt3 - dt1, TimeInterval::NanoSeconds(1));
}

TEST(TimeIntervalTest, CanPropagateByMultiplication)
{
    TimeInterval dt1 = TimeInterval::MilliSeconds(1);
    int64_t N = 1001;

    TimeInterval dt2 = dt1 * N;

    ASSERT_EQ(dt2, TimeInterval::Seconds(1) + TimeInterval::MilliSeconds(1));

    dt2 *= N;
    ASSERT_EQ(dt2, TimeInterval::Seconds(1001) + TimeInterval::MilliSeconds(1001));
}

TEST(TimeTest, CanConstructTimePoint)
{
    TimePoint t = TimePoint::MakeGnss(GnssSystem::kGps,
        TimeInterval::Weeks(2048) + TimeInterval::Seconds(604500));

    ASSERT_EQ(t.TOW().AsSeconds(), 604500);
}

TEST(TimeTest, CorrectlyComputesRemainders)
{
    int n_weeks = 2048;
    int n_secs = 4500;
    int n_nano_secs = 100;
    int n_milli_secs = 23;
    TimeInterval t = 
        TimeInterval::Weeks(n_weeks) 
        + TimeInterval::Seconds(n_secs)
        + TimeInterval::MilliSeconds(n_milli_secs)
        + TimeInterval::NanoSeconds(n_nano_secs);

    TimeInterval remainder = t.RemainderMod( TimeInterval::Weeks(1) );

    ASSERT_EQ( remainder, TimeInterval::Seconds(n_secs ) +
            TimeInterval::MilliSeconds(n_milli_secs ) +
            TimeInterval::NanoSeconds( n_nano_secs ) );

    remainder = t.RemainderMod( TimeInterval::Seconds(1) );

    ASSERT_EQ( remainder, TimeInterval::MilliSeconds(n_milli_secs ) +
            TimeInterval::NanoSeconds( n_nano_secs ) );

    remainder = t.RemainderMod( TimeInterval::MilliSeconds(1) );

    ASSERT_EQ( remainder, TimeInterval::NanoSeconds( n_nano_secs ) );

    remainder = t.RemainderMod( TimeInterval::NanoSeconds(1) );

    ASSERT_EQ( remainder, TimeInterval::Seconds(0) );

}

TEST(TimeTest, ComputesClockTicks)
{
    int64_t fs_exact = 40000000; // 40 Msps
    double fs = static_cast<double>( fs_exact );

    int64_t sample_counter = fs_exact * 3600 * 24 * 7 * 51 ;

    TimeInterval dtTicks = TimeInterval::Ticks( sample_counter, fs );

    ASSERT_EQ( dtTicks.AsWeeks(), 51 );

    int64_t tick_count = dtTicks.AsTicks( fs );

    ASSERT_EQ( tick_count, sample_counter );

    dtTicks += TimeInterval::Ticks( 1, fs );

    tick_count = dtTicks.AsTicks( fs );

    ASSERT_EQ( tick_count, sample_counter + 1 );
}

TEST(TimeConverterTest, CanConvertGalileoToGps)
{
    TimePoint tGps = TimePoint::MakeGnss(GnssSystem::kGps,
        TimeInterval::Weeks(2048) + TimeInterval::Seconds(604500));
    TimePoint tGal = TimePoint::MakeGnss(GnssSystem::kGalileo,
        tGps.TimeSinceEpoch() - TimeInterval::Weeks(1024));

    TimeConverter &converter = TimePoint::GetConverter();
    std::pair<bool, TimePoint> conv_pair = converter.Convert(tGal, ClockID::MakeGnss(GnssSystem::kGps));

    ASSERT_EQ(conv_pair.first, true);

    ASSERT_EQ(tGps, conv_pair.second);
}

TEST(TimeConverterTest, CanConvertToNonGnssTimes)
{
    TimePoint tGps = TimePoint::MakeGnss(GnssSystem::kGps, TimeInterval::Seconds(0));
    TimePoint gps_epoch_utc = TimePoint::MakeUTC( 1980, TimePoint::eMonth::January, 6, 0, 0 );
    TimePoint tGal = TimePoint::MakeGnss( GnssSystem::kGalileo, TimeInterval::Seconds(0) );
    TimePoint gal_epoch_utc = TimePoint::MakeUTC( 1999, TimePoint::eMonth::August, 22, 0, 0 )
        -TimeInterval::Seconds(13);
    TimePoint tBeiDou = TimePoint::MakeGnss( GnssSystem::kBeiDou, TimeInterval::Seconds(0) );
    TimePoint beidou_epoch_utc = TimePoint::MakeUTC( 2006, TimePoint::eMonth::January, 1, 0, 0 );

    TimeConverter &converter = TimePoint::GetConverter();
    std::pair<bool, TimePoint> conv_pair = converter.Convert(tGps, ClockID::MakeUTC());

    ASSERT_EQ(conv_pair.first, true);
    ASSERT_EQ(gps_epoch_utc, conv_pair.second);

    conv_pair = converter.Convert(tGal, gal_epoch_utc.GetClockID() );
    ASSERT_EQ(conv_pair.first, true);
    ASSERT_EQ(gal_epoch_utc, conv_pair.second);

    conv_pair = converter.Convert(tBeiDou, beidou_epoch_utc.GetClockID() );
    ASSERT_EQ(conv_pair.first, true);
    ASSERT_EQ(beidou_epoch_utc, conv_pair.second);
}

TEST(TimeConverterTest, CanHandleReceiverTimes)
{
    TimePoint tNow = TimePoint::GetCurrentUTC();

    int64_t fs_exact = 40000000; // 40 Msps
    double fs = static_cast<double>( fs_exact );

    int64_t sample_counter = fs_exact * 3600 * 24 * 7 * 51 ;

    TimePoint tRx = TimePoint::MakeReceiver( TimeInterval::Ticks( sample_counter, fs ) );

    tRx += TimeInterval::Ticks( 101, fs );

    ASSERT_EQ( tRx.Week(), 51 );

    ASSERT_NEAR( tRx.TOW().AsSeconds(), 101 / fs, 1e-9 );

    TimeConverter &converter = TimePoint::GetConverter();
    std::pair<bool, TimePoint> conv_pair = converter.Convert(tRx, ClockID::MakeGnss( GnssSystem::kGps ));

    // Can't convert yet since we don't know the receiver epoch
    ASSERT_EQ(conv_pair.first, false);

    converter.SetReceiverEpoch( tRx.GetClockID().GetId(), tNow );
    conv_pair = converter.Convert(tRx, ClockID::MakeGnss( GnssSystem::kGps ));

    ASSERT_EQ(conv_pair.first, true);

}
