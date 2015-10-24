/*!
 * \file tracking_loop_filter_test.cc
 * \brief  This file implements tests for the general loop filter
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll(at)gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "tracking_loop_filter.h"
#include "tracking_2nd_PLL_filter.h"

#include <gtest/gtest.h>

TEST(TrackingLoopFilterTest, FirstOrderLoop)
{
    int loop_order = 1;
    float noise_bandwidth = 5.0;
    float update_interval = 0.001;
    bool include_last_integrator = false;

    Tracking_loop_filter theFilter( update_interval,
            noise_bandwidth,
            loop_order,
            include_last_integrator );

    EXPECT_EQ( theFilter.get_noise_bandwidth(), noise_bandwidth );
    EXPECT_EQ( theFilter.get_update_interval(), update_interval );
    EXPECT_EQ( theFilter.get_include_last_integrator(), include_last_integrator );
    EXPECT_EQ( theFilter.get_order(), loop_order );

    std::vector< float > sample_data = { 0, 0, 1.0, 0.0, 0.0, 0.0 };

    theFilter.initialize( 0.0 );

    float g1 = noise_bandwidth*4.0;

    float result = 0.0;
    for( unsigned int i = 0; i < sample_data.size(); ++i )
    {
        result = theFilter.apply( sample_data[i] );

        ASSERT_FLOAT_EQ( result, sample_data[i]*g1 );
    }
}

TEST(TrackingLoopFilterTest, FirstOrderLoopWithLastIntegrator)
{
    int loop_order = 1;
    float noise_bandwidth = 5.0;
    float update_interval = 0.001;
    bool include_last_integrator = true;

    Tracking_loop_filter theFilter( update_interval,
            noise_bandwidth,
            loop_order,
            include_last_integrator );

    EXPECT_EQ( theFilter.get_noise_bandwidth(), noise_bandwidth );
    EXPECT_EQ( theFilter.get_update_interval(), update_interval );
    EXPECT_EQ( theFilter.get_include_last_integrator(), include_last_integrator );
    EXPECT_EQ( theFilter.get_order(), loop_order );

    std::vector< float > sample_data = { 0, 0, 1.0, 0.0, 0.0, 0.0 };

    theFilter.initialize( 0.0 );

    float g1 = noise_bandwidth*4.0;

    float result = 0.0;
    for( unsigned int i = 0; i < sample_data.size(); ++i )
    {
        result = theFilter.apply( sample_data[i] );
        std::cout << " " << result;
    }
    std::cout << std::endl;
}



TEST(TrackingLoopFilterTest, SecondOrderLoop)
{
    int loop_order = 2;
    float noise_bandwidth = 5.0;
    float update_interval = 0.001;
    bool include_last_integrator = false;

    Tracking_loop_filter theFilter( update_interval,
            noise_bandwidth,
            loop_order,
            include_last_integrator );

    Tracking_2nd_PLL_filter pllFilter( update_interval );
    pllFilter.set_PLL_BW( noise_bandwidth );

    EXPECT_EQ( theFilter.get_noise_bandwidth(), noise_bandwidth );
    EXPECT_EQ( theFilter.get_update_interval(), update_interval );
    EXPECT_EQ( theFilter.get_include_last_integrator(), include_last_integrator );
    EXPECT_EQ( theFilter.get_order(), loop_order );

    std::vector< float > sample_data = { 0, 0, 1.0, 0.0, 0.0, 0.0 };

    theFilter.initialize( 0.0 );
    pllFilter.initialize( 0.0 );

    float result = 0.0;
    float pll_result = 0.0;
    for( unsigned int i = 0; i < sample_data.size(); ++i )
    {
        result = theFilter.apply( sample_data[i] );
        pll_result = pllFilter.get_carrier_nco( sample_data[i] );

        ASSERT_FLOAT_EQ( result, pll_result );
    }
}

TEST(TrackingLoopFilterTest, SecondOrderLoopWithLastIntegrator)
{
    int loop_order = 2;
    float noise_bandwidth = 5.0;
    float update_interval = 0.001;
    bool include_last_integrator = true;

    Tracking_loop_filter theFilter( update_interval,
            noise_bandwidth,
            loop_order,
            include_last_integrator );

    EXPECT_EQ( theFilter.get_noise_bandwidth(), noise_bandwidth );
    EXPECT_EQ( theFilter.get_update_interval(), update_interval );
    EXPECT_EQ( theFilter.get_include_last_integrator(), include_last_integrator );
    EXPECT_EQ( theFilter.get_order(), loop_order );

    std::vector< float > sample_data = { 0, 0, 1.0, 0.0, 0.0, 0.0 };

    theFilter.initialize( 0.0 );

    float g1 = noise_bandwidth*4.0;

    float result = 0.0;
    for( unsigned int i = 0; i < sample_data.size(); ++i )
    {
        result = theFilter.apply( sample_data[i] );
        std::cout << " " << result;
    }
    std::cout << std::endl;
}


