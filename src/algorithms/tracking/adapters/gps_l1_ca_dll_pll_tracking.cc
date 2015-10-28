/*!
 * \file gps_l1_ca_dll_pll_tracking.cc
 * \brief Implementation of an adapter of a DLL+PLL tracking loop block
 * for GPS L1 C/A to a TrackingInterface
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
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


#include "gps_l1_ca_dll_pll_tracking.h"
#include <glog/logging.h>
#include "GPS_L1_CA.h"
#include "configuration_interface.h"


using google::LogMessage;

GpsL1CaDllPllTracking::GpsL1CaDllPllTracking(
        ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams,
        boost::shared_ptr<gr::msg_queue> queue) :
                role_(role), in_streams_(in_streams), out_streams_(out_streams),
                queue_(queue)
{
    DLOG(INFO) << "role " << role;
    //################# CONFIGURATION PARAMETERS ########################
    int fs_in;
    int vector_length;
    int f_if;
    bool dump;
    std::string dump_filename;
    std::string item_type;
    std::string default_item_type = "gr_complex";
    float pll_initial_bw_hz;
    float pll_final_bw_hz;
    float dll_initial_bw_hz;
    float dll_final_bw_hz;
    float initial_early_late_space_chips;
    float final_early_late_space_chips;
    int pll_loop_order;
    int dll_loop_order;
    bool aid_code_with_carrier;
    item_type = configuration->property(role + ".item_type", default_item_type);
    //vector_length = configuration->property(role + ".vector_length", 2048);
    fs_in = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    f_if = configuration->property(role + ".if", 0);
    dump = configuration->property(role + ".dump", false);
    pll_initial_bw_hz = configuration->property(role + ".pll_initial_bw_hz", 50.0);
    pll_final_bw_hz = configuration->property(role + ".pll_final_bw_hz", 15.0);
    dll_initial_bw_hz = configuration->property(role + ".dll_initial_bw_hz", 2.0);
    dll_final_bw_hz = configuration->property(role + ".dll_final_bw_hz", 2.0);
    initial_early_late_space_chips = configuration->property(role + ".initial_early_late_space_chips", 0.5);
    final_early_late_space_chips = configuration->property(role + ".final_early_late_space_chips", 0.5);
    pll_loop_order = configuration->property(role + ".pll_loop_order", 3);
    dll_loop_order = configuration->property(role + ".dll_loop_order", 1);
    aid_code_with_carrier = configuration->property(role + ".aid_code_with_carrier", true );

    std::string default_dump_filename = "./track_ch";
    dump_filename = configuration->property(role + ".dump_filename",
            default_dump_filename); //unused!
    vector_length = std::round(fs_in / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS));

    //################# MAKE TRACKING GNURadio object ###################
    if (item_type.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            tracking_ = gps_l1_ca_dll_pll_make_tracking_cc(
                    f_if,
                    fs_in,
                    vector_length,
                    queue_,
                    dump,
                    dump_filename,
                    pll_loop_order,
                    pll_initial_bw_hz,
                    pll_final_bw_hz,
                    dll_loop_order,
                    dll_initial_bw_hz,
                    dll_final_bw_hz,
                    initial_early_late_space_chips,
                    final_early_late_space_chips,
                    aid_code_with_carrier);
        }
    else
        {
            item_size_ = sizeof(gr_complex);
            LOG(WARNING) << item_type << " unknown tracking item type.";
        }
    channel_ = 0;
    channel_internal_queue_ = 0;
    DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
}


GpsL1CaDllPllTracking::~GpsL1CaDllPllTracking()
{}


void GpsL1CaDllPllTracking::start_tracking()
{
    tracking_->start_tracking();
}

/*
 * Set tracking channel unique ID
 */
void GpsL1CaDllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}

/*
 * Set tracking channel internal queue
 */
void GpsL1CaDllPllTracking::set_channel_queue(
        concurrent_queue<int> *channel_internal_queue)
{
    channel_internal_queue_ = channel_internal_queue;
    tracking_->set_channel_queue(channel_internal_queue_);
}

void GpsL1CaDllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_->set_gnss_synchro(p_gnss_synchro);
}

void GpsL1CaDllPllTracking::connect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	//nothing to connect, now the tracking uses gr_sync_decimator
}

void GpsL1CaDllPllTracking::disconnect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	//nothing to disconnect, now the tracking uses gr_sync_decimator
}

gr::basic_block_sptr GpsL1CaDllPllTracking::get_left_block()
{
    return tracking_;
}

gr::basic_block_sptr GpsL1CaDllPllTracking::get_right_block()
{
    return tracking_;
}

