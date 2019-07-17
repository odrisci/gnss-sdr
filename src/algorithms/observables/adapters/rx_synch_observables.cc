/*!
 * \file rx_synch_observables.cc
 * \brief Implementation of an adapter of a Galileo E1 observables block
 * to a ObservablesInterface
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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


#include "rx_synch_observables.h"
#include "configuration_interface.h"
#include <glog/logging.h>

using google::LogMessage;

RxSynchObservables::RxSynchObservables(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams) :
                            role_(role),
                            in_streams_(in_streams),
                            out_streams_(out_streams)
{
    std::string default_dump_filename = "./observables.dat";
    DLOG(INFO) << "role " << role;
    dump_ = configuration->property(role + ".dump", false);
    dump_mat_ = configuration->property(role + ".dump_mat", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);
    double default_rate = 1.0;
    int observable_interval_ms = configuration->property("GNSS-SDR.observable_interval_ms", 20);
    double rate_hz  = 1000.0 / static_cast<double>(observable_interval_ms);
    observables_ = rx_synch_make_observables(in_streams_, out_streams_, dump_, dump_mat_, dump_filename_, rate_hz);
    DLOG(INFO) << "pseudorange(" << observables_->unique_id() << ")";
}




RxSynchObservables::~RxSynchObservables()
{}




void RxSynchObservables::connect(gr::top_block_sptr top_block)
{
    if(top_block) { /* top_block is not null */};
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}



void RxSynchObservables::disconnect(gr::top_block_sptr top_block)
{
    if(top_block) { /* top_block is not null */};
    // Nothing to disconnect
}




gr::basic_block_sptr RxSynchObservables::get_left_block()
{
    return observables_;
}



gr::basic_block_sptr RxSynchObservables::get_right_block()
{
    return observables_;
}
