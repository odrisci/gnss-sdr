/*!
 * \file rx_synch_observables_gs.h
 * \brief Interface of the observables computation block for Galileo E1
 * \author Cillian O'Driscoll 2019. cillian.odriscoll(at)gmail.com
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_rx_synch_observables_gs_H
#define GNSS_SDR_rx_synch_observables_gs_H

#include "gnss_time.h"
#include <gnuradio/block.h>
#include <fstream>
#include <mutex>
#include <string>


class Gnss_Synchro;
class rx_synch_observables_gs;

typedef boost::shared_ptr<rx_synch_observables_gs> rx_synch_observables_gs_sptr;

rx_synch_observables_gs_sptr
rx_synch_make_observables(uint32_t nchannels_in, uint32_t nchannels_out, bool dump, bool dump_mat, std::string dump_filename, double rate_hz);

/*!
 * \brief This class implements a block that computes Galileo observables
 */
class rx_synch_observables_gs : public gr::block
{
public:
    ~rx_synch_observables_gs();
    void forecast(int noutput_items, gr_vector_int &ninput_items_required);

    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend rx_synch_observables_gs_sptr
    rx_synch_make_observables(uint32_t nchannels_in, uint32_t nchannels_out, bool dump, bool dump_mat, std::string dump_filename, double rate_hz);
    rx_synch_observables_gs(uint32_t nchannels_in, uint32_t nchannels_out, bool dump, bool dump_mat, std::string dump_filename, double rate_hz);
    void msg_handler_pvt_to_observables(pmt::pmt_t msg);
    int32_t save_matfile();
    void empty_current_measurements();

    int d_max_noutputs;
    bool d_dump;
    bool d_dump_mat;
    int d_rx_id;
    uint32_t d_nchannels_in;
    uint32_t d_nchannels_out;
    double d_rate_hz;
    std::vector< int > d_ninput_items_required;
    std::vector< Gnss_Synchro > d_current_measurements;
    TimeInterval d_rx_epoch_offset;
    TimePoint    d_receiver_time;

    std::string d_dump_filename;
    std::ofstream d_dump_file;
    std::mutex d_mutex;
};

#endif
