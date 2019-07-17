/*!
 * \file rx_synch_observables_gs.cc
 * \brief Implementation of receiver synchronous observable generation
 * \author Cillian O'Driscoll, cillian.odriscoll(@)gmail.com
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

#include "rx_synch_observables_gs.h"
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"
#include "display.h"
#include "gnss_synchro.h"
#include "gnss_time.h"
#include "gnss_time_converter.h"
#include <armadillo>
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <matio.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <utility>
#include <vector>

using google::LogMessage;


rx_synch_observables_gs_sptr rx_synch_make_observables(uint32_t nchannels_in, uint32_t nchannels_out, bool dump, bool dump_mat, std::string dump_filename, double rate_hz)
{
    return rx_synch_observables_gs_sptr(new rx_synch_observables_gs(nchannels_in, nchannels_out, dump, dump_mat, dump_filename, rate_hz));
}


rx_synch_observables_gs::rx_synch_observables_gs(uint32_t nchannels_in,
    uint32_t nchannels_out,
    bool dump,
    bool dump_mat,
    std::string dump_filename, double rate_hz) : gr::block("rx_synch_observables_gs",
                                                     gr::io_signature::make(nchannels_in, nchannels_in, sizeof(Gnss_Synchro)),
                                                     gr::io_signature::make(nchannels_out, nchannels_out, sizeof(Gnss_Synchro))),
                                                 d_nchannels_in(nchannels_in),
                                                 d_nchannels_out(nchannels_out),
                                                 d_dump(dump),
                                                 d_dump_mat(dump_mat),
                                                 d_rate_hz(rate_hz),
                                                 d_dump_filename(dump_filename),
                                                 d_rx_epoch_offset(TimeInterval::Seconds(0.0))

{
    // PVT input message port
    this->message_port_register_in(pmt::mp("pvt_to_observables"));
    this->set_msg_handler(pmt::mp("pvt_to_observables"), boost::bind(&rx_synch_observables_gs::msg_handler_pvt_to_observables, this, _1));

    // initialize internal vars
    d_rx_id = 0;
    d_ninput_items_required.assign(d_nchannels_in, 0);
    d_ninput_items_required[d_nchannels_in - 1] = 1;
    d_current_measurements.resize(d_nchannels_out);
    empty_current_measurements();
    // The following sets this block to force downstream blocks to be low-latency
    this->set_max_noutput_items(1);

    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                        {
                            d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Observables dump enabled Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "Exception opening observables dump file " << e.what();
                        }
                }
        }
}


rx_synch_observables_gs::~rx_synch_observables_gs()
{
    if (d_dump_file.is_open() == true)
        {
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
                }
        }
    if (d_dump_mat)
        {
            save_matfile();
        }
}


int32_t rx_synch_observables_gs::save_matfile()
{
    // READ DUMP FILE
    std::string dump_filename = d_dump_filename;
    std::ifstream::pos_type size;
    int32_t number_of_double_vars = 7;
    int32_t epoch_size_bytes = sizeof(double) * number_of_double_vars * d_nchannels_out;
    std::ifstream dump_file;
    std::cout << "Generating .mat file for " << dump_filename << std::endl;
    dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try
        {
            dump_file.open(dump_filename.c_str(), std::ios::binary | std::ios::ate);
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Problem opening dump file:" << e.what() << std::endl;
            return 1;
        }
    // count number of epochs and rewind
    int64_t num_epoch = 0LL;
    if (dump_file.is_open())
        {
            size = dump_file.tellg();
            num_epoch = static_cast<int64_t>(size) / static_cast<int64_t>(epoch_size_bytes);
            dump_file.seekg(0, std::ios::beg);
        }
    else
        {
            return 1;
        }

    auto RX_time = std::vector<std::vector<double>>(d_nchannels_out, std::vector<double>(num_epoch));
    auto TOW_at_current_symbol_s = std::vector<std::vector<double>>(d_nchannels_out, std::vector<double>(num_epoch));
    auto Carrier_Doppler_hz = std::vector<std::vector<double>>(d_nchannels_out, std::vector<double>(num_epoch));
    auto Carrier_phase_cycles = std::vector<std::vector<double>>(d_nchannels_out, std::vector<double>(num_epoch));
    auto Pseudorange_m = std::vector<std::vector<double>>(d_nchannels_out, std::vector<double>(num_epoch));
    auto PRN = std::vector<std::vector<double>>(d_nchannels_out, std::vector<double>(num_epoch));
    auto Flag_valid_pseudorange = std::vector<std::vector<double>>(d_nchannels_out, std::vector<double>(num_epoch));

    try
        {
            if (dump_file.is_open())
                {
                    for (int64_t i = 0; i < num_epoch; i++)
                        {
                            for (uint32_t chan = 0; chan < d_nchannels_out; chan++)
                                {
                                    dump_file.read(reinterpret_cast<char *>(&RX_time[chan][i]), sizeof(double));
                                    dump_file.read(reinterpret_cast<char *>(&TOW_at_current_symbol_s[chan][i]), sizeof(double));
                                    dump_file.read(reinterpret_cast<char *>(&Carrier_Doppler_hz[chan][i]), sizeof(double));
                                    dump_file.read(reinterpret_cast<char *>(&Carrier_phase_cycles[chan][i]), sizeof(double));
                                    dump_file.read(reinterpret_cast<char *>(&Pseudorange_m[chan][i]), sizeof(double));
                                    dump_file.read(reinterpret_cast<char *>(&PRN[chan][i]), sizeof(double));
                                    dump_file.read(reinterpret_cast<char *>(&Flag_valid_pseudorange[chan][i]), sizeof(double));
                                }
                        }
                }
            dump_file.close();
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Problem reading dump file:" << e.what() << std::endl;
            return 1;
        }

    auto RX_time_aux = std::vector<double>(d_nchannels_out * num_epoch);
    auto TOW_at_current_symbol_s_aux = std::vector<double>(d_nchannels_out * num_epoch);
    auto Carrier_Doppler_hz_aux = std::vector<double>(d_nchannels_out * num_epoch);
    auto Carrier_phase_cycles_aux = std::vector<double>(d_nchannels_out * num_epoch);
    auto Pseudorange_m_aux = std::vector<double>(d_nchannels_out * num_epoch);
    auto PRN_aux = std::vector<double>(d_nchannels_out * num_epoch);
    auto Flag_valid_pseudorange_aux = std::vector<double>(d_nchannels_out * num_epoch);

    uint32_t k = 0U;
    for (int64_t j = 0; j < num_epoch; j++)
        {
            for (uint32_t i = 0; i < d_nchannels_out; i++)
                {
                    RX_time_aux[k] = RX_time[i][j];
                    TOW_at_current_symbol_s_aux[k] = TOW_at_current_symbol_s[i][j];
                    Carrier_Doppler_hz_aux[k] = Carrier_Doppler_hz[i][j];
                    Carrier_phase_cycles_aux[k] = Carrier_phase_cycles[i][j];
                    Pseudorange_m_aux[k] = Pseudorange_m[i][j];
                    PRN_aux[k] = PRN[i][j];
                    Flag_valid_pseudorange_aux[k] = Flag_valid_pseudorange[i][j];
                    k++;
                }
        }

    // WRITE MAT FILE
    mat_t *matfp;
    matvar_t *matvar;
    std::string filename = d_dump_filename;
    if (filename.size() > 4)
        {
            filename.erase(filename.end() - 4, filename.end());
        }
    filename.append(".mat");
    matfp = Mat_CreateVer(filename.c_str(), nullptr, MAT_FT_MAT73);
    if (reinterpret_cast<int64_t *>(matfp) != nullptr)
        {
            std::array<size_t, 2> dims{static_cast<size_t>(d_nchannels_out), static_cast<size_t>(num_epoch)};
            matvar = Mat_VarCreate("RX_time", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), RX_time_aux.data(), MAT_F_DONT_COPY_DATA);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("TOW_at_current_symbol_s", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), TOW_at_current_symbol_s_aux.data(), MAT_F_DONT_COPY_DATA);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Carrier_Doppler_hz", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), Carrier_Doppler_hz_aux.data(), MAT_F_DONT_COPY_DATA);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Carrier_phase_cycles", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), Carrier_phase_cycles_aux.data(), MAT_F_DONT_COPY_DATA);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Pseudorange_m", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), Pseudorange_m_aux.data(), MAT_F_DONT_COPY_DATA);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("PRN", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), PRN_aux.data(), MAT_F_DONT_COPY_DATA);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Flag_valid_pseudorange", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), Flag_valid_pseudorange_aux.data(), MAT_F_DONT_COPY_DATA);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);
        }
    Mat_Close(matfp);

    return 0;
}

void rx_synch_observables_gs::empty_current_measurements()
{
    Gnss_Synchro empty_obs = Gnss_Synchro();
    empty_obs.Flag_valid_pseudorange = false;
    empty_obs.Flag_valid_word = false;
    empty_obs.Flag_valid_acquisition = false;
    empty_obs.PRN = 0;
    empty_obs.fs = 0;

    int ch = 0;
    for (auto &this_measurement : d_current_measurements)
        {
            this_measurement = empty_obs;
            this_measurement.Channel_ID = ch++;
        }
}

bool RxSynch_pairCompare_gnss_synchro_sample_counter(const std::pair<int, Gnss_Synchro> &a, const std::pair<int, Gnss_Synchro> &b)
{
    return (a.second.Tracking_sample_counter) < (b.second.Tracking_sample_counter);
}


bool RxSynch_valueCompare_gnss_synchro_sample_counter(const Gnss_Synchro &a, unsigned long int b)
{
    return (a.Tracking_sample_counter) < (b);
}


bool RxSynch_valueCompare_gnss_synchro_receiver_time(const Gnss_Synchro &a, double b)
{
    return (((double)a.Tracking_sample_counter + a.Code_phase_samples) / (double)a.fs) < (b);
}


bool RxSynch_pairCompare_gnss_synchro_d_TOW(const std::pair<int, Gnss_Synchro> &a, const std::pair<int, Gnss_Synchro> &b)
{
    return (a.second.TOW_at_current_symbol_ms) < (b.second.TOW_at_current_symbol_ms);
}


bool RxSynch_valueCompare_gnss_synchro_d_TOW(const Gnss_Synchro &a, double b)
{
    return (a.TOW_at_current_symbol_ms) < (b);
}


void rx_synch_observables_gs::msg_handler_pvt_to_observables(pmt::pmt_t msg)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    try
        {
            if (pmt::any_ref(msg).type() == typeid(double))
                {
                    double clock_correction = boost::any_cast<double>(pmt::any_ref(msg));

                    std::cout << "Applying clock correction: " << clock_correction << " s" << std::endl;

                    // Ok, so we are applying a correction to our "receiver clock",
                    // this means updating the offset between the monontic rx_time (which is just
                    // the number of samples since we started) and the system times for each
                    // system:
                    //
                    // Tsys = Trx + Toffset
                    //
                    // So we now apply a correction dt to our receiver clock
                    // Tsys -> Tsys + dt
                    // => Toffset -> Toffset + dt
                    //
                    TimeConverter &converter = TimePoint::GetConverter();
                    std::pair<bool, TimePoint> conv_pair = converter.Convert(
                        TimePoint::MakeReceiver(TimeInterval::Seconds(0), d_rx_id),
                        ClockID::MakeGnss(GnssSystem::kGps));

                    if (conv_pair.first)
                        {
                            TimePoint newEpoch = conv_pair.second + TimeInterval::Seconds(clock_correction);
                            LOG(INFO) << "Updating receiver epoch to : "
                                      << std::setprecision(15) << newEpoch;
                            converter.SetReceiverEpoch(d_rx_id, newEpoch);

                            LOG(INFO) << "Receiver time " << std::setprecision(15) << d_receiver_time
                                      << " corresponds to "
                                      << std::setprecision(15) << newEpoch + d_receiver_time.TimeSinceEpoch();
                        }

                    // Next we need to update the measurement epoch, to ensure it is
                    // as close as possible to the Tsys boundaries
                    d_rx_epoch_offset += TimeInterval::Seconds(clock_correction);
                    d_rx_epoch_offset = d_rx_epoch_offset.RemainderMod(TimeInterval::Seconds(1.0 / d_rate_hz));
                    LOG(INFO) << "Updated rx epoch offset to " << d_rx_epoch_offset;
                }
        }
    catch (boost::bad_any_cast &e)
        {
            LOG(WARNING) << "msg_handler_pvt_to_observables Bad any cast!";
        }
}

int rx_synch_observables_gs::general_work(int noutput_items,
    gr_vector_int &ninput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    Gnss_Synchro **in = (Gnss_Synchro **)&input_items[0];    // Get the input pointer
    Gnss_Synchro **out = (Gnss_Synchro **)&output_items[0];  // Get the output pointer
    int n_outputs = 0;
    int n_consume[d_nchannels_in];

    /*
   * 1. Read the GNSS SYNCHRO objects from available channels.
   *  Multi-rate GNURADIO Block. Read how many input items are avaliable in each channel
   *  Record all synchronization data into queues
   */
    for (unsigned int i = 0; i < d_nchannels_out; i++)
        {
            n_consume[i] = ninput_items[i];  // Full throttle by default
            if (ninput_items[i] < 1)
                {
                    continue;
                }

            // Get the most recent measurement from this channel
            d_current_measurements[i] = in[i][n_consume[i] - 1];

            if (d_current_measurements[i].correlation_length_ms != 0)
                {
                    //d_ninput_items_required[i] = static_cast<int>(1.0/(
                    //(d_rate_hz * d_current_measurements[i].correlation_length_ms)/1000.0));
                    //d_ninput_items_required[i] = 1;
                    //d_ninput_items_required[d_nchannels_in-1] = 0;
                    //LOG(INFO) << d_current_measurements[i].Channel_ID << ": "
                    //<< "['" << d_current_measurements[i].System
                    //<< d_current_measurements[i].PRN << "']"
                    //<< " Set d_ninput_items_required = "
                    //<< d_ninput_items_required[i];
                }
            else
                {
                    LOG(INFO) << i << ": "
                              << "['" << d_current_measurements[i].System
                              << d_current_measurements[i].PRN << "']"
                              << " Invalid obs!";
                }
        }

    n_consume[d_nchannels_in - 1] = 0;  //ninput_items[d_nchannels_in - 1];
    // Do we need to generate any outputs?
    if (ninput_items[d_nchannels_in - 1] > 0)
        {
            n_consume[d_nchannels_in - 1] = 1;  //ninput_items[d_nchannels_in - 1];
            Gnss_Synchro &epoch_synchro = in[d_nchannels_in - 1][0];
            d_receiver_time = TimePoint::MakeReceiver(
                TimeInterval::Ticks(epoch_synchro.Tracking_sample_counter,
                    epoch_synchro.fs),
                d_rx_id);
            TimeConverter &converter = TimePoint::GetConverter();

            for (auto &the_measurement : d_current_measurements)
                {
                    if (the_measurement.PRN == 0)
                        {
                            if (the_measurement.Flag_valid_word)
                                {
                                    LOG(INFO) << "Odd measurement on channel "
                                              << the_measurement.Channel_ID << ": "
                                              << "['" << the_measurement.System
                                              << the_measurement.PRN << "']"
                                              << " TOW: " << the_measurement.TOW_at_current_symbol_ms
                                              << " Fd : " << the_measurement.Carrier_Doppler_hz;
                                }

                            continue;
                        }
                    ClockID gnss_clock_id = ClockID::MakeGnss(
                        static_cast<GnssSystem>(the_measurement.System));
                    TimePoint t_tx;
                    bool t_tx_valid = the_measurement.Flag_valid_word;
                    // Compute the transmit time if possible
                    if (t_tx_valid)
                        {
                            t_tx = TimePoint::MakeGnss(static_cast<GnssSystem>(the_measurement.System),
                                TimeInterval::Weeks(the_measurement.Week_at_current_symbol) +
                                    TimeInterval::MilliSeconds(
                                        the_measurement.TOW_at_current_symbol_ms) -
                                    TimeInterval::Ticks(the_measurement.Code_phase_samples, the_measurement.fs));
                        }

                    TimePoint t_rx = TimePoint::MakeReceiver(
                        TimeInterval::Ticks(the_measurement.Tracking_sample_counter,
                            the_measurement.fs),
                        d_rx_id);

                    TimePoint t_rx_gnss = t_rx;

                    // Try to conver the receiver time into the gnss system time frame

                    std::pair<bool, TimePoint> t_rx_conv_pair = converter.Convert(t_rx,
                        gnss_clock_id);

                    // Did the conversion succeed?
                    if (!t_rx_conv_pair.first)
                        {
                            // Can we initialise time from the measurement?
                            if (t_tx_valid)
                                {
                                    std::lock_guard<std::mutex> lock(d_mutex);
                                    // Compute the nominal receive time as t_tx + t_transit_nominal
                                    TimePoint t_rx_nominal = t_tx + TimeInterval::MilliSeconds(70);

                                    TimePoint t_rx_epoch_nominal = t_rx_nominal - t_rx.TimeSinceEpoch();

                                    LOG(INFO) << "Setting receiver start epoch to " << std::setprecision(15) << t_rx_epoch_nominal;
                                    LOG(INFO) << "\t t_tx " << std::setprecision(11) << t_tx;
                                    LOG(INFO) << "\t t_rx_nominal " << std::setprecision(11) << t_rx_nominal;
                                    LOG(INFO) << "\t t_rx " << std::setprecision(15) << d_receiver_time;

                                    converter.SetReceiverEpoch(t_rx.GetClockID().GetId(), t_rx_epoch_nominal);
                                    t_rx_conv_pair = converter.Convert(t_rx, gnss_clock_id);

                                    if (t_rx_conv_pair.first)
                                        {
                                            TimeInterval update_interval = TimeInterval::Seconds(1.0 / d_rate_hz);
                                            TimeInterval dt1 = t_rx.TimeSinceEpoch().RemainderMod(update_interval);
                                            TimeInterval dt2 = t_rx_nominal.TimeSinceEpoch().RemainderMod(update_interval);
                                            d_rx_epoch_offset = dt2 - dt1;
                                            LOG(INFO) << "Setting rx epoch offset to " << std::setprecision(6) << d_rx_epoch_offset;
                                        }
                                }
                        }

                    if (t_rx_conv_pair.first)
                        {
                            t_rx_gnss = t_rx_conv_pair.second;
                        }

                    the_measurement.RX_time = t_rx_gnss.TOW().AsSeconds();

                    // Compute the PR if possible
                    if (the_measurement.Flag_valid_word && t_rx_conv_pair.first)
                        {
                            const TimeInterval oneWeek = TimeInterval::Weeks(1);
                            the_measurement.Pseudorange_m =
                                (t_rx_gnss - t_tx).RemainderMod(oneWeek).AsSeconds() * GPS_C_M_S;
                            the_measurement.Flag_valid_pseudorange = true;
                        }

                    // Now propagate to the epoch
                    TimeInterval dt = d_receiver_time - (t_rx + d_rx_epoch_offset);

                    double dt_secs = dt.AsSeconds();

                    double wavelength = GPS_C_M_MS / GetFrequency(the_measurement.System, the_measurement.Signal);

                    double dcp = the_measurement.Carrier_Doppler_hz * dt_secs;
                    the_measurement.RX_time += dt_secs;
                    the_measurement.Carrier_phase_rads -= GPS_TWO_PI * dcp;
                    the_measurement.Pseudorange_m -= dcp * wavelength;
                    the_measurement.Tracking_sample_counter += static_cast<uint64_t>(dt_secs * the_measurement.fs + 0.5);

                    out[the_measurement.Channel_ID][0] = the_measurement;
                }

            // now we have used the current measurements, reset them:
            empty_current_measurements();

            if (d_dump == true)
                {
                    // MULTIPLEXED FILE RECORDING - Record results to file
                    try
                        {
                            double tmp_double;
                            for (unsigned int i = 0; i < d_nchannels_out; i++)
                                {
                                    tmp_double = out[i][0].RX_time;
                                    d_dump_file.write((char *)&tmp_double, sizeof(double));
                                    tmp_double = out[i][0].TOW_at_current_symbol_ms;
                                    d_dump_file.write((char *)&tmp_double, sizeof(double));
                                    tmp_double = out[i][0].Carrier_Doppler_hz;
                                    d_dump_file.write((char *)&tmp_double, sizeof(double));
                                    tmp_double = out[i][0].Carrier_phase_rads / GPS_TWO_PI;
                                    d_dump_file.write((char *)&tmp_double, sizeof(double));
                                    tmp_double = out[i][0].Pseudorange_m;
                                    d_dump_file.write((char *)&tmp_double, sizeof(double));
                                    tmp_double = out[i][0].PRN;
                                    d_dump_file.write((char *)&tmp_double, sizeof(double));
                                    tmp_double = out[i][0].Flag_valid_pseudorange;
                                    d_dump_file.write((char *)&tmp_double, sizeof(double));
                                    tmp_double = out[i][0].Tracking_sample_counter;
                                    d_dump_file.write((char *)&tmp_double, sizeof(double));
                                }
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "Exception writing observables dump file " << e.what();
                        }
                }


            n_outputs++;
        }


    //Multi-rate consume!
    for (unsigned int i = 0; i < d_nchannels_in; i++)
        {
            consume(i, n_consume[i]);  //which input, how many items
        }

    return n_outputs;
}

void rx_synch_observables_gs::forecast(
    int noutput_items,
    gr_vector_int &ninput_items_required)
{
    for (unsigned int i = 0; i < d_nchannels_in; i++)
        {
            ninput_items_required[i] = d_ninput_items_required[i] * noutput_items;
        }
}
