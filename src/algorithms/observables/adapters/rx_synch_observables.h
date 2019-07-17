/*!
 * \file rx_synch_observables.h
 * \brief Implementation of an adapter of an observables block accepting all kind
 * of signals to a ObservablesInterface
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas 2013. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_RX_SYNCH_OBSERVABLES_H_
#define GNSS_SDR_RX_SYNCH_OBSERVABLES_H_

#include <string>
#include "observables_interface.h"
#include "rx_synch_observables_gs.h"


class ConfigurationInterface;

/*!
 * \brief This class implements an ObservablesInterface for observables of all kind of GNSS signals
 */
class RxSynchObservables : public ObservablesInterface
{
public:
    RxSynchObservables(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams);

    virtual ~RxSynchObservables();

    std::string role() override
    {
        return role_;
    }

    //!  Returns "RxSynch_Observables"
    std::string implementation() override
    {
        return "RxSynch_Observables";
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    void reset() override
    {
        return;
    }

    //! All blocks must have an item_size() function implementation
    size_t item_size() override
    {
        return sizeof(gr_complex);
    }

private:
    rx_synch_observables_gs_sptr observables_;
    bool dump_;
    bool dump_mat_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif
