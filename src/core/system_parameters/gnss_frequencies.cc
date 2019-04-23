/*!
 * \file gnss_frequencies.h
 * \brief  GNSS Frequencies
 * \author Carles Fernandez, 2017. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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
#include "gnss_frequencies.h"
double GetFrequency( char System, char Signal[3], int freq_num )
{
    double ret = -1.0;
    double df = 0.0;

    switch( System )
    {
        case 'R' : //GLONASS
            if( Signal[0] == '1' )
            {
                ret = FREQ1_GLO;
                df = DFRQ1_GLO;
            }
            else if( Signal[0] == '2' )
            {
                ret = FREQ2_GLO;
                df = DFRQ2_GLO;
            }
            else if( Signal[0] == '3' )
            {
                ret = FREQ3_GLO;
            }
            ret += freq_num*df;
            break;
        case 'C' : // BeiDou
            if( Signal[0] == '2' )
                ret = FREQ1_BDS;
            else if( Signal[0] == '7' )
                ret = FREQ2_BDS;
            else if( Signal[0] == '6' )
                ret = FREQ3_BDS;
            break;

        case 'G' : // GPS
            if( Signal[0] == '1' )
                ret = FREQ1;
            else if( Signal[0] == '2' )
                ret = FREQ2;
            else if( Signal[0] == '5' )
                ret = FREQ5;
            break;

        case 'E' : // Galileo
            if( Signal[0] == '1' )
                ret = FREQ1;
            else if( Signal[0] == '5' )
                ret = FREQ5;
            else if( Signal[0] == '7' )
                ret = FREQ7;
            else if( Signal[0] == '8' )
                ret = FREQ8;
            else if( Signal[0] == '6' )
                ret = FREQ6;
            break;

        case 'S' : // SBAS
            if( Signal[0] == '1' )
                ret = FREQ1;
            else if( Signal[0] == '5' )
                ret = FREQ5;
            break;

        case 'J': // QZSS
            if( Signal[0] == '1' )
                ret = FREQ1;
            else if( Signal[0] == '5' )
                ret = FREQ5;
            else if( Signal[0] == '2' )
                ret = FREQ2;
            else if( Signal[0] == '6' )
                ret = FREQ6;
            break;

        case 'I' : // IRNSS
            if( Signal[0] == '5' )
                ret = FREQ5;
            else if( Signal[0] == '9' )
                ret = FREQ9;

            break;

    }

    return ret;

}

