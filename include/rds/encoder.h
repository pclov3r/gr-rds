/*
 * Copyright (C) 2014 Bastian Bloessl <bloessl@ccs-labs.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDED_RDS_ENCODER_H
#define INCLUDED_RDS_ENCODER_H

#include <rds/api.h>
#include <gnuradio/sync_block.h>
#include <vector>

namespace gr {
namespace rds {

class RDS_API encoder : virtual public gr::sync_block
{
public:
	typedef std::shared_ptr<encoder> sptr;
	static sptr make(unsigned char pty_locale,
                     int pty,
                     bool ptyn,
                     std::string ptyn_str,
                     bool ms,
                     bool di_stereo,
                     bool di_artificial_head,
                     bool di_compressed,
                     bool di_dynamic_pty,
                     std::string ps,
                     bool af,
                     const std::vector<double>& af_list,
                     bool tp,
                     bool ta,
                     bool tmc,
                     bool ct,
                     int pi_country_code,
                     int pi_coverage_area,
                     int pi_reference_number,
                     std::string radiotext,
                     bool enable_ecc,
                     unsigned char ecc,
                     int max_latency = -1);

    virtual void set_ps(std::string ps) = 0;
    virtual void set_af_list(const std::vector<double>& af_list) = 0;
    virtual void set_ptyn(std::string ptyn_str) = 0;
};

}
}

#endif /* INCLUDED_RDS_ENCODER_H */
