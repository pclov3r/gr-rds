/*
 * Copyright (C) 2014, 2016 Bastian Bloessl <bloessl@ccs-labs.org>
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

#ifndef INCLUDED_RDS_ENCODER_IMPL_H
#define INCLUDED_RDS_ENCODER_IMPL_H

#include <rds/encoder.h>
#include <gnuradio/thread/thread.h>
#include <time.h>
#include <vector>

namespace gr {
namespace rds {

class encoder_impl : public encoder
{
public:
	encoder_impl(unsigned char pty_locale, int pty, bool ms, std::string ps,
                 bool af, const std::vector<double>& af_list, bool tp, bool ta, bool tmc, bool ct,
                 int pi_country_code, int pi_coverage_area, int pi_reference_number,
                 std::string radiotext, bool ecc, unsigned char ecc_code);

    void set_ps(std::string ps) override;
    void set_af_list(const std::vector<double>& af_list) override;

private:
	~encoder_impl() override;

	// --- Member Functions ---
	int work(int noutput_items,
			gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items) override;
	void rebuild();
	void set_ms(bool ms);
	void set_tp(bool tp);
	void set_ta(bool ta);
	void set_pty(unsigned int pty);
	void set_pi(unsigned int pty);
	void set_radiotext(std::string text);
	void count_groups();
	void create_group(const int, const bool);
   	void generate_ct_group();
	void prepare_group0(const bool);
	void prepare_group1a();
	void prepare_group2(const bool);
	void prepare_group3a();
	void prepare_group4a(const time_t& time_to_encode);
	void prepare_group8a();
	void prepare_group11a();
	void prepare_buffer(int);
	unsigned int encode_af(double);
	unsigned int calc_syndrome(unsigned long, unsigned char);
	void rds_in(pmt::pmt_t msg);


	// --- Member Variables ---

	// RDS Configuration & Data
	gr::thread::mutex d_mutex;
	unsigned char   d_pty_locale;
	unsigned int    d_pi;
	unsigned char   d_pty;
	std::vector<double> d_af_list;
	unsigned char   d_ecc_code;

	// Feature-Enabling Flags
	bool d_ms;
	bool d_tp;
	bool d_ta;
	bool d_af;
	bool d_tmc;
	bool d_ct;
	bool d_ecc;

	// Data Buffers
	char d_radiotext[64]; // Using char is more standard for strings
	char d_ps[8];

	// Internal State & Buffers
	unsigned int  d_infoword[4];
	unsigned int  d_checkword[4];
	unsigned long d_block[4]; // Using unsigned long is safer for 26-bit values
	unsigned char **d_buffer;
	int           d_nbuffers;
	char          d_groups[32]; // Using char is sufficient for 0/1 flags

	// Message Segment Counters
	unsigned int d_ps_segment_index;
	unsigned int d_radiotext_segment_index;
	unsigned int d_tmc_segment_index;
	unsigned int d_af_index;

	// Streaming counters
	int    d_current_buffer;
	int    d_buffer_bit_counter;
	time_t d_last_ct_time;

    // State variables for the correct "injection" logic
    bool d_send_ct_next;
    bool d_is_sending_ct;
    unsigned char d_ct_buffer[104];

	// RDS-TMC Alert-C Data
	struct TmcAlertData {
		unsigned char duration_persistence;
		unsigned char extent;
		unsigned int event_code;
		unsigned int location_code;
	};
	TmcAlertData d_tmc_alert_data;
};

} /* namespace rds */
} /* namespace gr */

#endif /* INCLUDED_RDS_ENCODER_IMPL_H */
