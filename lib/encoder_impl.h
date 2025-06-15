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
private:
    // This struct holds the dynamic state for the mpx-gen style scheduler.
    // Each of the 32 possible RDS groups (0A-15B) has one of these.
    struct GroupSchedulerState {
        int rate;    // The repetition interval (e.g., 4 means "send every ~4 slots").
        int counter; // Tracks how "overdue" the group is for transmission.
    };

public:
	encoder_impl(unsigned char pty_locale, int pty, bool ptyn, std::string ptyn_str, bool ms,
                 bool di_stereo, bool di_artificial_head, bool di_compressed, bool di_dynamic_pty,
                 std::string ps, bool af, const std::vector<double>& af_list, bool tp, bool ta, bool tmc, bool ct,
                 int pi_country_code, int pi_coverage_area, int pi_reference_number,
                 std::string radiotext, bool ecc, unsigned char ecc_code);

    // Public API functions
    void set_ps(std::string ps) override;
    void set_af_list(const std::vector<double>& af_list) override;
    void set_ptyn(std::string ptyn_str) override;

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
   	void set_di_stereo(bool stereo);
    	void set_di_artificial_head(bool artificial_head);
    	void set_di_compressed(bool compressed);
    	void set_di_dynamic_pty(bool dynamic_pty);
    	void set_af_enabled(bool af);
    	void set_ptyn_enabled(bool ptyn);
    	void set_tmc_enabled(bool tmc);
    	void set_ct_enabled(bool ct);
    	void set_ecc_enabled(bool ecc);
    	void set_ecc_code(unsigned char ecc_code);
	void create_group(const int, const bool);
	void prepare_group0(const bool);
	void prepare_group1a();
	void prepare_group2(const bool);
	void prepare_group3a();
	void prepare_group4a(const time_t& time_to_encode);
	void prepare_group8a();
    	void prepare_group10a();
	void prepare_group11a();
	void prepare_buffer();
	unsigned int encode_af(double);
	unsigned int calc_syndrome(unsigned long, unsigned char);
	void rds_in(pmt::pmt_t msg);


	// --- Member Variables ---
	gr::thread::mutex d_mutex;

	// Core RDS Parameters
	unsigned int    d_pi;
	unsigned char   d_pty;
	unsigned char   d_pty_locale;
	unsigned char   d_ecc_code;
	std::vector<double> d_af_list;

	// Dynamic Broadcast Flags
	bool d_ms;

	// Decoder Identification Flags
    	bool d_di_stereo;
    	bool d_di_artificial_head;
    	bool d_di_compressed;
    	bool d_di_dynamic_pty;

	// Other Broadcast Flags
	bool d_tp;
	bool d_ta;

	// Feature-Enable Flags
	bool d_af;
	bool d_tmc;
	bool d_ct;
	bool d_ecc;
    	bool d_ptyn;

	// Data Buffers
	char d_ps[8];
	char d_radiotext[64];
    	char d_ptyn_str[8];

	// Internal State for Group Generation
	unsigned int  d_infoword[4];
	unsigned int  d_checkword[4];
	unsigned long d_block[4];
	char          d_groups[32];

	// Message Segment Counters
	unsigned int d_ps_segment_index;
	unsigned int d_radiotext_segment_index;
	unsigned int d_tmc_segment_index;
	unsigned int d_af_index;
    	unsigned int d_ptyn_segment_index;
    	bool         d_ptyn_ab_flag;

	// State for Dynamic Scheduler
    	GroupSchedulerState d_scheduler_states[32];
    	unsigned char d_current_group_buffer[104];

	// Streaming counters
	int    d_buffer_bit_counter;
	time_t d_last_ct_time;
    	bool   d_rebuild_needed; // Flag to signal a safe state reset is needed

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
