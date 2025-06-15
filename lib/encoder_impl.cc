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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "encoder_impl.h"
#include "constants.h"
#include <gnuradio/io_signature.h>
#include <boost/spirit/include/qi.hpp>
#include <math.h>
#include <ctype.h>
#include <time.h>
#include <cstdio>
#include <iostream>
#include <vector>
#include <limits>

using namespace gr::rds;

// Default group repetition intervals, modeled on the mpx-gen/MiniRDS scheduler.
// The 'rate' parameter defines the approximate interval, in group slots, at which
// a group should be transmitted. A lower rate corresponds to a higher transmission frequency.
//
// RDS TIMING: The standard group rate is ~11.4 groups/sec (~87.5 ms per group).
// A group's approximate repetition frequency can be calculated as: (11.4 / rate) Hz.
//
// IMPORTANT: Rates for unimplemented groups are set to 0 to disable them. The
// original mpx-gen/MiniRDS scheduler rate is kept in a comment to guide future implementation.
//
// See mpx-gen/MiniRDS at https://github.com/Anthony96922/MiniRDS
const int DEFAULT_GROUP_RATES[16] = {
	4,   // Group 0A/0B (PS, AF): High repetition (~2.85 Hz)
	16,  // Group 1A (ECC): Low repetition (~0.71 Hz)
	8,   // Group 2A/2B (RadioText): Medium repetition (~1.42 Hz)
	16,  // Group 3A (ODA): Low repetition (~0.71 Hz)
	-1,  // Group 4A (Clock-Time): Special case, sent once per minute.
	0,   // Group 5: Not Implemented (mpxgen rate: 32, ~0.36 Hz)
	0,   // Group 6: Not Implemented (mpxgen rate: 32, ~0.36 Hz)
	0,   // Group 7: Not Implemented (mpxgen rate: 32, ~0.36 Hz)
	16,  // Group 8A (TMC/ODA): Low repetition (~0.71 Hz)
	0,   // Group 9: Not Implemented (mpxgen rate: 32, ~0.36 Hz)
	0,   // Group 10: Not Implemented (mpxgen rate: 16, ~0.71 Hz)
	16,  // Group 11A (ODA): Low repetition (~0.71 Hz)
	0,   // Group 12: Not Implemented (mpxgen rate: 32, ~0.36 Hz)
	0,   // Group 13: Not Implemented (mpxgen rate: 32, ~0.36 Hz)
	0,   // Group 14: Not Implemented (mpxgen rate: 4, ~2.85 Hz)
	0    // Group 15: Not Implemented (mpxgen rate: 32, ~0.36 Hz)
};

encoder_impl::encoder_impl(unsigned char pty_locale, int pty, bool ms, std::string ps,
                           bool af, const std::vector<double>& af_list, bool tp, bool ta, bool tmc, bool ct,
                           int pi_country_code, int pi_coverage_area, int pi_reference_number,
                           std::string radiotext, bool ecc, unsigned char ecc_code)
    : gr::sync_block("gr_rds_encoder", gr::io_signature::make(0, 0, 0), gr::io_signature::make(1, 1, sizeof(unsigned char))),

      // RDS Parameters and Flags
      d_pty_locale(pty_locale),   // PTY display standard (Europe or North America)
      d_pi(0),                    // Program Identification, calculated in constructor body
      d_pty(pty),                 // programm type (education)
      d_af_list(af_list),         // alternate frequency list
      d_ecc_code(ecc_code),       // Value for the new ECC feature
      d_ms(ms),                   // music/speech switch (1=music)
      d_tp(tp),                   // traffic programm
      d_ta(ta),                   // traffic announcement

      // Feature Flags
      d_af(af),                   // Enable sending Alternate Frequency in Group 0A
      d_tmc(tmc),                 // Enable sending TMC groups
      d_ct(ct),                   // Enable sending Clock-Time group
      d_ecc(ecc),                 // Enable sending Extended Country Code group

      // Internal State
      d_ps_segment_index(0),
      d_radiotext_segment_index(0),
      d_tmc_segment_index(0),
      d_af_index(0),
      d_buffer_bit_counter(0),
      d_last_ct_time(0),

      // Hard-coded TMC Data
      d_tmc_alert_data({3, 2, 1340, 11023})
{
	message_port_register_in(pmt::mp("rds in"));
	set_msg_handler(pmt::mp("rds in"), [this](pmt::pmt_t msg) { this->rds_in(msg); });

	std::memset(d_infoword,    0, sizeof(d_infoword));
	std::memset(d_checkword,   0, sizeof(d_checkword));
	std::memset(d_groups,      0, sizeof(d_groups));
    std::memset(d_radiotext,   ' ', sizeof(d_radiotext));
    std::memset(d_ps,          ' ', sizeof(d_ps));
    std::memset(d_current_group_buffer, 0, sizeof(d_current_group_buffer));

	if (pi_country_code == 0) {
		d_pi = pi_reference_number;
	} else {
		d_pi = (pi_country_code & 0xF) << 12 | (pi_coverage_area & 0xF) << 8 | (pi_reference_number);
	}

	set_radiotext(radiotext);
	set_ps(ps);

	// Configure which groups are set based on flags
	d_groups[0] = 1;  // 0A: basic tuning and switching
	d_groups[2] = 1;  // 2A: radio text
	d_groups[11] = 1; // 11A: Open Data Applications (in-house)
    if (d_ecc) { d_groups[1] = 1; } // 1A
    if (d_tmc) {
        d_groups[3] = 1; // 3A: announce TMC
        d_groups[8] = 1; // 8A: TMC data
    }

    rebuild();
}

encoder_impl::~encoder_impl() {
}

// Configures the dynamic scheduler based on currently enabled groups.
// This function is invoked on initialization and upon runtime parameter changes.
//
// It populates the scheduler state for each potential group (0A-15B) with its
// configured rate. The counter for each group is initialized to its rate - 1.
// This specific initialization is critical as it ensures that upon startup,
// all enabled groups are considered "urgent" and are transmitted once in a balanced
// sequence before the regular scheduling cadence begins.
void encoder_impl::rebuild() {
    gr::thread::scoped_lock lock(d_mutex);

    // Reset transmission state
    d_buffer_bit_counter = 0;
    d_last_ct_time = 0;
    d_ps_segment_index = 0;
    d_radiotext_segment_index = 0;
    d_tmc_segment_index = 0;
    d_af_index = 0;

    // Configure the scheduler state for all 32 possible groups
    for (int i = 0; i < 32; i++) {
        if (d_groups[i]) {
            int group_type = i % 16;
            d_scheduler_states[i].rate = DEFAULT_GROUP_RATES[group_type];
            if (d_scheduler_states[i].rate > 0) {
                d_scheduler_states[i].counter = d_scheduler_states[i].rate - 1;
            } else {
                d_scheduler_states[i].counter = 0;
            }
        } else {
            d_scheduler_states[i].rate = 0; // A rate of 0 disables the group from scheduling.
            d_scheduler_states[i].counter = 0;
        }
    }
}

void encoder_impl::rds_in(pmt::pmt_t msg) {
	if(!pmt::is_pair(msg)) return;

	using boost::spirit::qi::phrase_parse;
	using boost::spirit::qi::lexeme;
	using boost::spirit::qi::char_;
	using boost::spirit::qi::hex;
	using boost::spirit::qi::uint_;
	using boost::spirit::qi::bool_;
	using boost::spirit::qi::double_;
	using boost::spirit::qi::space;
	using boost::spirit::qi::lit;

	int msg_len = pmt::blob_length(pmt::cdr(msg));
	std::string in = std::string((char*)pmt::blob_data(pmt::cdr(msg)), msg_len);

	unsigned int ui1;
	std::string s1;
	bool b1;
    std::vector<double> vd1;

	if(phrase_parse(in.begin(), in.end(), "pty" >> (("0x" >> hex) | uint_), space, ui1)) {
		set_pty(ui1);
	} else if(phrase_parse(in.begin(), in.end(), "text" >> lexeme[+(char_ - '\n')] >> -lit("\n"), space, s1)) {
		set_radiotext(s1);
	} else if(phrase_parse(in.begin(), in.end(), "ps" >> lexeme[+(char_ - '\n')] >> -lit("\n"), space, s1)) {
		set_ps(s1);
	} else if(phrase_parse(in.begin(), in.end(), "ta" >> bool_, space, b1)) {
		set_ta(b1);
	} else if(phrase_parse(in.begin(), in.end(), "tp" >> bool_, space, b1)) {
		set_tp(b1);
	} else if(phrase_parse(in.begin(), in.end(), "ms" >> bool_, space, b1)) {
		set_ms(b1);
	} else if(phrase_parse(in.begin(), in.end(), "pi" >> lit("0x") >> hex, space, ui1)) {
		set_pi(ui1);
	} else if(phrase_parse(in.begin(), in.end(), "af_list" >> +double_, space, vd1)) {
        set_af_list(vd1);
    } else {
		std::cout << "RDS: command not understood" << std::endl;
	}

	rebuild();
}

void encoder_impl::set_ms(bool ms) { d_ms = ms; }
void encoder_impl::set_tp(bool tp) { d_tp = tp; }
void encoder_impl::set_ta(bool ta) { d_ta = ta; }
void encoder_impl::set_pty(unsigned int pty) { if (pty <= 31) d_pty = pty; }
void encoder_impl::set_pi(unsigned int pi) { if (pi <= 0xFFFF) d_pi = pi; }

void encoder_impl::set_af_list(const std::vector<double>& af_list)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_af_list = af_list;
    d_af_index = 0;
    rebuild();
}

void encoder_impl::set_radiotext(std::string text) {
	size_t len = std::min(sizeof(d_radiotext), text.length());
	std::memset(d_radiotext, ' ', sizeof(d_radiotext));
	std::memcpy(d_radiotext, text.c_str(), len);
}

void encoder_impl::set_ps(std::string ps) {
	size_t len = std::min(sizeof(d_ps), ps.length());
	std::memset(d_ps, ' ', sizeof(d_ps));
	std::memcpy(d_ps, ps.c_str(), len);
}

/* see Annex B, page 64 of the standard */
unsigned int encoder_impl::calc_syndrome(unsigned long message, unsigned char mlen) {
	unsigned long reg = 0;
	const unsigned long poly = 0x5B9;
	const unsigned char plen = 10;
	for (unsigned int i = mlen; i > 0; i--)  {
		reg = (reg << 1) | ((message >> (i - 1)) & 0x01);
		if (reg & (1 << plen)) reg = reg ^ poly;
	}
	for (unsigned int i = plen; i > 0; i--) {
		reg = reg << 1;
		if (reg & (1 << plen)) reg = reg ^ poly;
	}
	return reg & ((1 << plen) - 1);
}

/*
 * Encodes a frequency value (in MHz) into its corresponding 8-bit RDS AF code.
 * This implementation follows the specification for AF Method A, which is
 * detailed in the RDS standard IEC 62106-2:2018, Annex E, Table E.1. */
unsigned int encoder_impl::encode_af(const double af) {
	unsigned int af_code = 0;
	if(( af >= 87.6) && (af <= 107.9))
		af_code = nearbyint((af - 87.5) * 10);
	else if((af >= 153) && (af <= 279))
		af_code = nearbyint((af - 144) / 9);
	else if((af >= 531) && (af <= 1602))
		af_code = nearbyint((af - 531) / 9 + 16);
	return af_code;
}

/* create the 4 infowords, according to group type.
 * then calculate checkwords and put everything in the groups */
void encoder_impl::create_group(const int group_type, const bool AB) {
	d_infoword[0] = d_pi;
	d_infoword[1] = (((group_type & 0xf) << 12) | (AB << 11) | (d_tp << 10) | (d_pty << 5));

	if(group_type == 0) prepare_group0(AB);
	else if(group_type == 1) prepare_group1a();
	else if(group_type == 2) prepare_group2(AB);
	else if(group_type == 3) prepare_group3a();
	else if(group_type == 4) prepare_group4a(time(NULL) + 60);
	else if(group_type == 8) prepare_group8a();
	else if(group_type == 11) prepare_group11a();

	for(int i= 0; i < 4; i++) {
		d_checkword[i]=calc_syndrome(d_infoword[i], 16);
		d_block[i] = ((d_infoword[i] & 0xffff) << 10) | (d_checkword[i] & 0x3ff);
		// add the offset word
		if((i == 2) && AB) d_block[2] ^= offset_word[4];
		else d_block[i] ^= offset_word[i];
	}

	prepare_buffer();
}

void encoder_impl::prepare_group0(const bool AB) {
	d_infoword[1] |= (d_ta << 4) | (d_ms << 3);
	//FIXME: make DI configurable
	if(d_ps_segment_index == 3)
		d_infoword[1] |= 0x5;  // d0=1 (stereo), d1-3=0
	d_infoword[1] |= (d_ps_segment_index & 0x3);
	if(!AB) { // This is Group 0A
        if (d_af && !d_af_list.empty()) { // AF is enabled and list is not empty
            // Get the current AF from the list
            double current_af_freq = d_af_list[d_af_index];
            unsigned int af_code = encode_af(current_af_freq / 1000000.0);

            // The code 225 means "1 AF follows". We send one at a time.
            d_infoword[2] = (225 << 8) | (af_code & 0xff);

            // Cycle to the next AF for the next Group 0A transmission
            d_af_index = (d_af_index + 1) % d_af_list.size();
        } else { // AF is disabled or list is empty: repeat PI code for robustness
            d_infoword[2] = d_pi;
        }
	} else { // This is Group 0B
		d_infoword[2] = d_pi;
	}
	d_infoword[3] = (d_ps[2 * d_ps_segment_index] << 8) | d_ps[2 * d_ps_segment_index + 1];
	d_ps_segment_index = (d_ps_segment_index + 1) % 4;
}

void encoder_impl::prepare_group1a(void) {
	d_infoword[2] = (0x80 << 8) | d_ecc_code;
	d_infoword[3] = 0;
}

void encoder_impl::prepare_group2(const bool AB) {
	d_infoword[1] |= ((AB << 4) | (d_radiotext_segment_index & 0xf));
	if(!AB) {
		d_infoword[2] = (d_radiotext[d_radiotext_segment_index * 4] << 8 | d_radiotext[d_radiotext_segment_index * 4 + 1]);
		d_infoword[3] = (d_radiotext[d_radiotext_segment_index * 4 + 2] << 8 | d_radiotext[d_radiotext_segment_index * 4 + 3]);
	} else {
		d_infoword[2] = d_pi;
		d_infoword[3] = (d_radiotext[d_radiotext_segment_index * 2] << 8 | d_radiotext[d_radiotext_segment_index * 2 + 1]);
	}
	d_radiotext_segment_index = (d_radiotext_segment_index + 1) % 16;
}

void encoder_impl::prepare_group3a(void) {
	if(d_tmc_segment_index) {
		d_infoword[1] |= (0x31d0 & 0x1f);
		d_infoword[2] = 0x6280;
		d_infoword[3] = 0xcd46;
	} else {
		d_infoword[1] |= (0x31d0 & 0x1f);
		d_infoword[2] = 0x0066;
		d_infoword[3] = 0xcd46; // AID for TMC (Alert C)
	}
	d_tmc_segment_index = (d_tmc_segment_index + 1) % 2;
}

void encoder_impl::prepare_group4a(const time_t& time_to_encode) {
    tm utc_struct;
    tm local_struct;
    #if defined(_WIN32)
        gmtime_s(&utc_struct, &time_to_encode);
        localtime_s(&local_struct, &time_to_encode);
    #else // POSIX
        gmtime_r(&time_to_encode, &utc_struct);
        localtime_r(&time_to_encode, &local_struct);
    #endif

	int minute = utc_struct.tm_min;
	int hour = utc_struct.tm_hour;
	int day = utc_struct.tm_mday;
	int month = utc_struct.tm_mon + 1;
	int year = utc_struct.tm_year + 1900;

    int a = (14 - month) / 12;
    int y = year + 4800 - a;
    int m_calc = month + 12 * a - 3;
    int jdn = day + (153 * m_calc + 2) / 5 + 365 * y + y / 4 - y / 100 + y / 400 - 32045;
    int mjd = jdn - 2400001;

    long offset_secs = 0;
    #if defined(_WIN32)
        long timezone_sec = 0;
        _get_timezone(&timezone_sec);
        offset_secs = -timezone_sec;
        if (local_struct.tm_isdst > 0) {
            long dst_sec = 0;
            _get_dstbias(&dst_sec);
            offset_secs += dst_sec;
        }
    #else
        offset_secs = local_struct.tm_gmtoff;
    #endif

    double offset_in_half_hours = round((double)offset_secs / 1800.0);
    unsigned int offset_sign_bit = (offset_in_half_hours < 0) ? 1 : 0;
    unsigned int offset_magnitude = (unsigned int)fabs(offset_in_half_hours);
    unsigned int offset_code = (offset_sign_bit << 5) | (offset_magnitude & 0x1F);

	d_infoword[1] |= ((mjd >> 15) & 0x3);
	d_infoword[2] = (((mjd >> 7) & 0xff) << 8) | ((mjd & 0x7f) << 1) | ((hour >> 4) & 0x1);
	d_infoword[3] = ((hour & 0xf) << 12) | (minute << 6) | (offset_code & 0x3F);
}

// TMC Alert-C
void encoder_impl::prepare_group8a(void) {
	d_infoword[1] |= (1 << 3) | (d_tmc_alert_data.duration_persistence & 0x7);
	d_infoword[2] = (1 << 15) | ((d_tmc_alert_data.extent & 0x7) << 11) | (d_tmc_alert_data.event_code & 0x7ff);
	d_infoword[3] = d_tmc_alert_data.location_code;
}

// for now single-group only
void encoder_impl::prepare_group11a(void) {
	d_infoword[1] |= (0xb1c8 & 0x1f);
	d_infoword[2] = 0x2038;
	d_infoword[3] = 0x4456;
}

void encoder_impl::prepare_buffer() {
	for(int q = 0; q < 104; q++) {
		int a = q / 26;
		int b = 25 - (q % 26);
		d_current_group_buffer[q] = (unsigned char)(d_block[a] >> b) & 0x1;
	}
}

// Main processing loop implementing the dynamic RDS group scheduler.
// At the beginning of each 104-bit group slot, this function determines which
// RDS group to transmit next based on a two-stage process:
// 1. High-Priority Injection: Checks for time-critical groups (e.g., Group 4A).
// 2. Urgency-Based Scheduling: If no high-priority group is due, it selects the
//    regular group with the highest "urgency" score.
//
// Once a group is selected, its bitstream is generated into a buffer, and the
// scheduler's internal state is updated for the next cycle.
int encoder_impl::work (int noutput_items,
		gr_vector_const_void_star &input_items,
		gr_vector_void_star &output_items) {

	gr::thread::scoped_lock lock(d_mutex);
	unsigned char *out = (unsigned char *) output_items[0];

	for(int i = 0; i < noutput_items; i++) {
        // A new group must be scheduled and generated at the start of each 104-bit block.
        if (d_buffer_bit_counter == 0) {
            int group_to_send = -1;
            bool ab_flag_to_send = false;
            int chosen_idx = -1;

            // 1. High-Priority Override: Check for time-sensitive groups.
            // Group 4A (Clock-Time) is injected once per minute, overriding the regular schedule.
	    // NOTE: Due to significant buffering throughout the signal chain (GNU Radio, OS, SDR hardware),
	    // the actual transmission of this group will be noticeably delayed relative to the top of the minute.
	    // The `prepare_group4a` function attempts to compensate for this by encoding the time
	    // for the *upcoming* minute.
            time_t now;
            time(&now);
            if (d_ct && (now / 60) != (d_last_ct_time / 60)) {
                d_last_ct_time = now;
                group_to_send = 4;
                ab_flag_to_send = false; // Group 4A
            }

            // 2. Regular Scheduling: Find the most "urgent" group if no high-priority group was sent.
            if (group_to_send == -1) {
                // Initialize max_urgency to the lowest possible float value. This is the
                // robust C++ way to find a maximum value and guarantees that the first
                // valid group will always become the initial candidate, preventing a stall.
                float max_urgency = std::numeric_limits<float>::lowest();

                // The urgency is a ratio of the time since last transmission (counter)
                // to the desired interval (rate). A higher ratio indicates higher urgency.
                for (int j = 0; j < 32; j++) {
                    if (d_scheduler_states[j].rate > 0) { // Check if group is schedulable
                        float urgency = (float)d_scheduler_states[j].counter / (float)d_scheduler_states[j].rate;
                        if (urgency > max_urgency) {
                            max_urgency = urgency;
                            chosen_idx = j;
                        }
                    }
                }

                if (chosen_idx != -1) {
                    group_to_send = chosen_idx % 16;
                    ab_flag_to_send = (chosen_idx >= 16);
                }
            }

            // 3. Generate the bitstream for the chosen group.
            if (group_to_send != -1) {
                create_group(group_to_send, ab_flag_to_send);
            } else {
                // Failsafe: If no group is schedulable, transmit a null group.
                std::memset(d_current_group_buffer, 0, sizeof(d_current_group_buffer));
            }

            // 4. Update Scheduler State. This is the core of the mpx-gen algorithm.
            if (chosen_idx != -1) { // Only update counters if a regular group was chosen.
                for (int j = 0; j < 32; j++) {
                    if (d_scheduler_states[j].rate > 0) { // If group is enabled
                        if (j == chosen_idx) {
                            // The chosen group has its urgency "paid off".
                            d_scheduler_states[j].counter -= d_scheduler_states[j].rate;
                        } else {
                            // All other groups become slightly more urgent for the next slot.
                            d_scheduler_states[j].counter++;
                        }
                    }
                }
            }
        }

        // 5. Output the next bit from the currently generated group.
        out[i] = d_current_group_buffer[d_buffer_bit_counter];

        // Advance the bit counter for the current 104-bit group.
		if(++d_buffer_bit_counter > 103) {
			d_buffer_bit_counter = 0; // Reset for the next group slot.
		}
	}

	return noutput_items;
}

encoder::sptr encoder::make (unsigned char pty_locale, int pty, bool ms,
		std::string ps, bool af, const std::vector<double>& af_list, bool tp,
		bool ta, bool tmc, bool ct, int pi_country_code, int pi_coverage_area,
		int pi_reference_number, std::string radiotext, bool ecc, unsigned char ecc_code) {

	return gnuradio::get_initial_sptr(
			new encoder_impl(pty_locale, pty, ms, ps, af, af_list, tp, ta,
                    tmc, ct, pi_country_code, pi_coverage_area, pi_reference_number,
					radiotext, ecc, ecc_code));
}
