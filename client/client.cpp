/* ******************************************************************************
 * Copyright (c) 2015-2018 Google, Inc.  All rights reserved.
 * ******************************************************************************/

/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of VMware, Inc. nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL VMWARE, INC. OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#include "dr_api.h"
#include "drmgr.h"
#include "drx.h"
#include "client_tools.h"
#include "avx512ctx-shared.h"
#include <bits/stdc++.h>
#include <stdlib.h> /* qsort */
#include <string.h>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <ios>
#include <sstream>
#include <iostream>
#include <vector>
#include <deque>
#include <iomanip>

using namespace std;

// print information about every gather/scatter operation encountered

// #define PRINT_ALL_OCC

// print information about patterns as they are discovered
// #define PRINT_PATTERN

// print all occurrences of all patterns
// otherwise, occurrences of each pattern are truncated to the first 100
// #define PRINT_ALL_OCC_FINAL

// minimum number of occurrences of a pattern to print
#define MIN_NUM_OCC 1

// Required: MAX_BASE_DELTA_PATTERN_QUEUE > 2 * MAX_BASE_DELTA_PATTERN_PERIOD
#define MAX_BASE_DELTA_PATTERN_PERIOD 20
#define MAX_BASE_DELTA_PATTERN_QUEUE 50
#define MAX_INDEX_PATTERN 16


struct PatternStruct {
    // note that using null as an indication of no previous base value
    // is not possible due to some gather/scatter instructions having
    // null as a valid base register value, which indicates that the
    // base register value is 0.
    bool no_prev_base = true;
    ptrdiff_t prev_base;

    // if a pattern is not currently being matched, is used to find a pattern
    // if a pattern is currently being matched, is used to store the deltas to attempt to match
    array<ptrdiff_t, MAX_BASE_DELTA_PATTERN_QUEUE> base_delta_queue;
    int base_delta_queue_num_elem = 0;

    // the pattern which is currently being matched
    array<ptrdiff_t, MAX_BASE_DELTA_PATTERN_PERIOD> base_delta_patt;
    int base_delta_patt_num_elem = 0;

    // the number of times the current pattern has been matched so far
    // if no pattern is currently being matched, this value is 0
    unsigned long long num_occ = 0;

    bool no_index_patt = true;
    array<ptrdiff_t, MAX_INDEX_PATTERN> index_patt;
    int index_patt_num_elem = 0;
};

struct per_thread_t {
    // map from is_write (false(0) if gather, true(1) if scatter) to current pattern
    PatternStruct pattern_map[2];
};

static void event_thread_init(void *drcontext);
static dr_emit_flags_t event_app_instruction(void *drcontext, void *tag, instrlist_t *bb, instr_t *instr, bool for_trace, bool translating, void *user_data);
static void read_instr_reg_state(app_pc instr_addr, int base_regno);
static bool match_base_delta_pattern(PatternStruct &patt, bool use_existing_pattern);
static void record_and_print_remaining_pattern_occurrences(PatternStruct &patt, bool is_write);
static void record_and_print_pattern_occurrence(PatternStruct &patt, bool is_write);
static void record_pattern_occurrence(PatternStruct &patt, bool is_write);
static void print_pattern(PatternStruct &patt, bool is_write);
static void event_thread_exit(void *drcontext);
static void event_exit(void);

// map from is_write
//     to   map from    index_patt
//              to      map from    base_delta_patt
//                                  to deque of num_occ
static void *count_mutex;
static unordered_map <bool, map<vector<ptrdiff_t>, map<vector<ptrdiff_t>, vector<unsigned long long>>>> pattern_count;
static int tls_idx = 0;

#ifdef PRINT_PATTERN
static void *print_pattern_mutex;
#endif

DR_EXPORT void dr_client_main(client_id_t id, int argc, const char *argv[]) {
    if (!drmgr_init())
        DR_ASSERT(false);
    drx_init();

    /* Register events: */
    dr_register_exit_event(event_exit);
    if (!drmgr_register_thread_init_event(event_thread_init) ||
        !drmgr_register_thread_exit_event(event_thread_exit) ||
        !drmgr_register_bb_instrumentation_event(NULL, event_app_instruction, NULL)) {
        DR_ASSERT(false);
    }
    tls_idx = drmgr_register_tls_field();
    count_mutex = dr_mutex_create();

#ifdef PRINT_PATTERN
    print_pattern_mutex = dr_mutex_create();
#endif

    dr_log(NULL, DR_LOG_ALL, 1, "Client 'Gather Scatter Pattern Identification' initializing\n");
    if (dr_is_notify_on()) {
        dr_fprintf(STDOUT, "Client Gather Scatter Pattern Identification is running\n");
    }
}

static void event_thread_init(void *drcontext) {
    per_thread_t *data = (per_thread_t *)dr_thread_alloc(drcontext, sizeof(per_thread_t));
    drmgr_set_tls_field(drcontext, tls_idx, data);
    data->pattern_map[false].base_delta_patt_num_elem = 0;
    data->pattern_map[false].index_patt_num_elem = 0;
    data->pattern_map[false].num_occ = 0;
    data->pattern_map[false].no_prev_base = true;
    data->pattern_map[false].no_index_patt = true;
    data->pattern_map[true].base_delta_patt_num_elem = 0;
    data->pattern_map[true].index_patt_num_elem = 0;
    data->pattern_map[true].num_occ = 0;
    data->pattern_map[true].no_prev_base = true;
    data->pattern_map[true].no_index_patt = true;

    cout << "thread init " << dr_get_thread_id(drcontext) << endl;
}

/* This is called separately for each instruction in the block. */
static dr_emit_flags_t event_app_instruction(void *drcontext, void *tag, instrlist_t *bb,
                                            instr_t *instr, bool for_trace, bool translating,
                                            void *user_data) {
    drmgr_disable_auto_predication(drcontext, bb);
    if (drmgr_is_first_instr(drcontext, instr)) {
        instr_t *ins;

        /* Normally looking ahead should be performed in the analysis event, but
         * here that would require storing the counts into an array passed in
         * user_data.  We avoid that overhead by cheating drmgr's model a little
         * bit and looking forward.  An alternative approach would be to insert
         * each counter before its respective instruction and have an
         * instru2instru pass that pulls the increments together to reduce
         * overhead.
         */
        for (ins = instrlist_first_app(bb); ins != NULL; ins = instr_get_next_app(ins)) {
            if (instr_is_scatter(ins) || instr_is_gather(ins)) {
                int base_regno;
                opnd_t opnd;

                if (instr_is_scatter(ins)) {
                    for (int i = 0; i < instr_num_dsts(ins); i++) {
                        opnd = instr_get_dst(ins, i);
                        if (opnd_is_base_disp(opnd)) {
                            break;
                        }
                    }
                } else if (instr_is_gather(ins)) {
                    for (int i = 0; i < instr_num_srcs(ins); i++) {
                        opnd = instr_get_src(ins, i);
                        if (opnd_is_base_disp(opnd)) {
                            break;
                        }
                    }
                }
                base_regno = opnd_get_base(opnd);

                dr_insert_clean_call(drcontext, bb, ins, (void *)read_instr_reg_state, false, 2, OPND_CREATE_INTPTR(instr_get_app_pc(ins)), OPND_CREATE_INT32(base_regno));
            }
        }
    }
    return DR_EMIT_DEFAULT;
}

static void read_instr_reg_state(app_pc instr_addr, int base_regno) {
    void *drcontext = dr_get_current_drcontext();
    dr_mcontext_t mcontext;
    mcontext.size = sizeof(mcontext);
    mcontext.flags = DR_MC_ALL;
    DR_ASSERT(DR_MC_ALL & DR_MC_CONTROL);
    DR_ASSERT(DR_MC_ALL & DR_MC_INTEGER);
    DR_ASSERT(DR_MC_ALL & DR_MC_MULTIMEDIA);
    dr_get_mcontext(drcontext, &mcontext);

    instr_t instr;
    instr_init(drcontext, &instr);
	instr_reset(drcontext, &instr);
    if ((decode(drcontext, instr_addr, &instr)) == NULL) {
        dr_fprintf(STDERR, "ERROR UNABLE TO DECODE INSTRUCTION\n");
        instr_free(drcontext, &instr);
        return;
    }
    DR_ASSERT(instr_operands_valid(&instr));

    app_pc base = (app_pc) reg_get_value(base_regno, &mcontext);
    ptrdiff_t base_diff = base - ((app_pc) 0);
    app_pc idx;
    array<ptrdiff_t, MAX_INDEX_PATTERN> index_queue;
    uint index_queue_num_elem = 0;
    bool is_write;


#ifdef PRINT_ALL_OCC
    cout << "opcode: " << decode_opcode_name(instr_get_opcode(&instr)) << endl;
    cout << "base: " << (unsigned long long) base << endl;
    cout << "indicies: ";
#endif

    // get addresses accessed by gather/scatter
    while (instr_compute_address_ex(&instr, &mcontext, index_queue_num_elem, &idx, &is_write)) {
        index_queue.at(index_queue_num_elem) = idx - base;
        index_queue_num_elem += 1;


#ifdef PRINT_ALL_OCC
        cout << (unsigned long long) (idx - base) << "\t";
#endif
    }
#ifdef PRINT_ALL_OCC
    cout << endl << endl;
#endif

    if (index_queue_num_elem == 0) {
        instr_free(drcontext, &instr);
        return;
    }

    // zero indicies and add to base
    // assumes indicies are non negative
    ptrdiff_t min_index = index_queue.at(0);
    for (int i = 1; i < index_queue_num_elem; i++) {
        if (index_queue.at(i) < min_index) {
            min_index = index_queue.at(i);
        }
    }
    if (min_index != 0) {
        for (int i = 0; i < index_queue_num_elem; i++) {
            index_queue.at(i) -= min_index;
        }
        base_diff += min_index;
    }

    PatternStruct patt = ((per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx))->pattern_map[is_write];

    bool index_match = (patt.index_patt_num_elem == index_queue_num_elem);
    if (index_match) {
        for (int i = 0; i < index_queue_num_elem; i++) {
            if (index_queue.at(i) != patt.index_patt.at(i)) {
                index_match = false;
                break;
            }
        }
    }

    if (patt.no_index_patt) {
        DR_ASSERT(patt.no_prev_base);
        for (int i = 0; i < index_queue_num_elem; i++) {
            patt.index_patt.at(i) = index_queue.at(i);
            patt.index_patt_num_elem += 1;
        }
        patt.no_index_patt = false;
        patt.no_prev_base = false;
    } else if (index_match) {
        patt.base_delta_queue.at(patt.base_delta_queue_num_elem) = base_diff - patt.prev_base;
        patt.base_delta_queue_num_elem += 1;

        if (patt.base_delta_patt_num_elem == 0) {
            if (patt.base_delta_queue_num_elem >= MAX_BASE_DELTA_PATTERN_QUEUE) {            
                if (!match_base_delta_pattern(patt, false)) {
                    for (int i = 0; i < patt.base_delta_queue_num_elem - 1; i++) {
                        patt.base_delta_queue.at(i) = patt.base_delta_queue.at(i+1);
                    }
                    patt.base_delta_queue_num_elem -= 1;
                }
            }
        } else { // base delta pattern exists
            if (patt.base_delta_queue_num_elem >= patt.base_delta_patt_num_elem) {
                bool base_delta_match = true;
                for (int i = 0; i < patt.base_delta_patt_num_elem; i++) {
                    if (patt.base_delta_queue.at(i) != patt.base_delta_patt.at(i)) {
                        base_delta_match = false;
                        break;
                    }
                }
                
                if (base_delta_match) {
                    for (int i = 0; i < patt.base_delta_queue_num_elem - patt.base_delta_patt_num_elem; i++) {
                        patt.base_delta_queue.at(i) = patt.base_delta_queue.at(i + patt.base_delta_patt_num_elem);
                    }
                    patt.base_delta_queue_num_elem -= patt.base_delta_patt_num_elem;
                    patt.num_occ += 1;
                } else { // base delta pattern mismatch
                    record_and_print_pattern_occurrence(patt, is_write);
                }
            }
        }
    } else { // index patt mismatch 
        record_and_print_remaining_pattern_occurrences(patt, is_write);
    }

    patt.prev_base = base_diff;

    ((per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx))->pattern_map[is_write] = patt;

    instr_free(drcontext, &instr);
}

static bool match_base_delta_pattern(PatternStruct &patt, bool use_existing_pattern) {
    if (!use_existing_pattern) {
        patt.base_delta_patt_num_elem = 0;
        for (int i = 0; i < patt.base_delta_patt.max_size(); i++) {
            patt.base_delta_patt.at(i) = 1337;
        }

        int max_patt_len = 1;
        int max_patt_len_per = -1;
        for (int patt_per = 1; patt_per <= MAX_BASE_DELTA_PATTERN_PERIOD; patt_per++) {
            int patt_idx = 0;
            int last_complete_patt_idx = -1;
            for (int i = 0; i < patt.base_delta_queue_num_elem; i++) {
                if (patt.base_delta_queue.at(patt_idx) != patt.base_delta_queue.at(i)) {
                    break;
                }
                patt_idx += 1;
                if (patt_idx >= patt_per) {
                    patt_idx = 0;
                    last_complete_patt_idx = i+1;
                }
            }
            if (last_complete_patt_idx - patt_per > max_patt_len) { //need to subtract pattern_per since it will always match itself
                max_patt_len = last_complete_patt_idx;
                max_patt_len_per = patt_per;
            }
        }

        if (max_patt_len_per > 0) {
            DR_ASSERT(patt.base_delta_patt_num_elem == 0);
            for (int i = 0; i < max_patt_len_per; i++) {
                patt.base_delta_patt.at(i) = patt.base_delta_queue.at(i);
            }
            patt.base_delta_patt_num_elem = max_patt_len_per;
        }
    }
    

    // consume as many patterns as possible
    if (patt.base_delta_patt_num_elem > 0) {
        int last_full_match_idx = -1;
        for (int i = 0; i < patt.base_delta_queue_num_elem; i++) {
            int base_delta_patt_idx = i % patt.base_delta_patt_num_elem; 
            if (patt.base_delta_queue.at(i) != patt.base_delta_patt.at(base_delta_patt_idx)) {
                break;
            }
            if (base_delta_patt_idx == patt.base_delta_patt_num_elem - 1) {
                last_full_match_idx = i+1;
                patt.num_occ += 1;
            }
        }

        if (last_full_match_idx != -1) {
            for (int i = 0; i < patt.base_delta_queue_num_elem - last_full_match_idx; i++) {
                patt.base_delta_queue.at(i) = patt.base_delta_queue.at(i + last_full_match_idx);
            }
            patt.base_delta_queue_num_elem = patt.base_delta_queue_num_elem - last_full_match_idx;
        }
        
        DR_ASSERT(use_existing_pattern || last_full_match_idx != -1);
        DR_ASSERT(use_existing_pattern || patt.num_occ > 0);
        return true;
    } else {
        // if no pattern was found, pop first delta
        return false;
    }

}

// hard reset
// will clear base pattern, index pattern and queue
static void record_and_print_remaining_pattern_occurrences(PatternStruct &patt, bool is_write) {
    // finish matching any remaining instructions of the current pattern
    match_base_delta_pattern(patt, true);
    record_and_print_pattern_occurrence(patt, is_write);

    while (patt.base_delta_queue_num_elem > 0) {
        if (!match_base_delta_pattern(patt, false)) {
            for (int i = 0; i < patt.base_delta_queue_num_elem - 1; i++) {
                patt.base_delta_queue.at(i) = patt.base_delta_queue.at(i+1);
            }
            patt.base_delta_queue_num_elem -= 1;
        } else {
            record_and_print_pattern_occurrence(patt, is_write);
        }
    }

    patt.index_patt_num_elem = 0;
    patt.index_patt.fill(1337);
    patt.no_index_patt = true;
}

// soft reset
// will keep queue
static void record_and_print_pattern_occurrence(PatternStruct &patt, bool is_write) {
    record_pattern_occurrence(patt, is_write);
    print_pattern(patt, is_write);

    patt.base_delta_patt_num_elem = 0;
    patt.base_delta_patt.fill(1337);
    patt.num_occ = 0;
    patt.no_prev_base = true;
}

// Records a given pattern within the global count map
// Also resets the index and base pattern.
// For correctness, this method should only be called at the end of the pattern
// Ie, there has been a base or index mismatch.
static void record_pattern_occurrence(PatternStruct &patt, bool is_write) {
    if (patt.base_delta_patt_num_elem > 0 && patt.num_occ >= MIN_NUM_OCC) {
        vector<ptrdiff_t> index_patt_vec;
        for (int i = 0; i < patt.index_patt_num_elem; i++) {
            index_patt_vec.push_back(patt.index_patt.at(i));
        }
        vector<ptrdiff_t> base_delta_patt_vec;
        for (int i = 0; i < patt.base_delta_patt_num_elem; i++) {
            base_delta_patt_vec.push_back(patt.base_delta_patt.at(i));
        }
        dr_mutex_lock(count_mutex);
        pattern_count[is_write][index_patt_vec][base_delta_patt_vec].push_back(patt.num_occ);
        dr_mutex_unlock(count_mutex);
    }
}

static void print_pattern(PatternStruct &patt, bool is_write) {
#ifdef PRINT_PATTERN
    if (patt.base_delta_patt_num_elem > 0 && patt.num_occ >= MIN_NUM_OCC) {
        dr_mutex_lock(print_pattern_mutex);
        cout << (is_write ? "scatter" : "gather") << endl;
        cout << "index pattern:" << endl;
        for (int i = 0; i < patt.index_patt_num_elem; i++) {
            cout << patt.index_patt.at(i) << "\t";
        }
        cout << endl;
        cout << "base deltas: " << endl;
        for (int i = 0; i < patt.base_delta_patt_num_elem; i++) {
            cout << patt.base_delta_patt.at(i)<< "\t";
        }
        cout << endl;
        cout << "num occurrences: " << patt.num_occ << endl;
        cout << endl << endl;

        dr_mutex_unlock(print_pattern_mutex); 
    }
#endif
}

static void event_thread_exit(void *drcontext) {
    per_thread_t *data = (per_thread_t *)drmgr_get_tls_field(drcontext, tls_idx);
    cout << "thread exit " << dr_get_thread_id(drcontext) << endl;
    
    record_and_print_remaining_pattern_occurrences(data->pattern_map[false], false);
    record_and_print_remaining_pattern_occurrences(data->pattern_map[true], true);

    /* clean up memory */
    dr_thread_free(drcontext, data, sizeof(per_thread_t));
}

static void event_exit(void) {
    set<unsigned long long> total_num_occ_set;
    unordered_map<unsigned long long, vector<string>> total_num_occ_to_output_string_vec_map;

    for (const auto& is_write_elem : pattern_count) {
        for (const auto& index_patt_elem : is_write_elem.second) {
            for (const auto& base_delta_patt_elem : index_patt_elem.second) {
                string output_string = "";

                output_string += (is_write_elem.first ? "scatter\n" : "gather\n");
                output_string += "index patt\n";
                for (int i = 0; i < index_patt_elem.first.size(); i++) {
                    output_string += to_string(((unsigned long long) index_patt_elem.first[i])) + "\t";
                }
                if (index_patt_elem.first.size() == 0) {
                    output_string += "EMPTY";
                }
                output_string += "\nbase deltas\n";
                for (int i = 0; i < base_delta_patt_elem.first.size(); i++) {
                    output_string += to_string(((unsigned long long) base_delta_patt_elem.first[i])) + "\t";
                }
                if (base_delta_patt_elem.first.size() == 0) {
                    output_string += "EMPTY";
                }
                output_string +=  "\noccurrences\n";

                unsigned long long total_num_occ = 0;

                for (int i = 0; i < base_delta_patt_elem.second.size(); i++) {
#ifdef PRINT_ALL_OCC_FINAL
                    output_string += to_string(base_delta_patt_elem.second[i]) + "\t";
#else
                    if (i < 100) {
                        output_string += to_string(base_delta_patt_elem.second[i]) + "\t";
                    }
#endif
                    total_num_occ += base_delta_patt_elem.second[i];
                }

#ifndef PRINT_ALL_OCC_FINAL
                if (base_delta_patt_elem.second.size() > 100) {
                    output_string += "\n[output truncated]\n";
                }
#endif
                output_string += "\ntotal number of occurrences:\n" + to_string(total_num_occ) + "\n\n\n";

                total_num_occ_set.insert(total_num_occ);
                total_num_occ_to_output_string_vec_map[total_num_occ].push_back(output_string);
            }
        }
    }

    vector<unsigned long long> total_num_occ_arr(total_num_occ_set.begin(), total_num_occ_set.end());
    sort(total_num_occ_arr.begin(), total_num_occ_arr.end(), greater<unsigned long long>());
    for (auto total_num_occ : total_num_occ_arr) {
        for (auto total_num_occ_output_string : total_num_occ_to_output_string_vec_map[total_num_occ]) {
            cout << total_num_occ_output_string;
        }
    }
    
    if (!drmgr_unregister_bb_insertion_event(event_app_instruction) ||
        !drmgr_unregister_thread_init_event(event_thread_init) ||
        !drmgr_unregister_thread_exit_event(event_thread_exit)) {
        DR_ASSERT(false);
    }
    dr_mutex_destroy(count_mutex);
#ifdef PRINT_PATTERN
    dr_mutex_destroy(print_pattern_mutex);
#endif
    drx_exit();
    drmgr_exit();
}
