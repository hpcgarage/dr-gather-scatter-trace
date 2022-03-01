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
#include <ios>
#include <sstream>
#include <iostream>
#include <vector>
#include <deque>
#include <iomanip>

using namespace std;

#define PRINT_PATTERN

#define MAX_REG_SZ 64       // Assumed maximum register size
#define MAX_NUM_REGS 16     // Assumed maximum number of registers a single instruction can use
#define REG_BUF_SZ MAX_REG_SZ * MAX_NUM_REGS

// copied from dynamorio core/ir/x86/encode.c
const char *const reg_names[] = {
    "<NULL>", "rax",   "rcx",   "rdx",   "rbx",   "rsp",   "rbp",   "rsi",       "rdi",
    "r8",     "r9",    "r10",   "r11",   "r12",   "r13",   "r14",   "r15",       "eax",
    "ecx",    "edx",   "ebx",   "esp",   "ebp",   "esi",   "edi",   "r8d",       "r9d",
    "r10d",   "r11d",  "r12d",  "r13d",  "r14d",  "r15d",  "ax",    "cx",        "dx",
    "bx",     "sp",    "bp",    "si",    "di",    "r8w",   "r9w",   "r10w",      "r11w",
    "r12w",   "r13w",  "r14w",  "r15w",  "al",    "cl",    "dl",    "bl",        "ah",
    "ch",     "dh",    "bh",    "r8l",   "r9l",   "r10l",  "r11l",  "r12l",      "r13l",
    "r14l",   "r15l",  "spl",   "bpl",   "sil",   "dil",   "mm0",   "mm1",       "mm2",
    "mm3",    "mm4",   "mm5",   "mm6",   "mm7",   "xmm0",  "xmm1",  "xmm2",      "xmm3",
    "xmm4",   "xmm5",  "xmm6",  "xmm7",  "xmm8",  "xmm9",  "xmm10", "xmm11",     "xmm12",
    "xmm13",  "xmm14", "xmm15", "xmm16", "xmm17", "xmm18", "xmm19", "xmm20",     "xmm21",
    "xmm22",  "xmm23", "xmm24", "xmm25", "xmm26", "xmm27", "xmm28", "xmm29",     "xmm30",
    "xmm31",  "",      "",      "",      "",      "",      "",      "",          "",
    "",       "",      "",      "",      "",      "",      "",      "",          "",
    "",       "",      "",      "",      "",      "",      "",      "",          "",
    "",       "",      "",      "",      "",      "",      "st0",   "st1",       "st2",
    "st3",    "st4",   "st5",   "st6",   "st7",   "es",    "cs",    "ss",        "ds",
    "fs",     "gs",    "dr0",   "dr1",   "dr2",   "dr3",   "dr4",   "dr5",       "dr6",
    "dr7",    "dr8",   "dr9",   "dr10",  "dr11",  "dr12",  "dr13",  "dr14",      "dr15",
    "cr0",    "cr1",   "cr2",   "cr3",   "cr4",   "cr5",   "cr6",   "cr7",       "cr8",
    "cr9",    "cr10",  "cr11",  "cr12",  "cr13",  "cr14",  "cr15",  "<invalid>", "ymm0",
    "ymm1",   "ymm2",  "ymm3",  "ymm4",  "ymm5",  "ymm6",  "ymm7",  "ymm8",      "ymm9",
    "ymm10",  "ymm11", "ymm12", "ymm13", "ymm14", "ymm15", "ymm16", "ymm17",     "ymm18",
    "ymm19",  "ymm20", "ymm21", "ymm22", "ymm23", "ymm24", "ymm25", "ymm26",     "ymm27",
    "ymm28",  "ymm29", "ymm30", "ymm31", "",      "",      "",      "",          "",
    "",       "",      "",      "",      "",      "",      "",      "",          "",
    "",       "",      "",      "",      "",      "",      "",      "",          "",
    "",       "",      "",      "",      "",      "",      "",      "",          "",
    "zmm0",   "zmm1",  "zmm2",  "zmm3",  "zmm4",  "zmm5",  "zmm6",  "zmm7",      "zmm8",
    "zmm9",   "zmm10", "zmm11", "zmm12", "zmm13", "zmm14", "zmm15", "zmm16",     "zmm17",
    "zmm18",  "zmm19", "zmm20", "zmm21", "zmm22", "zmm23", "zmm24", "zmm25",     "zmm26",
    "zmm27",  "zmm28", "zmm29", "zmm30", "zmm31", "",      "",      "",          "",
    "",       "",      "",      "",      "",      "",      "",      "",          "",
    "",       "",      "",      "",      "",      "",      "",      "",          "",
    "",       "",      "",      "",      "",      "",      "",      "",          "",
    "",       "k0",    "k1",    "k2",    "k3",    "k4",    "k5",    "k6",        "k7",
    "",       "",      "",      "",      "",      "",      "",      "",          "bnd0",
    "bnd1",   "bnd2",  "bnd3",
};

// Required: MAX_PATTERN_QUEUE > 2 * MAX_PATTERN_PERIOD
#define MAX_PATTERN_PERIOD 20
#define MAX_PATTERN_QUEUE 50

struct PatternStruct {
    // // stores the first base + index value of the pattern
    // bool needs_first_instr = true;
    // BaseIndexTupleStruct init;

    // note that using null as an indication of no previous base value
    // is not possible due to some gather/scatter instructions having
    // null as a valid base register value, which indicates that the
    // base register value is 0.
    bool no_prev_base = true;
    app_pc prev_base;

    // stores up to MAX_PATTERN_QUEUE deltas
    // if a pattern is not currently being matched, is used to find a pattern
    // if a pattern is currently being matched, is used to store the deltas to attempt to match
    // bool no_prev_base_delta = true;
    // app_pc init_base;
    deque<ptrdiff_t> base_delta_queue;

    // the pattern which is currently being matched
    // consists of up to MAX_PATTERN_PERIOD deltas
    deque<ptrdiff_t> base_delta_patt;

    // the number of times the current pattern has been matched so far
    // if no pattern is currently being matched, this value is 0
    unsigned long long num_occ = 0;

    deque<ptrdiff_t> index_patt;
    deque<ptrdiff_t> index_queue;
};

// map from is_write (false if gather, true if scatter) to current pattern
static unordered_map <bool, PatternStruct> pattern_map;

static unordered_map <string, unsigned long long> pattern_count;

static void event_exit(void);
static dr_emit_flags_t event_app_instruction(void *drcontext, void *tag, instrlist_t *bb, instr_t *instr, bool for_trace, bool translating, void *user_data);

DR_EXPORT void dr_client_main(client_id_t id, int argc, const char *argv[]) {
    if (!drmgr_init())
        DR_ASSERT(false);
    drx_init();

    /* Register events: */
    dr_register_exit_event(event_exit);
    if (!drmgr_register_bb_instrumentation_event(NULL, event_app_instruction, NULL))
        DR_ASSERT(false);

    dr_log(NULL, DR_LOG_ALL, 1, "Client 'Gather Scatter Pattern Identification' initializing\n");
    if (dr_is_notify_on()) {
        dr_fprintf(STDOUT, "Client Gather Scatter Pattern Identification is running\n");
    }
}

static string byte_vector_str(vector<byte> v) {
    ostringstream os;
    for (int i = 0; i < v.size(); ++i) {
        os << setfill('0') << setw(2);
        os << hex << static_cast<int>(v[i]);
    }
    return os.str();
}

static void print_pattern(PatternStruct &patt) {
#ifdef PRINT_PATTERN
    if (patt.index_patt.size() > 0 && patt.base_delta_patt.size() > 0) {
        cout << "index pattern:" << endl;
        for (int i = 0; i < patt.index_patt.size(); i++) {
            cout << patt.index_patt[i] << "\t";
        }
        cout << endl;
        cout << "base deltas: " << endl;
        for (int i = 0; i < patt.base_delta_patt.size(); i++) {
            cout << patt.base_delta_patt[i] << "\t";
        }
        cout << "\nnum occurrences: " << patt.num_occ << endl;
        cout << endl << endl;
    }
#endif
}

static bool match_base_delta_pattern(PatternStruct &patt) {
    int max_patt_len = 1;
    int max_patt_len_per = -1;
    for (int patt_per = 1; patt_per <= MAX_PATTERN_PERIOD; patt_per++) {
        int patt_idx = 0;
        int last_complete_patt_idx = -1;
        for (int i = 0; i < patt.base_delta_queue.size(); i++) {
            if (patt.base_delta_queue[patt_idx] != patt.base_delta_queue[i]) {
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
        DR_ASSERT(patt.base_delta_patt.size() == 0);
        for (int i = 0; i < max_patt_len_per; i++) {
            patt.base_delta_patt.push_back(patt.base_delta_queue[i]);
        }


        // cout << "matched pattern!" << endl
        //     << "found: " << endl;
        // for (int k = 0; k < ps.patt.size(); k++) {
        //     cout << k << " base: " << byte_vector_str(ps.patt[k].base) << endl;
        //     cout << k << " index: " << byte_vector_str(ps.patt[k].index) << endl;
        // }
        // cout << "found pattern in" << endl;
        // for (int k = 0; k < ps.delta_queue.size(); k++) {
        //     cout << k << " base: " << byte_vector_str(ps.delta_queue[k].base) << endl;
        //     cout << k << " index: " << byte_vector_str(ps.delta_queue[k].index) << endl;
        // }
    }

    // consume as many patterns as possible
    if (patt.base_delta_patt.size() > 0) {
        bool matched = false;
        while (patt.base_delta_queue.size() >= patt.base_delta_patt.size()) {
            bool match = true;
            for (int i = 0; i < patt.base_delta_patt.size(); i++) {
                if (patt.base_delta_queue[i] != patt.base_delta_patt[i]) {
                    match = false;
                    break;
                }
            }

            if (match) {
                for (int i = 0; i < patt.base_delta_patt.size(); i++) {
                    patt.base_delta_queue.pop_front();
                }
                patt.num_occ += 1;
                matched = true;
            } else {
                break;
            }
        }
        DR_ASSERT(max_patt_len_per == -1 || matched);
        return true;
    } else {
        // if no pattern was found, pop first delta
        // patt.base_delta_queue.pop_front();
        return false;
    }

}


static void event_exit(void) {
    match_base_delta_pattern(pattern_map[true]);
    print_pattern(pattern_map[true]);
    match_base_delta_pattern(pattern_map[false]);
    print_pattern(pattern_map[false]);
    
    if (!drmgr_unregister_bb_insertion_event(event_app_instruction))
        DR_ASSERT(false);
    drx_exit();
    drmgr_exit();
}

static void read_instr_reg_state(app_pc instr_addr, int base_regno) {
    void *drcontext = dr_get_current_drcontext();
    dr_mcontext_t mcontext;
    mcontext.size = sizeof(mcontext);
    mcontext.flags = DR_MC_ALL;
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
    app_pc idx;
    uint ordinal = 0;
    bool is_write;
    while (instr_compute_address_ex(&instr, &mcontext, ordinal, &idx, &is_write)) {
        PatternStruct &patt = pattern_map[is_write];

        if (patt.no_prev_base) {
            patt.prev_base = base;
        }

        // base mismatch
        if (base != patt.prev_base) {
            // cout << "1" << endl;
            
            // for (int i = 0; i < patt.index_patt.size(); i++) {
            //     cout << patt.index_patt[i] << "\t";
            // }
            // cout << endl;

            // for (int i = 0; i < patt.index_queue.size(); i++) {
            //     cout << patt.index_queue[i] << "\t";
            // }
            // cout << endl;

            // there is no index pattern
            if (patt.index_patt.size() == 0) {
                // cout << "2" << endl;
                patt.index_patt = patt.index_queue;
                patt.index_queue.clear();
                DR_ASSERT(patt.num_occ == 0);
                // patt.base_delta_queue.push_back(base - patt.prev_base);
            } else if (patt.index_queue == patt.index_patt) { // index queue matches index pattern
                // cout << "3" << endl;
                // there is no current base delta pattern
                if (patt.base_delta_patt.size() == 0) {
                    // cout << "5" << endl;
                    if (patt.base_delta_queue.size() < MAX_PATTERN_QUEUE) { // base delta queue is not full
                    // cout << "6" << endl;
                        // patt.base_delta_queue.push_back(base - patt.prev_base);
                    } else { // base delta queue is full
                    // cout << "7" << endl;
                        //pattern search algorithm

                        // cout << "attempting to find pattern in" << endl;
                        // for (int k = 0; k < ps.delta_queue.size(); k++) {
                        //     cout << k << " base: " << byte_vector_str(ps.delta_queue[k].base) << endl;
                        //     cout << k << " index: " << byte_vector_str(ps.delta_queue[k].index) << endl;
                        // }

                        if (!match_base_delta_pattern(patt)) {
                            patt.base_delta_queue.pop_front();
                        }
                    }
                    
                } else if (patt.base_delta_queue.size() < patt.base_delta_patt.size()) { // base delta queue is smaller than base delta pattern
                // cout << "8" << endl;
                    // patt.base_delta_queue.push_back(base - patt.prev_base);
                } else { // base delta queue is same size as base delta pattern
                // cout << "9" << endl;
                    DR_ASSERT(patt.base_delta_queue.size() == patt.base_delta_patt.size());

                    // for (int i = 0; i < patt.base_delta_queue.size(); i++) {
                    //     cout << patt.base_delta_queue[i] << "\t";
                    // }
                    // cout << endl;

                    // for (int i = 0; i < patt.base_delta_patt.size(); i++) {
                    //     cout << patt.base_delta_patt[i] << "\t";
                    // }
                    // cout << endl;

                    // base delta queue matches base delta pattern
                    if (patt.base_delta_queue == patt.base_delta_patt) {
                        // cout << "10" << endl;
                        patt.num_occ++;
                        patt.base_delta_queue.clear();
                    } else { // base delta queue mismatches base delta pattern
                    // cout << "11" << endl;
                        // print out pattern
                        print_pattern(patt);
                        // start new pattern
                        patt.num_occ = 0;
                        // do not clear base delta queue
                        patt.base_delta_queue.pop_front();
                    }
                }

                patt.index_queue.clear();
            } else { // index queue mismatches index pattern
            // cout << "12" << endl;
                // print out pattern, if one exists
                print_pattern(patt);
                // start new pattern
                patt.base_delta_patt.clear();
                patt.base_delta_queue.clear();
                patt.num_occ = 0;
                // patt.index_patt = patt.index_queue;
                patt.index_queue.clear();
                patt.index_patt.clear();
            }

            if (!patt.no_prev_base) {
                patt.base_delta_queue.push_back(base - patt.prev_base);
            }
        }

        if (patt.no_prev_base) {
            patt.no_prev_base = false;
        }

        patt.index_queue.push_back(idx - base);
        patt.prev_base = base;
        ordinal++;
    }

    instr_free(drcontext, &instr);
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