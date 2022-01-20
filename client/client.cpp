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

// TODO DOCUMENTATION
// reads the register states of gather/scatter instructions and returns a count
// of the number of times a value for a register appears for each gather/scatter type.

// TODO convert c strings to cpp strings
// TODO use cpp prints

// TODO UPDATE dr client main TO GS CLIENT description

#include "dr_api.h"
#include "drmgr.h"
#include "drx.h"
#include "client_tools.h"
#include "avx512ctx-shared.h"
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

struct BaseIndexTupleStruct {
    vector<byte> base;
    vector<byte> index;

    // used only for deltas
    bool base_is_negative;
    bool index_is_negative;
};

struct PatternStruct {
    // stores the first base + index value of the pattern
    bool needs_first_instr = true;
    BaseIndexTupleStruct init;

    // stores the previous base + index values
    // used to calculate the delta from a new base + index to the previous
    BaseIndexTupleStruct prev;

    // stores up to MAX_PATTERN_QUEUE deltas
    // if a pattern is not currently being matched, is used to find a pattern
    // if a pattern is currently being matched, is used to store the deltas to attempt to match
    deque<BaseIndexTupleStruct> delta_queue;

    // the pattern which is currently being matched
    // consists of up to MAX_PATTERN_PERIOD deltas
    vector<BaseIndexTupleStruct> patt;

    // the number of times the current pattern has been matched so far
    // if no pattern is currently being matched, this value is 0
    unsigned long num_occurrences = 0;
};

#define NUM_INDEX_REG_TYPES 3
enum IndexRegType {
    XMM = 0,
    YMM = 1,
    ZMM = 2,
};

// map from (dynamorio instruction opcode name string) to (vector of size NUM_INDEX_REG_TYPES containing PatternStruct)
static unordered_map <string, vector<PatternStruct>> pattern_map;

#define MIN_NUM_OCC 1 // minimum number of occurrences of the pattern for it to be printed
// Note that a num_occ of x means the instruction has appeared x+1 times


static void event_exit(void);
static dr_emit_flags_t event_app_instruction(void *drcontext, void *tag, instrlist_t *bb, instr_t *instr, bool for_trace, bool translating, void *user_data);

DR_EXPORT void dr_client_main(client_id_t id, int argc, const char *argv[]) {
    dr_set_client_name("DynamoRIO Sample Client 'opcodes'", "http://dynamorio.org/issues");
    if (!drmgr_init())
        DR_ASSERT(false);
    drx_init();

    /* Register events: */
    dr_register_exit_event(event_exit);
    if (!drmgr_register_bb_instrumentation_event(NULL, event_app_instruction, NULL))
        DR_ASSERT(false);

    /* Make it easy to tell from the log file which client executed. */
    dr_log(NULL, DR_LOG_ALL, 1, "Client 'opcodes' initializing\n");
    /* Also give notification to stderr. */
    if (dr_is_notify_on()) {
#    ifdef WINDOWS
        /* Ask for best-effort printing to cmd window.  Must be called at init. */
        dr_enable_console_printing();
#    endif
        dr_fprintf(STDOUT, "Client opcodes is running\n");
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

static void print_pattern(PatternStruct &ps, string opcode_name) {
    if (ps.num_occurrences > 0) {
        cout << endl << "opcode: " << opcode_name << endl
        << "inital base: " << byte_vector_str(ps.init.base) << endl
        << "inital index: " << byte_vector_str(ps.init.index) << endl
        << "deltas:" << endl;
        for (int i = 0; i < ps.patt.size(); i++) {
            cout << i << " base: " << byte_vector_str(ps.patt[i].base) << endl;
            cout << i << " index: " << byte_vector_str(ps.patt[i].index) << endl;
        }
        cout << "occurrences: " << ps.num_occurrences << endl << endl;
    }
}

static void event_exit(void) {
    for (auto const &opcode_entry : pattern_map) {
        const string &opcode_name = opcode_entry.first;
        for (int index_reg_type = 0; index_reg_type < NUM_INDEX_REG_TYPES; index_reg_type++) {
            print_pattern(pattern_map[opcode_name][index_reg_type], opcode_name);
            // deque<PatternStruct> &patternDeque = pattern_map[opcode_name][index_reg_type];
            // if (!patternDeque.empty()) {
            //     if (patternDeque.size() > 1) {
            //         DR_ASSERT(patternDeque.size() == 2);
            //         struct PatternStruct &prev_pattern = patternDeque[0];
            //         struct PatternStruct &curr_pattern = patternDeque[1];
            //         if (prev_pattern.index_val == curr_pattern.index_val
            //             && prev_pattern.base_delta == curr_pattern.base_delta
            //             && prev_pattern.delta_is_negative == curr_pattern.delta_is_negative
            //             && prev_pattern.num_occurrences == curr_pattern.num_occurrences) {
            //             prev_pattern.num_agg += 1;
            //             patternDeque.pop_back();
            //         }
            //     }

            //     PatternStruct &patt_struct = pattern_map[opcode_name][index_reg_type][0];
            //     if (patt_struct.num_occurrences >= MIN_NUM_OCC) {
            //         cout << "endopcode: " << opcode_name
            //             << " delta: " << (patt_struct.delta_is_negative ? "-" : "") << byte_vector_str(patt_struct.base_delta)
            //             << " pattern: " << byte_vector_str(patt_struct.index_val)
            //             << " length: " << patt_struct.num_occurrences
            //             << " agg: " << patt_struct.num_agg
            //             << endl;
            //     }
            // }
        }
    }
    
    if (!drmgr_unregister_bb_insertion_event(event_app_instruction))
        DR_ASSERT(false);
    drx_exit();
    drmgr_exit();
}

static int read_reg (int regno, byte (&reg_buf)[REG_BUF_SZ], int (&regno_buf)[MAX_NUM_REGS],
                    unordered_set <int> (&regno_set), int &num_regs_read, bool &has_opmask) {
    if (reg_is_opmask(regno)) {
        if (has_opmask) {
            if (regno_set.find(regno) == regno_set.end()) {
                dr_fprintf(STDERR, "SANITY CHECK: Multiple opmasks in use\n");
            }
        } else {
            has_opmask = true;
        }
    }
    if (regno_set.find(regno) == regno_set.end()) {
        regno_buf[num_regs_read] = regno;
        regno_set.insert(regno);
        num_regs_read++;
    }
}

static vector<byte> calc_delta(vector<byte> v1, vector<byte> v2, bool &is_negative) {
    // TODO make this subtraction algorithm not dumb and bad
    is_negative = false;
    int carryover = 0;
    vector<byte> delta;
    for (int j = v1.size()-1; j >= 0; j--) {
        int diff = static_cast<int>(v1[j]) - static_cast<int>(v2[j]);
        if (carryover > 0) {
            diff -= 1;
            carryover = 0;
        }
        if (diff < 0) {
            diff += 256; // 16^2
            carryover = 1;
        }
        if (diff < 0 || diff > 255)
            DR_ASSERT(false);

        delta.insert(delta.begin(), static_cast<byte>(diff));
    }

    if (carryover > 0) {
        is_negative = true;
        carryover = 0;
        delta.clear();
        for (int j = v1.size()-1; j >= 0; j--) {
            int diff = static_cast<int>(v2[j]) - static_cast<int>(v1[j]);
            if (carryover > 0) {
                diff -= 1;
                carryover = 0;
            }
            if (diff < 0) {
                diff += 256; // 16^2
                carryover = 1;
            }
            if (diff < 0 || diff > 255)
                DR_ASSERT(false);

            delta.insert(delta.begin(), static_cast<byte>(diff));
        }
    }
    return delta;
}

static BaseIndexTupleStruct calc_base_index_tuple_delta(BaseIndexTupleStruct tup1, BaseIndexTupleStruct tup2) {
    BaseIndexTupleStruct ret;
    
    bool base_is_neg = false;
    ret.base = calc_delta(tup2.base, tup1.base, base_is_neg);
    ret.base_is_negative = base_is_neg;
    
    bool index_is_neg = false;
    ret.index = calc_delta(tup2.index, tup1.index, index_is_neg);
    ret.index_is_negative = index_is_neg;
    
    return ret;
}

// attempts to match a patternstruct pattern as many times as possible within
// its delta_queue. returns number of matches
static bool match_patt(PatternStruct &ps) {
    if (ps.patt.size() == 0) {
        cout << "attempted to match size 0 patt" << endl;
        return false;
    }
    bool matched = false;
    while (ps.delta_queue.size() >= ps.patt.size()) {
        bool match = true;
        for (int i = 0; i < ps.patt.size(); i++) {
            if (ps.delta_queue[i].base != ps.patt[i].base || ps.delta_queue[i].index != ps.patt[i].index) {
                match = false;
                break;
            }
        }

        if (match) {
            for (int i = 0; i < ps.patt.size(); i++) {
                ps.delta_queue.pop_front();
            }
            ps.num_occurrences += 1;
            matched = true;
        } else {
            break;
        }
    }
    return matched;
}


static void read_instr_reg_state(app_pc instr_addr) {
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

    byte base_reg_buf[MAX_REG_SZ];
    byte index_reg_buf[MAX_REG_SZ];

    // // if buffer remains 0xabababababab... after register value then reading probably failed
    // // for simplicity, we allocate MAX_REG_SZ for each register, even if the register size is smaller
    // memset(base_reg_buf, 0xab, sizeof(base_reg_buf));
    // memset(index_reg_buf, 0xab, sizeof(index_reg_buf));

    opnd_t opnd;
    reg_id_t base_regno;
    reg_id_t index_regno;

    if (instr_is_scatter(&instr)) {
        for (int i = 0; i < instr_num_dsts(&instr); i++) {
            opnd = instr_get_dst(&instr, i);
            if (opnd_is_base_disp(opnd)) {
                base_regno = opnd_get_base(opnd);
                index_regno = opnd_get_index(opnd);
                break;
            }
        }
    } else if (instr_is_gather(&instr)) {
        for (int i = 0; i < instr_num_srcs(&instr); i++) {
            opnd = instr_get_src(&instr, i);
            if (opnd_is_base_disp(opnd)) {
                base_regno = opnd_get_base(opnd);
                index_regno = opnd_get_index(opnd);
                break;
            }
        }
    }

    if (!reg_get_value_ex(base_regno, &mcontext, (byte*) base_reg_buf)) {
        dr_fprintf(STDERR, "ERROR: problem reading %s value\n", reg_names[base_regno]);
        instr_free(drcontext, &instr);
        return;
    }
    opnd_size_t base_reg_sz = reg_get_size(base_regno);
    string base_reg_name(reg_names[base_regno]);
    vector<byte> base_reg_val;
    for (int j = base_reg_sz - 1; j >= 0; --j) {
        base_reg_val.push_back(base_reg_buf[j]);
    }

    if (!reg_get_value_ex(index_regno, &mcontext, (byte*) index_reg_buf)) {
        dr_fprintf(STDERR, "ERROR: problem reading %s value\n", reg_names[index_regno]);
        instr_free(drcontext, &instr);
        return;
    }
    opnd_size_t index_reg_sz = reg_get_size(index_regno);
    string index_reg_name(reg_names[index_regno]);
    vector<byte> index_reg_val;
    for (int j = index_reg_sz - 1; j >= 0; --j) {
        index_reg_val.push_back(index_reg_buf[j]);
    }

    string opcode_name(decode_opcode_name(instr_get_opcode(&instr)));

    IndexRegType index_reg_type;
    if (reg_is_strictly_xmm(index_regno)) {
        index_reg_type = XMM;
    } else if (reg_is_strictly_ymm(index_regno)) {
        index_reg_type = YMM;
    } else if (reg_is_strictly_zmm(index_regno)) {
        index_reg_type = ZMM;
    } else {
        dr_fprintf(STDERR, "ERROR: unsupported index register %s. Supported index registers are xmm, ymm, zmm\n", reg_names[index_regno]);
        instr_free(drcontext, &instr);
        return;
    }

    

    if (pattern_map[opcode_name].empty()) {
        pattern_map[opcode_name].resize(NUM_INDEX_REG_TYPES);
    }
    // deque <PatternStruct> &reg_patt_deque = pattern_map[opcode_name][index_reg_type];

    PatternStruct &ps = pattern_map[opcode_name][index_reg_type];

    BaseIndexTupleStruct curr_tup;
    curr_tup.base = base_reg_val;
    curr_tup.index = index_reg_val;

    // need first instruction to begin calculating deltas
    if (ps.needs_first_instr) {
        ps.needs_first_instr = false;
        ps.prev = curr_tup;
        ps.init = curr_tup;
        instr_free(drcontext, &instr);
        return;
    }

    
    ps.delta_queue.push_back(calc_base_index_tuple_delta(curr_tup, ps.prev));
    // cout << "push to dq, new size: " << ps.delta_queue.size() << endl;
    ps.prev = curr_tup;

    if (ps.patt.size() > 0 && ps.delta_queue.size() >= ps.patt.size()) {
        bool matched = match_patt(ps);
        if (!matched) {
            // cout << endl << "printing pattern occurrences..." << endl;
            print_pattern(ps, opcode_name);
            // cout << "mismatch! resetting..." << endl;
            // ps.needs_first_instr = true;
            ps.patt.clear();
            ps.num_occurrences = 0;
        }
    }

    // if no pattern has been found yet
    if (ps.num_occurrences == 0) {
        // attempt to find a pattern
        DR_ASSERT(ps.patt.size() == 0);
        if (ps.delta_queue.size() >= MAX_PATTERN_QUEUE) {
            // cout << "attempting to find pattern in" << endl;
            // for (int k = 0; k < ps.delta_queue.size(); k++) {
            //     cout << k << " base: " << byte_vector_str(ps.delta_queue[k].base) << endl;
            //     cout << k << " index: " << byte_vector_str(ps.delta_queue[k].index) << endl;
            // }


            int max_patt_len = 1;
            int max_patt_len_per = -1;
            for (int patt_per = 1; patt_per <= MAX_PATTERN_PERIOD; patt_per++) {
                int patt_idx = 0;
                int last_complete_patt_idx = -1;
                for (int i = 0; i < ps.delta_queue.size(); i++) {
                    // TODO negatives
                    if (ps.delta_queue[patt_idx].base != ps.delta_queue[i].base || ps.delta_queue[patt_idx].index != ps.delta_queue[i].index) {
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
                DR_ASSERT(ps.patt.size() == 0);
                for (int i = 0; i < max_patt_len_per; i++) {
                    ps.patt.push_back(ps.delta_queue[i]);
                }


                cout << "matched pattern!" << endl
                    << "found: " << endl;
                for (int k = 0; k < ps.patt.size(); k++) {
                    cout << k << " base: " << byte_vector_str(ps.patt[k].base) << endl;
                    cout << k << " index: " << byte_vector_str(ps.patt[k].index) << endl;
                }
                // cout << "found pattern in" << endl;
                // for (int k = 0; k < ps.delta_queue.size(); k++) {
                //     cout << k << " base: " << byte_vector_str(ps.delta_queue[k].base) << endl;
                //     cout << k << " index: " << byte_vector_str(ps.delta_queue[k].index) << endl;
                // }
            }
                // for (int i = 0; i < patt_per; i++) {
                //     if (ps.delta_queue[i].base != ps.delta_queue[i + patt_per].base || ps.delta_queue[i].index != ps.delta_queue[i + patt_per].index) {
                //         break;
                //     }
                //     if (i == patt_per - 1) {
                //         for (int j = 0; j < patt_per; j++) {
                //             ps.patt.push_back(ps.delta_queue[j]);
                //         }


                //         cout << "matched pattern!" << endl
                //             << "found: " << endl;
                //         for (int k = 0; k < ps.patt.size(); k++) {
                //             cout << k << " base: " << byte_vector_str(ps.patt[k].base) << endl;
                //             cout << k << " index: " << byte_vector_str(ps.patt[k].index) << endl;
                //         }
                //     }
                // }
                // if (ps.patt.size() > 0) { // pattern has been found
                //     break;
                // }
            

            // consume as many patterns as possible
            if (ps.patt.size() > 0) {
                bool matched = match_patt(ps);
                DR_ASSERT(matched);
            } else {
                // if no pattern was found, pop first delta
                ps.delta_queue.pop_front();
            }
        }

        

        instr_free(drcontext, &instr);
        return;

    } // ps.num_occurrences == 0





    // if (reg_patt_deque.empty()) { // if this is the first time a reg is encountered
    //     struct PatternStruct ps;
    //     ps.index_val = index_reg_val;
    //     ps.base_prev_val = base_reg_val;
    //     ps.num_occurrences = 0;
    //     ps.num_agg = 0;
    //     pattern_map[opcode_name][index_reg_type].push_back(ps);
    // } else {
    //     struct PatternStruct &curr_pattern = pattern_map[opcode_name][index_reg_type].back();
    //     bool pattern_end = false;

    //     if (curr_pattern.index_val == index_reg_val) { // continue pattern
    //         // Since it's the same register, all register values should be the same size.
    //         // If check fails, may be due to multithreading, which is not supported yet.
    //         if (curr_pattern.base_prev_val.size() != base_reg_val.size()) {
    //             // cout << "prev val " << prev_val.size() << " next val size " << next_val.size() << endl;
    //             DR_ASSERT(false);
    //         }

    //         bool is_negative = false;
    //         vector<byte> delta = calc_delta(curr_pattern.base_prev_val, base_reg_val, is_negative);

    //         if (curr_pattern.num_occurrences == 0 || (curr_pattern.base_delta == delta && curr_pattern.delta_is_negative == is_negative)) {
    //             curr_pattern.base_prev_val = base_reg_val;
    //             curr_pattern.base_delta = delta;
    //             curr_pattern.delta_is_negative = is_negative;
    //             curr_pattern.num_occurrences += 1;
    //         } else {
    //             deque <PatternStruct> &patternDeque = pattern_map[opcode_name][index_reg_type];
    //             if (patternDeque.size() > 1) {
    //                 // DR_ASSERT(patternDeque.size() - 2 == 0);
    //                 struct PatternStruct &prev_pattern = patternDeque[0];
    //                 if (prev_pattern.index_val == curr_pattern.index_val
    //                     && prev_pattern.base_delta == curr_pattern.base_delta
    //                     && prev_pattern.delta_is_negative == curr_pattern.delta_is_negative
    //                     && prev_pattern.num_occurrences == curr_pattern.num_occurrences) {
    //                     prev_pattern.num_agg += 1;
    //                     patternDeque.pop_back();
    //                 } else {
    //                     pattern_end = true;
    //                 }
    //             }
                
    //             struct PatternStruct ps;
    //             ps.index_val = index_reg_val;
    //             ps.base_prev_val = base_reg_val;
    //             ps.num_occurrences = 0;
    //             ps.num_agg = 0;
    //             pattern_map[opcode_name][index_reg_type].push_back(ps);
                
    //         }
    //     } else { // end of pattern
    //         deque <PatternStruct> &patternDeque = pattern_map[opcode_name][index_reg_type];
    //         if (patternDeque.size() > 1) {
    //             struct PatternStruct &prev_pattern = patternDeque[0];
    //             struct PatternStruct &curr_pattern = patternDeque[1];
    //             if (prev_pattern.index_val == curr_pattern.index_val
    //                 && prev_pattern.base_delta == curr_pattern.base_delta
    //                 && prev_pattern.delta_is_negative == curr_pattern.delta_is_negative
    //                 && prev_pattern.num_occurrences == curr_pattern.num_occurrences) {
    //                 prev_pattern.num_agg += 1;
    //                 patternDeque.pop_back();
    //             }
    //         }
    //         pattern_end = true;

    //         struct PatternStruct ps;
    //         ps.index_val = index_reg_val;
    //         ps.base_prev_val = base_reg_val;
    //         ps.num_occurrences = 0;
    //         ps.num_agg = 0;
    //         pattern_map[opcode_name][index_reg_type].push_back(ps);
    //     }

    //     if (pattern_end) {
    //         PatternStruct &patt_struct = pattern_map[opcode_name][index_reg_type][0];
    //         if (patt_struct.num_occurrences >= MIN_NUM_OCC) { // nontrivial pattern
    //             cout << "opcode: " << opcode_name
    //                 << " delta: " << (patt_struct.delta_is_negative ? "-" : "") << byte_vector_str(patt_struct.base_delta)
    //                 << " pattern: " << byte_vector_str(patt_struct.index_val)
    //                 << " length: " << patt_struct.num_occurrences
    //                 << " agg: " << patt_struct.num_agg
    //                 << endl;
    //         }
    //         pattern_map[opcode_name][index_reg_type].pop_front();
    //     }
    // }

    instr_free(drcontext, &instr);
}

/* This is called separately for each instruction in the block. */
static dr_emit_flags_t event_app_instruction(void *drcontext, void *tag, instrlist_t *bb,
                                            instr_t *instr, bool for_trace, bool translating,
                                            void *user_data) {
    drmgr_disable_auto_predication(drcontext, bb);
    if (drmgr_is_first_instr(drcontext, instr)) {
        instr_t *ins;
        // uint isa_idx = get_count_isa_idx(drcontext);

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
                dr_insert_clean_call(drcontext, bb, ins, (void *)read_instr_reg_state, false, 1, OPND_CREATE_INTPTR(instr_get_app_pc(ins)));
            }
        }
    }
    return DR_EMIT_DEFAULT;
}