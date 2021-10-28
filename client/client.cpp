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
#include <iomanip>

#define COUNT_DELTA
// #define PRINT_OCC

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

// key: dynamorio instruction opcode name
// val: map with key: dynamorio register name
//               val: map with key: value of register
//                             val: number of occurrences of that value
#ifdef COUNT_FREQ
static unordered_map <string, unordered_map <string, unordered_map <string, unsigned long long>>> count_map;
#endif


// key: dynamorio instruction opcode name
// val: map with key: dynamorio register name
//               val: vector of PattenStructs, where the last element in the
//                    vector is the most recent
#ifdef COUNT_DELTA
struct PatternStruct {
    vector<byte> delta_to_previous_pattern;
    vector<byte> init_val;
    vector<byte> prev_val;
    vector<byte> delta;
    bool delta_is_negative;
    unsigned long long num_occurrences = 0;
};
static unordered_map <string, unordered_map <string, vector <PatternStruct>>> pattern_map;
#endif

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
#ifdef SHOW_RESULTS
    /* Also give notification to stderr. */
    if (dr_is_notify_on()) {
#    ifdef WINDOWS
        /* Ask for best-effort printing to cmd window.  Must be called at init. */
        dr_enable_console_printing();
#    endif
        dr_fprintf(STDOUT, "Client opcodes is running\n");
    }
#endif
}

static string byte_vector_str(vector<byte> v) {
    ostringstream os;
    for (int i = 0; i < v.size(); ++i) {
        os << setfill('0') << setw(2);
        os << hex << static_cast<int>(v[i]);
    }
    return os.str();
}

static void event_exit(void) {
#ifdef SHOW_RESULTS
#ifdef COUNT_FREQ
    for (auto const &opcode_entry : count_map) {
        cout << opcode_entry.first << endl;
        for (auto const &reg_entry : opcode_entry.second) {
            cout << "    " << reg_entry.first << endl;
            for (auto const &regval_entry : reg_entry.second) {
                cout << "        " << regval_entry.first << ":\t " << regval_entry.second << endl;
            }
        }
    }
#endif //COUNT_FREQ

#ifdef COUNT_DELTA
    for (auto const &opcode_entry : pattern_map) {
        cout << opcode_entry.first << endl;
        for (auto const &reg_entry : opcode_entry.second) {
            cout << "    " << reg_entry.first << endl;
            for (auto const &regval_entry : reg_entry.second) {
                cout << "            " << byte_vector_str(regval_entry.delta_to_previous_pattern) << endl;
                cout << "            " << byte_vector_str(regval_entry.init_val) << endl;
                cout << "            " << (regval_entry.delta_is_negative ? "-" : "") << byte_vector_str(regval_entry.delta) << endl;
                cout << "            " << regval_entry.num_occurrences << endl;
                cout << endl;
            }
        }
    }
#endif //COUNT_DELTA
#endif /* SHOW_RESULTS */

    if (!drmgr_unregister_bb_insertion_event(event_app_instruction))
        DR_ASSERT(false);
    drx_exit();
    drmgr_exit();
}

static int read_reg (int regno, byte (&reg_buf)[REG_BUF_SZ], int (&regno_buf)[MAX_NUM_REGS],
                    unordered_set <int> (&regno_set), string (&reg_desc_buf)[MAX_NUM_REGS],
                    int &num_regs_read, bool &has_opmask) {
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
        reg_desc_buf[num_regs_read] = "DST REG";
        regno_set.insert(regno);
        num_regs_read++;
    }
}

static vector<byte> calc_delta(vector<byte> v1, vector<byte> v2, bool &is_negative) {
    // TODO check edianness
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
        return;
    }
    DR_ASSERT(instr_operands_valid(&instr));

    byte reg_buf[MAX_REG_SZ * MAX_NUM_REGS];
    int regno_buf[MAX_NUM_REGS];
    unordered_set <int> regno_set;
    string reg_desc_buf[MAX_NUM_REGS];
    int num_regs_read = 0;
    bool has_opmask = false;

    // if buffer remains 0xabababababab... after register value then reading probably failed
    // for simplicity, we allocate MAX_REG_SZ for each register, even if the register size is smaller
    memset(reg_buf, 0xab, sizeof(reg_buf));

    opnd_t opnd;
    for (int i = 0; i < instr_num_dsts(&instr); i++) {
        opnd = instr_get_dst(&instr, i);
        if (opnd_is_reg(opnd)) {
            read_reg(opnd_get_reg(opnd), reg_buf, regno_buf, regno_set, reg_desc_buf, num_regs_read, has_opmask);
        } else if (opnd_is_base_disp(opnd)) {
            read_reg(opnd_get_base(opnd), reg_buf, regno_buf, regno_set, reg_desc_buf, num_regs_read, has_opmask);
            read_reg(opnd_get_index(opnd), reg_buf, regno_buf, regno_set, reg_desc_buf, num_regs_read, has_opmask);
        } else {
            dr_fprintf(STDERR, "UNKNOWN DESTINATION PARAM TYPE, SKIPPING\n");
        }
    }

    for (int i = 0; i < instr_num_srcs(&instr); i++) {
        opnd = instr_get_src(&instr, i);
        if (opnd_is_reg(opnd)) {
            read_reg(opnd_get_reg(opnd), reg_buf, regno_buf, regno_set, reg_desc_buf, num_regs_read, has_opmask);
        } else if (opnd_is_base_disp(opnd)) {
            read_reg(opnd_get_base(opnd), reg_buf, regno_buf, regno_set, reg_desc_buf, num_regs_read, has_opmask);
            read_reg(opnd_get_index(opnd), reg_buf, regno_buf, regno_set, reg_desc_buf, num_regs_read, has_opmask);
        } else {
            dr_fprintf(STDERR, "UNKNOWN SOURCE PARAM TYPE, SKIPPING\n");
        }
    }

    for (int i = 0; i < num_regs_read; i++) {
        if (!reg_get_value_ex(regno_buf[i], &mcontext, (byte*) &reg_buf[i * MAX_REG_SZ])) {
            dr_fprintf(STDERR, "ERROR: problem reading %s value\n", reg_names[regno_buf[i]]);
            continue;
        }
    }

    string opcode_name(decode_opcode_name(instr_get_opcode(&instr)));

    for (int i = 0; i < num_regs_read; i++) {
        opnd_size_t reg_sz = reg_get_size(regno_buf[i]);
        string reg_name(reg_names[regno_buf[i]]);
        vector<byte> reg_val;

        for (int j = i * MAX_REG_SZ + reg_sz - 1; j >= i * MAX_REG_SZ; --j) {
            reg_val.push_back(reg_buf[j]);
        }


        #ifdef COUNT_FREQ
        // We can't use a vector as a key in unordered map, so convert to string
        string reg_val_str = byte_vector_str(reg_val);
        if (count_map[opcode_name][reg_name].find(reg_val_str) == count_map[opcode_name][reg_name].end()) {
            count_map[opcode_name][reg_name][reg_val_str] = 1;
        } else {
            count_map[opcode_name][reg_name][reg_val_str] += 1;
        }
        #endif //COUNT_FREQ


        #ifdef COUNT_DELTA
        if (pattern_map[opcode_name][reg_name].empty()) { // if this is the first time a reg is encountered
            struct PatternStruct ps;
            ps.delta_to_previous_pattern = vector<byte>(reg_val.size(), 0);
            ps.init_val = reg_val;
            ps.prev_val = reg_val;
            ps.num_occurrences = 0;
            pattern_map[opcode_name][reg_name].push_back(ps);
        } else {
            // calculate delta
            struct PatternStruct &curr_pattern = pattern_map[opcode_name][reg_name].back();

            // if curr_pattern.num_occurrences == 0 then previous entry was start of delta pattern
            vector<byte> &prev_val = curr_pattern.prev_val;
            vector<byte> &next_val = reg_val;

            // Since it's the same register, all register values should be the same size
            if (prev_val.size() != next_val.size())
                DR_ASSERT(false);

            bool is_negative = false;
            vector<byte> delta = calc_delta(prev_val, next_val, is_negative);

            if (curr_pattern.num_occurrences == 0 || (curr_pattern.delta == delta && curr_pattern.delta_is_negative == is_negative)) {
                curr_pattern.prev_val = reg_val;
                curr_pattern.delta = delta;
                curr_pattern.delta_is_negative = is_negative;
                curr_pattern.num_occurrences += 1;
            } else {
                struct PatternStruct ps;
                ps.delta_to_previous_pattern = delta;
                ps.init_val = reg_val;
                ps.prev_val = reg_val;
                ps.num_occurrences = 0;
                pattern_map[opcode_name][reg_name].push_back(ps);
            }
        }
        #endif //COUNT_DELTA

        #ifdef PRINT_OCC
        cout << opcode_name << ":\t\t" << reg_name << ":\t\t" << byte_vector_str(reg_val) << endl;
        #endif
    }

    #ifdef PRINT_OCC
    cout << endl;
    #endif
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