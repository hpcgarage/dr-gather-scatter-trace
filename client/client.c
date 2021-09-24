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

/* Code Manipulation API Sample:
 * opcodes.c
 *
 * Reports the total number and register values of gather/scatter instructions executed.
 * Note that only top NUM_COUNT_SHOW instruction counts are displayed.
 */

#include "dr_api.h"
#include "drmgr.h"
#include "drx.h"
#include "client_tools.h"
#include "avx512ctx-shared.h"
#include <stdlib.h> /* qsort */
#include <string.h>

#ifdef WINDOWS
#    define DISPLAY_STRING(msg) dr_messagebox(msg)
#else
#    define DISPLAY_STRING(msg) dr_printf("%s\n", msg);
#endif

#define NULL_TERMINATE(buf) (buf)[(sizeof((buf)) / sizeof((buf)[0])) - 1] = '\0'

/* We keep a separate execution count per opcode.
 *
 * XXX: our counters are racy on ARM.  We use DRX_COUNTER_LOCK to make them atomic
 * (at a perf cost) on x86.
 *
 * XXX: we're using 32-bit counters.  64-bit counters are more challenging: they're
 * harder to make atomic on 32-bit x86, and drx does not yet support them on ARM.
 */
enum {
#ifdef X86
    ISA_X86_32,
    ISA_X86_64,
#elif defined(ARM)
    ISA_ARM_A32,
    ISA_ARM_THUMB,
#elif defined(AARCH64)
    ISA_ARM_A64,
#endif
    NUM_ISA_MODE,
};

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

static uint count[NUM_ISA_MODE][OP_LAST + 1];
#define NUM_COUNT sizeof(count[0]) / sizeof(count[0][0])
/* We only display the top 15 counts.  This sample could be extended to
 * write all the counts to a file.
 *
 * XXX: DynamoRIO uses a separate stack for better transparency. DynamoRIO stack
 * has limited size, so we should keep NUM_COUNT_SHOW small to avoid the message
 * buffer (char msg[NUM_COUNT_SHOW*80]) in event_exit() overflowing the stack.
 * It won't work on Windows either if the output is too large.
 */
#define NUM_COUNT_SHOW 15

static void
event_exit(void);
static dr_emit_flags_t
event_app_instruction(void *drcontext, void *tag, instrlist_t *bb, instr_t *instr,
                      bool for_trace, bool translating, void *user_data);

DR_EXPORT void
dr_client_main(client_id_t id, int argc, const char *argv[])
{
    dr_set_client_name("DynamoRIO Sample Client 'opcodes'",
                       "http://dynamorio.org/issues");
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
        dr_fprintf(STDERR, "Client opcodes is running\n");
    }
#endif
}

#ifdef SHOW_RESULTS
/* We use cur_isa to iterate each ISA counters in event_exit, so there will be
 * no race on accessing it in compare_counts.
 */
static uint cur_isa;
static int
compare_counts(const void *a_in, const void *b_in)
{
    const uint a = *(const uint *)a_in;
    const uint b = *(const uint *)b_in;
    if (count[cur_isa][a] > count[cur_isa][b])
        return 1;
    if (count[cur_isa][a] < count[cur_isa][b])
        return -1;
    return 0;
}

static const char *
get_isa_mode_name(uint isa_mode)
{
#    ifdef X86
    return (isa_mode == ISA_X86_32) ? "32-bit X86" : "64-bit AMD64";
#    elif defined(ARM)
    return (isa_mode == ISA_ARM_A32) ? "32-bit ARM" : "32-bit Thumb";
#    elif defined(AARCH64)
    return "64-bit AArch64";
#    else
    return "unknown";
#    endif
}
#endif

static void
event_exit(void)
{
#ifdef SHOW_RESULTS
    char msg[NUM_COUNT_SHOW * 80];
    int len, i;
    size_t sofar = 0;
    /* First, sort the counts */
    uint indices[NUM_COUNT];
    for (cur_isa = 0; cur_isa < NUM_ISA_MODE; cur_isa++) {
        sofar = 0;
        for (i = 0; i <= OP_LAST; i++)
            indices[i] = i;
        qsort(indices, NUM_COUNT, sizeof(indices[0]), compare_counts);

        if (count[cur_isa][indices[OP_LAST]] == 0)
            continue;
        len = dr_snprintf(msg, sizeof(msg) / sizeof(msg[0]),
                          "Gather/scatter execution counts in %s mode:\n",
                          get_isa_mode_name(cur_isa));
        DR_ASSERT(len > 0);
        sofar += len;
        for (i = OP_LAST - 1 - NUM_COUNT_SHOW; i <= OP_LAST; i++) {
            if (count[cur_isa][indices[i]] != 0) {
                len = dr_snprintf(msg + sofar, sizeof(msg) / sizeof(msg[0]) - sofar,
                                  "  %9lu : %-15s\n", count[cur_isa][indices[i]],
                                  decode_opcode_name(indices[i]));
                DR_ASSERT(len > 0);
                sofar += len;
            }
        }
        NULL_TERMINATE(msg);
        DISPLAY_STRING(msg);
    }
#endif /* SHOW_RESULTS */
    if (!drmgr_unregister_bb_insertion_event(event_app_instruction))
        DR_ASSERT(false);
    drx_exit();
    drmgr_exit();
}

static uint
get_count_isa_idx(void *drcontext)
{
    switch (dr_get_isa_mode(drcontext)) {
#ifdef X86
    case DR_ISA_X86: return ISA_X86_32;
    case DR_ISA_AMD64: return ISA_X86_64;
#elif defined(ARM)
    case DR_ISA_ARM_A32: return ISA_ARM_A32; break;
    case DR_ISA_ARM_THUMB: return ISA_ARM_THUMB;
#elif defined(AARCH64)
    case DR_ISA_ARM_A64: return ISA_ARM_A64;
#endif
    default: DR_ASSERT(false); /* NYI */
    }
    return 0;
}

static void print_reg_val(char *prefix, int regno, dr_mcontext_t *mcontext) {
    #define MAX_REG_SZ 64

    // if buffer remains 0xabababababab... after register value then reading probably failed
    // for simplicity, we allocate MAX_REG_SZ for each register, even if the register size is smaller
    byte reg_buf[MAX_REG_SZ];
    memset(reg_buf, 0xab, sizeof(reg_buf));

    if (!reg_get_value_ex(regno, mcontext, &reg_buf)) {
        dr_fprintf(STDERR, "ERROR: problem reading %s value\n", reg_names[regno]);
        return;
    }

    int reg_sz;
    if (reg_is_strictly_xmm(regno))                      reg_sz = 16;
    else if (reg_is_strictly_ymm(regno))                 reg_sz = 32;
    else if (reg_is_strictly_zmm(regno))                 reg_sz = 64;
    else if (reg_is_opmask(regno))                       reg_sz = 8;
    else if (regno >= DR_REG_R8 && regno <= DR_REG_R15)  reg_sz = 8;
    else {
        reg_sz = MAX_REG_SZ;
        dr_fprintf(STDERR, "WARNING: unknown reg size, may contain garbage.\n");
    }
    
    dr_fprintf(STDERR, "%s %s: ", prefix, reg_names[regno]);
    for (int k = 0; k < reg_sz; ++k) {
        dr_fprintf(STDERR, "%02x", reg_buf[k]);
    }
    dr_fprintf(STDERR, "\n");
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

    dr_fprintf(STDOUT, "\nInstruction %s Register State:\n", decode_opcode_name(instr_get_opcode(&instr)));
    opnd_t opnd;
    DR_ASSERT(instr_operands_valid(&instr));

    #define MAX_REG_SZ 64

    // if buffer remains 0xabababababab... after register value then reading probably failed
    // for simplicity, we allocate MAX_REG_SZ for each register, even if the register size is smaller
    byte reg_buf[MAX_REG_SZ];
    memset(reg_buf, 0xab, sizeof(reg_buf));

    for (int i = 0; i < instr_num_dsts(&instr); i++) {
        opnd = instr_get_dst(&instr, i);
        if (opnd_is_reg(opnd)) {
            print_reg_val("DST REG", opnd_get_reg(opnd), &mcontext);
        } else if (opnd_is_base_disp(opnd)) {
            print_reg_val("DST BASE", opnd_get_base(opnd), &mcontext);
            print_reg_val("DST DISP", opnd_get_index(opnd), &mcontext);
        } else {
            dr_fprintf(STDERR, "UNKNOWN DESTINATION PARAM TYPE, SKIPPING\n");
        }
    }
    for (int i = 0; i < instr_num_srcs(&instr); i++) {
        opnd = instr_get_src(&instr, i);
        if (opnd_is_reg(opnd)) {
            print_reg_val("SRC REG", opnd_get_reg(opnd), &mcontext);
        } else if (opnd_is_base_disp(opnd)) {
            print_reg_val("SRC BASE", opnd_get_base(opnd), &mcontext);
            print_reg_val("SRC DISP", opnd_get_index(opnd), &mcontext);
        } else {
            dr_fprintf(STDERR, "UNKNOWN SOURCE PARAM TYPE, SKIPPING\n");
        }
    }
}

// static void read_reg_state(int reg) {
//     void *drcontext = dr_get_current_drcontext();
//     dr_mcontext_t mcontext;
//     mcontext.size = sizeof(mcontext);
//     mcontext.flags = DR_MC_ALL;
//     dr_get_mcontext(drcontext, &mcontext);

//     #define MAX_REG_SZ 64

//     // if buffer remains 0xabababababab... after register value then reading probably failed
//     // for simplicity, we allocate MAX_REG_SZ for each register, even if the register size is smaller
//     byte reg_buf[MAX_REG_SZ];
//     memset(reg_buf, 0xab, sizeof(reg_buf));

//     bool get_reg_value_ok = true;
//     get_reg_value_ok = reg_get_value_ex(reg, &mcontext, &reg_buf);

//     if (!get_reg_value_ok) {
//         dr_fprintf(STDERR, "ERROR: problem reading %s value\n", reg_names[reg]);
//     } else {
//         int reg_off = 0;
//         int reg_sz = -1;
//         if (reg_is_strictly_xmm(reg))                   reg_sz = 16;
//         if (reg_is_strictly_ymm(reg))                   reg_sz = 32;
//         if (reg_is_strictly_zmm(reg))                   reg_sz = 64;
//         if (reg_is_opmask(reg))                         reg_sz = 8;
//         if (reg >= DR_REG_R8 && reg <= DR_REG_R15)  reg_sz = 8;
//         if (reg_sz == -1) {
//             reg_sz = MAX_REG_SZ;
//             dr_fprintf(STDERR, "WARNING: unknown reg size, may contain garbage.\n");
//         }

//         dr_fprintf(STDERR, "%s: ", reg_names[reg]);
//         for (int k = reg_off; k < reg_off + reg_sz; ++k) {
//             dr_fprintf(STDERR, "%02x", reg_buf[k]);
//         }
//         dr_fprintf(STDERR, "\n");
//     }
// }


// static void
// clobber_avx512_state()
// {
//     byte buf[64];
//     memset(buf, 0, sizeof(buf));
//     dr_fprintf(STDERR, "Clobbering all zmm registers\n");
//     __asm__ __volatile__("vmovups %0, %%zmm0" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm1" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm2" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm3" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm4" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm5" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm6" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm7" : : "m"(buf));
// #ifdef X64
//     __asm__ __volatile__("vmovups %0, %%zmm8" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm9" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm10" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm11" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm12" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm13" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm14" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm15" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm16" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm17" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm18" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm19" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm20" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm21" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm22" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm23" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm24" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm25" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm26" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm27" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm28" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm29" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm30" : : "m"(buf));
//     __asm__ __volatile__("vmovups %0, %%zmm31" : : "m"(buf));
// #endif
//     dr_fprintf(STDERR, "Clobbering all mask registers\n");
//     __asm__ __volatile__("kmovw %0, %%k0" : : "m"(buf));
//     __asm__ __volatile__("kmovw %0, %%k1" : : "m"(buf));
//     __asm__ __volatile__("kmovw %0, %%k2" : : "m"(buf));
//     __asm__ __volatile__("kmovw %0, %%k3" : : "m"(buf));
//     __asm__ __volatile__("kmovw %0, %%k4" : : "m"(buf));
//     __asm__ __volatile__("kmovw %0, %%k5" : : "m"(buf));
//     __asm__ __volatile__("kmovw %0, %%k6" : : "m"(buf));
//     __asm__ __volatile__("kmovw %0, %%k7" : : "m"(buf));
// }



#define MAX_NUM_REGS_READ 32

/* This is called separately for each instruction in the block. */
static dr_emit_flags_t
event_app_instruction(void *drcontext, void *tag, instrlist_t *bb, instr_t *instr,
                      bool for_trace, bool translating, void *user_data)
{
    drmgr_disable_auto_predication(drcontext, bb);
    if (drmgr_is_first_instr(drcontext, instr)) {
        instr_t *ins;
        uint isa_idx = get_count_isa_idx(drcontext);

        /* Normally looking ahead should be performed in the analysis event, but
         * here that would require storing the counts into an array passed in
         * user_data.  We avoid that overhead by cheating drmgr's model a little
         * bit and looking forward.  An alternative approach would be to insert
         * each counter before its respective instruction and have an
         * instru2instru pass that pulls the increments together to reduce
         * overhead.
         */
        for (ins = instrlist_first_app(bb); ins != NULL; ins = instr_get_next_app(ins)) {
            /* We insert all increments sequentially up front so that drx can
             * optimize the spills and restores.
             */
            if (instr_is_scatter(ins) || instr_is_gather(ins)) {
                dr_insert_clean_call(drcontext, bb, ins, (void *)read_instr_reg_state, false, 1, OPND_CREATE_INTPTR(instr_get_app_pc(ins)));
                // dr_insert_clean_call(drcontext, bb, ins, (void *)clobber_avx512_state, false, 0);
                drx_insert_counter_update(drcontext, bb, instr,
                    // We're using drmgr, so these slots here won't be used: drreg's slots will be.
                    SPILL_SLOT_MAX + 1,
                    IF_AARCHXX_(SPILL_SLOT_MAX + 1) & count[isa_idx][instr_get_opcode(ins)],
                    1,
                    // DRX_COUNTER_LOCK is not yet supported on ARM
                    IF_X86_ELSE(DRX_COUNTER_LOCK, 0));
            }
	    }
    }
    return DR_EMIT_DEFAULT;
}


