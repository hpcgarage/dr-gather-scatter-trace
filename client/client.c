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
 * Reports the dynamic count of the total number of instructions executed
 * broken down by opcode.
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
                          "Top %d opcode execution counts in %s mode:\n", NUM_COUNT_SHOW,
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

// static void print_sg_instr_reg_val(uint opcode) {
//     dr_mcontext_t mc;
//     // reg_id_t regs = {
//     //     DR_REG_XMM0,
//     // }
//     // char regnames = {

//     // }
//     switch(opcode) {
//         case OP_vpscatterdd:
//         case OP_vscatterdpd:
//         case OP_vscatterdps:
//         case OP_vpscatterdq:
//         case OP_vpscatterqd:
//         case OP_vscatterqpd:
//         case OP_vscatterqps:
//         case OP_vpscatterqq:
//         case OP_vpgatherdd:
//         case OP_vgatherdpd:
//         case OP_vgatherdps:
//         case OP_vpgatherdq:
//         case OP_vpgatherqd:
//         case OP_vgatherqpd:
//         case OP_vgatherqps:
//         case OP_vpgatherqq:
//             dr_get_mcontext(dr_get_current_drcontext(), &mc);
//             reg_t xmm1 = reg_get_value(DR_REG_XMM1, &mc);
//             reg_t ymm1 = reg_get_value(DR_REG_YMM1, &mc);
//             reg_t zmm1 = reg_get_value(DR_REG_ZMM1, &mc);
//             if (xmm1 || ymm1 || zmm1)
//                 printf("xmm1 %lu, ymm1 %lu, zmm1 %lu\n", xmm1, ymm1, zmm1);
//             break;
//         default:
//             printf("ERROR! print sg instr reg val encountered non sg instr, opcode: %d\n", opcode);
//     }
// }

static void
print_zmm(byte zmm_buf[], byte zmm_ref[])
{
    int zmm_off = 0;
    for (int i = 0; i < NUM_SIMD_REGS; ++i, zmm_off += 64) {
        dr_fprintf(STDERR, "got zmm[%d]:\n", i);
        for (int b = zmm_off; b < zmm_off + 64; ++b)
            dr_fprintf(STDERR, "%x", zmm_buf[b]);
        dr_fprintf(STDERR, "\nref zmm[%d]:\n", i);
        for (int b = zmm_off; b < zmm_off + 64; ++b)
            dr_fprintf(STDERR, "%x", zmm_ref[b]);
        dr_fprintf(STDERR, "\n");
    }
}

static void
print_opmask(byte opmask_buf[], byte opmask_ref[])
{
    int opmask_off = 0;
    for (int i = 0; i < NUM_OPMASK_REGS; ++i, opmask_off += 8) {
        dr_fprintf(STDERR, "got k[%i]:\n", i);
        for (int b = opmask_off; b < opmask_off + 8; ++b)
            dr_fprintf(STDERR, "%x", opmask_buf[b]);
        dr_fprintf(STDERR, "\nref k[%d]:\n", i);
        for (int b = opmask_off; b < opmask_off + 8; ++b)
            dr_fprintf(STDERR, "%x", opmask_ref[b]);
        dr_fprintf(STDERR, "\n");
    }
}

static void
read_avx512_state()
{
    byte zmm_buf[NUM_SIMD_REGS * 64];
    byte zmm_ref[NUM_SIMD_REGS * 64];
    byte opmask_buf[NUM_OPMASK_REGS * 8];
    byte opmask_ref[NUM_OPMASK_REGS * 8];
    memset(zmm_buf, 0xde, sizeof(zmm_buf));
    memset(zmm_ref, 0xab, sizeof(zmm_ref));
    memset(opmask_buf, 0xde, sizeof(opmask_buf));
    memset(opmask_ref, 0, sizeof(opmask_ref));
    for (int i = 0; i < NUM_OPMASK_REGS; ++i) {
        /* See comment in applicaton part of the test: We limit the test
         * to 2 byte wide mask registers.
         */
        memset(&opmask_ref[i * 8], 0xabab, 2);
    }

    dr_fprintf(STDERR, "Reading application state\n");

    void *drcontext = dr_get_current_drcontext();
    dr_mcontext_t mcontext;
    mcontext.size = sizeof(mcontext);
    mcontext.flags = DR_MC_ALL;
    dr_get_mcontext(drcontext, &mcontext);

    bool get_reg_value_ok = true;
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM0, &mcontext, &zmm_buf[0 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM1, &mcontext, &zmm_buf[1 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM2, &mcontext, &zmm_buf[2 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM3, &mcontext, &zmm_buf[3 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM4, &mcontext, &zmm_buf[4 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM5, &mcontext, &zmm_buf[5 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM6, &mcontext, &zmm_buf[6 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM7, &mcontext, &zmm_buf[7 * 64]);
#ifdef X64
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM8, &mcontext, &zmm_buf[8 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM9, &mcontext, &zmm_buf[9 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM10, &mcontext, &zmm_buf[10 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM11, &mcontext, &zmm_buf[11 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM12, &mcontext, &zmm_buf[12 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM13, &mcontext, &zmm_buf[13 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM14, &mcontext, &zmm_buf[14 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM15, &mcontext, &zmm_buf[15 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM16, &mcontext, &zmm_buf[16 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM17, &mcontext, &zmm_buf[17 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM18, &mcontext, &zmm_buf[18 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM19, &mcontext, &zmm_buf[19 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM20, &mcontext, &zmm_buf[20 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM21, &mcontext, &zmm_buf[21 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM22, &mcontext, &zmm_buf[22 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM23, &mcontext, &zmm_buf[23 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM24, &mcontext, &zmm_buf[24 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM25, &mcontext, &zmm_buf[25 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM26, &mcontext, &zmm_buf[26 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM27, &mcontext, &zmm_buf[27 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM28, &mcontext, &zmm_buf[28 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM29, &mcontext, &zmm_buf[29 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM30, &mcontext, &zmm_buf[30 * 64]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_ZMM31, &mcontext, &zmm_buf[31 * 64]);
    if (!get_reg_value_ok)
        dr_fprintf(STDERR, "ERROR: problem reading zmm value\n");
#endif
    if (memcmp(zmm_buf, zmm_ref, sizeof(zmm_buf)) != 0) {
#if VERBOSE
        print_zmm(zmm_buf, zmm_ref);
#endif
        dr_fprintf(STDERR, "ERROR: wrong zmm value\n");
    }
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_K0, &mcontext, &opmask_buf[0 * 8]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_K1, &mcontext, &opmask_buf[1 * 8]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_K2, &mcontext, &opmask_buf[2 * 8]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_K3, &mcontext, &opmask_buf[3 * 8]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_K4, &mcontext, &opmask_buf[4 * 8]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_K5, &mcontext, &opmask_buf[5 * 8]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_K6, &mcontext, &opmask_buf[6 * 8]);
    get_reg_value_ok =
        get_reg_value_ok && reg_get_value_ex(DR_REG_K7, &mcontext, &opmask_buf[7 * 8]);
    if (!get_reg_value_ok)
        dr_fprintf(STDERR, "ERROR: problem reading mask value\n");
    if (memcmp(opmask_buf, opmask_ref, sizeof(opmask_buf)) != 0) {
#if VERBOSE
        print_opmask(opmask_buf, opmask_ref);
#endif
        dr_fprintf(STDERR, "ERROR: wrong mask value\n");
    }
}

static void
clobber_avx512_state()
{
    byte buf[64];
    memset(buf, 0, sizeof(buf));
    dr_fprintf(STDERR, "Clobbering all zmm registers\n");
    __asm__ __volatile__("vmovups %0, %%zmm0" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm1" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm2" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm3" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm4" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm5" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm6" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm7" : : "m"(buf));
#ifdef X64
    __asm__ __volatile__("vmovups %0, %%zmm8" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm9" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm10" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm11" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm12" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm13" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm14" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm15" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm16" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm17" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm18" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm19" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm20" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm21" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm22" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm23" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm24" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm25" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm26" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm27" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm28" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm29" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm30" : : "m"(buf));
    __asm__ __volatile__("vmovups %0, %%zmm31" : : "m"(buf));
#endif
    dr_fprintf(STDERR, "Clobbering all mask registers\n");
    __asm__ __volatile__("kmovw %0, %%k0" : : "m"(buf));
    __asm__ __volatile__("kmovw %0, %%k1" : : "m"(buf));
    __asm__ __volatile__("kmovw %0, %%k2" : : "m"(buf));
    __asm__ __volatile__("kmovw %0, %%k3" : : "m"(buf));
    __asm__ __volatile__("kmovw %0, %%k4" : : "m"(buf));
    __asm__ __volatile__("kmovw %0, %%k5" : : "m"(buf));
    __asm__ __volatile__("kmovw %0, %%k6" : : "m"(buf));
    __asm__ __volatile__("kmovw %0, %%k7" : : "m"(buf));
}

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
                instr_t *nxt = instr_get_next(ins);
                // uint opcode = instr_get_opcode(ins);
                // dr_insert_clean_call(drcontext, bb, nxt, (void *) print_sg_instr_reg_val, true, 1, OPND_CREATE_INT32(opcode));
                dr_insert_clean_call(drcontext, bb, nxt, (void *)read_avx512_state,
                                         false, 0);
                dr_insert_clean_call(drcontext, bb, nxt,
                                        (void *)clobber_avx512_state, false, 0);
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


