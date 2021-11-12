import json
import sys
import re
import os

def main():
    if len(sys.argv) < 2:
        print("Please input the path to the output of the dynamorio client")
        return

    filepath = sys.argv[1]

    print(f"Reading from file {filepath}")
    if not os.path.isfile(filepath):
        print(f"Count not find file, aborting")
        return

    with open(filepath, "r") as outfile:
        data = outfile.read()
        json_str = re.match("[\s\S]+BEGIN JSON DATA\n([\s\S]+)END JSON DATA\n[\s\S]+", data).group(1)
        if not json_str:
            print("Error: unable to get json data from outfile")
            return
        json_data = json.loads(json_str)

        for opcode in json_data:
            opcode_reg_dict = json_data[opcode]
            regs = opcode_reg_dict.keys()
            r_reg = None
            vector_regs = []
            is_scatter = False
            
            if "scatter" in opcode:
                is_scatter = True
            else:
                assert "gather" in opcode

            for reg in regs:
                if re.compile("r\d+").fullmatch(reg):
                    r_reg = reg
                if re.compile("[xyz]mm\d+").fullmatch(reg):
                    for patt in opcode_reg_dict[reg]:
                        if (patt["is_dst"] == "true") == is_scatter:
                            vector_regs.append(reg)
                            break

            # TODO deal with multiple differing vector regs over the run
            assert len(vector_regs) == 1

            curr_total_r_occ = 0
            curr_r_idx = 0
            curr_total_vec_occ = 0
            curr_vec_idx = 0

            curr_init_r_idx = curr_r_idx
            curr_r_patt = opcode_reg_dict[r_reg][curr_r_idx]
            curr_vec_patt = opcode_reg_dict[vector_regs[0]][curr_vec_idx]


            while curr_r_idx < len(opcode_reg_dict[r_reg]) and curr_vec_idx < len(opcode_reg_dict[vector_regs[0]]):
                new_r_patt = opcode_reg_dict[r_reg][curr_r_idx]
                new_vec_patt = opcode_reg_dict[vector_regs[0]][curr_vec_idx]

                if not (curr_r_patt["delta"] == new_r_patt["delta"] and curr_r_patt["num_occ"] == new_r_patt["num_occ"]):
                    # TODO zmm is 16, check for xmm, ymm
                    patt_list = [int(num_str, 16) for num_str in [curr_vec_patt['patt_val'][i:i+16] for i in range(0, len(curr_vec_patt['patt_val']), 16)]]
                    print(f"kernel {'Scatter' if is_scatter else 'Gather'} delta {abs(int(curr_r_patt['delta'], 16) // 8)} patt: {patt_list} length {int(curr_r_patt['num_occ']) + 1} agg {curr_r_idx - curr_init_r_idx}")
                    curr_init_r_idx = curr_r_idx
                    curr_r_patt = new_r_patt
                    curr_vec_patt = new_vec_patt

                curr_total_r_occ += int(curr_r_patt["num_occ"]) + 1
                curr_r_idx += 1
                if curr_total_r_occ >= curr_total_vec_occ + int(curr_vec_patt["num_occ"]) + 1:
                    curr_vec_idx += 1
                    curr_total_vec_occ += int(curr_vec_patt["num_occ"]) + 1

            patt_list = [int(num_str, 16) for num_str in [curr_vec_patt['patt_val'][i:i+16] for i in range(0, len(curr_vec_patt['patt_val']), 16)]]
            print(f"kernel {'Scatter' if is_scatter else 'Gather'} delta {abs(int(curr_r_patt['delta'], 16) // 8)} patt: {patt_list} length {int(curr_r_patt['num_occ']) + 1} agg {curr_r_idx - curr_init_r_idx}")
            

if __name__ == "__main__":
    main()