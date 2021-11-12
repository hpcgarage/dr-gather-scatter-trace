import json
import sys
import re
import os

def main():
    if len(sys.argv) < 2:
        print("Please input the path to the output of the dynamorio client")
        return

    filepath = sys.argv[1]
    # filepath = "/home/vincent/dr-gather-scatter-trace/client/out.log"
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
            regs = json_data[opcode].keys()
            r_reg = None
            for reg in regs:
                if re.compile("r\d+").fullmatch(reg):
                    r_reg = reg
                    # print(f"R register: {r_reg}")
                    break

            num_occ = int(json_data[opcode][r_reg][0]["num_occ"])
            delta = int(json_data[opcode][r_reg][0]["delta"], 16)
            oldi = 0
            for i, patt in enumerate(json_data[opcode][r_reg]):
                if int(patt["num_occ"]) != num_occ or int(patt["delta"], 16) != delta:
                    print(f"delta {delta} length {num_occ} agg {i - oldi - 1}")
                    num_occ = patt["num_occ"]
                    delta = patt["delta"]
                    oldi = i
            print(f"delta {delta // 8} length {num_occ+1} agg {len(json_data[opcode][r_reg]) - oldi - 1}")
            

if __name__ == "__main__":
    main()