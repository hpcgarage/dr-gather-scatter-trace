#!/usr/bin/gawk -f

# Requires Gawk 4 or higher

BEGIN { RS = "" ; FS = "\n\n" } {
        if ($1 ~ /^opcode: .+/) {
                match($1, /(([0-9]+ base: \w+\n[0-9]+ index: \w+\n)+)occurrences: ([0-9]+)/, res)
                if (res[1] in count) {
                        count[res[1]] += res[3]
                } else {
                        count[res[1]] = 0 + res[3]
                }
        }
}

END {
        PROCINFO["sorted_in"] = "@val_num_desc"
        for (i in count) {
                if (count[i] > 0) {
                        print "\n" i "num occ: " count[i] "\n"
                }
        }
}