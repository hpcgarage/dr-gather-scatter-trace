import re
import sys
from collections import defaultdict

with open(sys.argv[1]) as file:
    matches = re.findall(r"(\w+) (DST|SRC) (REG|BASE|DISP) ([\w\s]+): ([\w\d]+)\n", file.read())

    regdict = defaultdict(lambda: defaultdict(lambda: defaultdict(int)))
    for match in matches:
        assert len(match) == 5
        opcode, target, regtype, regname, val = match
        regdict[opcode][regname][val] += 1
    print(regdict)

