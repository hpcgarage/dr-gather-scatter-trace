import re
import sys
from collections import defaultdict

with open(sys.argv[1]) as file:
    matches = re.findall(r"(DST|SRC) (REG|BASE|DISP) ([\w\s]+): ([\w\d]+)\n", file.read())

    regdict = defaultdict(lambda: defaultdict(int))
    for match in matches:
        assert len(match) == 4
        target, regtype, regname, val = match
        regdict[regname][val] += 1
    print(regdict)

