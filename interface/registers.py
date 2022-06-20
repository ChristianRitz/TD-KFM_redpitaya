def parse_registers(filename):
    import re
    pattern = r'^#define\s+([\w]+)\s+(\w+)'
    registers = {}
    with open(filename, 'r') as f:
        for line in f:
            match = re.match(pattern, line)
            if not match:
                continue
            name = match.group(1)
            num = match.group(2)
            registers[name] = num
    return registers


import os as _os
_header_filename = _os.path.realpath(
    _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                  '..', 'src', 'registers.h')
)

_registers = parse_registers(_header_filename)
globals().update(_registers)


if __name__ == '__main__':
    print(_registers)
