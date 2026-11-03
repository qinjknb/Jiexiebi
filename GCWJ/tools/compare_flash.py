import sys

def parse_hex(path):
    """Parse Intel HEX into dict addr->byte."""
    data = {}
    with open(path, 'r') as f:
        base = 0
        for line in f:
            line = line.strip()
            if not line.startswith(':'):
                continue
            b = bytes.fromhex(line[1:])
            count, addr, rtype = b[0], (b[1] << 8) | b[2], b[3]
            payload = b[4:4+count]
            if rtype == 0x00:
                for i, byte in enumerate(payload):
                    data[base + addr + i] = byte
            elif rtype == 0x04:
                base = ((payload[0] << 8) | payload[1]) << 16
            elif rtype == 0x01:
                break
    return data

hex_data = parse_hex(r'D:\GCWJ\build\GCWJ.hex')
with open(r'D:\GCWJ\tools\flashdump.bin', 'rb') as f:
    flash = f.read()

print('hex size:', len(hex_data), 'flash dump size:', len(flash))

# compare every byte that the hex defines
FLASH_BASE = 0x08000000
mismatch = []
for addr, byte in hex_data.items():
    off = addr - FLASH_BASE
    if 0 <= off < len(flash):
        if flash[off] != byte:
            mismatch.append((addr, hex(byte), hex(flash[off])))
    else:
        mismatch.append((addr, hex(byte), '<out of dump>'))

print('total mismatches:', len(mismatch))
for m in mismatch[:40]:
    print('  0x%08X  hex=%s flash=%s' % m)

# show raw flash bytes around interesting addresses
def dump_at(addr, n=24):
    print(f'--- flash @0x{addr:08X} ---')
    print(' '.join('%02X' % flash[addr - FLASH_BASE + i] for i in range(n)))

dump_at(0x08000000, 16)
dump_at(0x08000180, 32)
dump_at(0x08000330, 48)
