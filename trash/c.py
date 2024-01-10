def generate_crc_table(poly):
    table = [0] * 256
    WIDTH = 8
    TOP_BIT = 1 << (WIDTH - 1)

    for dividend in range(256):
        remainder = dividend << (WIDTH - 8)

        for _ in range(8):
            if remainder & TOP_BIT:
                remainder = (remainder << 1) ^ poly
            else:
                remainder = (remainder << 1)

        table[dividend] = remainder & 0xFF

    return table

def print_crc_table(table):
    for i in range(len(table)):
        if i % 16 == 0:
            print()
        print(f'{table[i]:02X}', end=' ')

# Generate CRC table for polynomial = 7 (0x07)
crc_table = generate_crc_table(7)

# Print the CRC table
print("CRC Table for Polynomial 0x07:")
print_crc_table(crc_table)
