# ppm2c --- convert an ASCII PPM fle to a C array              2020-10-28
# Copyright (c) 2020 John Honniball. All rights reserved

ppm = open('P1030550_tiny.ppm', 'r')
cArray = open('P1030550_tiny.h', 'w')

ppmType = ppm.readline()
ppmComment = ppm.readline()
ppmDimensions = ppm.readline()
ppmMaxPixel = ppm.readline()

cArray.write('const uint16_t Image[64][64] = {\n')

for y in range(64):
    cArray.write('   {')

    for x in range(64):
        rStr = ppm.readline()
        gStr = ppm.readline()
        bStr = ppm.readline()
        
        r5 = int(rStr) // 8;
        g6 = int(gStr) // 4;
        b5 = int(bStr) // 8;
        
        rgb565 = (r5 << 11) | (g6 << 5) | b5;
        
        cArray.write("0x%04x" % rgb565)

        if x < 63:
            if ((x + 1) % 8) == 0:
                cArray.write(",\n    ")
            else:
                cArray.write(", ")

    if y < 63:
        cArray.write('},\n') 
    else:
        cArray.write('}\n')

cArray.write('};')

cArray.close()
ppm.close()
