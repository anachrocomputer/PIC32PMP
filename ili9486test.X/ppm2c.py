# ppm2c --- convert an ASCII PPM fle to a C array              2020-10-28
# Copyright (c) 2020 John Honniball. All rights reserved

import sys
import os

if len(sys.argv) != 2:
    print("Usage: ppm2c <ppm_filename>")
    exit()

(imgName, extName) = os.path.splitext(os.path.basename(sys.argv[1]))
cName = imgName + '.h'

ppm = open(sys.argv[1], 'r')

ppmType = ppm.readline()

if ppmType.strip() != 'P3':
    print("PPM file must be in ASCII format (P3)")
    ppm.close()
    exit()

ppmComment = ppm.readline()
ppmDimensions = ppm.readline()
ppmMaxPixel = ppm.readline()

(wdStr, htStr) = ppmDimensions.split(' ')

wd = int(wdStr)
ht = int(htStr)

if (wd < 1) or (wd > 320):
    print("Image width (%d) is no good" % wd)
    ppm.close()
    exit()

if (ht < 1) or (ht > 480):
    print("Image height (%d) is no good" % ht)
    ppm.close()
    exit()

cArray = open(cName, 'w')
cArray.write('const uint16_t %s[%d][%d] = {\n' % (imgName, ht, wd))

for y in range(ht):
    cArray.write('   {')

    for x in range(wd):
        rStr = ppm.readline()
        gStr = ppm.readline()
        bStr = ppm.readline()
        
        r5 = int(rStr) // 8;
        g6 = int(gStr) // 4;
        b5 = int(bStr) // 8;
        
        rgb565 = (r5 << 11) | (g6 << 5) | b5;
        
        cArray.write("0x%04x" % rgb565)

        if x < (wd - 1):
            if ((x + 1) % 8) == 0:
                cArray.write(",\n    ")
            else:
                cArray.write(", ")

    if y < (ht - 1):
        cArray.write('},\n') 
    else:
        cArray.write('}\n')

cArray.write('};\n')

cArray.close()
ppm.close()

