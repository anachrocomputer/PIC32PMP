# ppm2c --- convert an ASCII PPM fle to a C array              2020-10-28
# Copyright (c) 2020 John Honniball. All rights reserved

ppmName = 'Sunflower.ppm'
cName = 'Sunflower.h'

ppm = open(ppmName, 'r')
cArray = open(cName, 'w')

ppmType = ppm.readline()
ppmComment = ppm.readline()
ppmDimensions = ppm.readline()
ppmMaxPixel = ppm.readline()

wd = 320
ht = 240

cArray.write('const uint16_t %s[%d][%d] = {\n' % ('Sunflower', ht, wd))

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

