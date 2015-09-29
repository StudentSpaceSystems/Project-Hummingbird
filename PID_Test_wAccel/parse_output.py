## Written by Kenneth Hau
## Requires format of:
##
## Ax:\tNUM\tAy:\tNUM\tAz:\tNUM\n
## XOutput:\tNUM\n
## Speed2:\tNUM\tSpeed4:\tNUM\n\n
## (easily modifiable)

## Regex verified by regex101.com/python

## dirname = "/home/kenneth/Projects/git/Project-Hummingbird/PID_Test_wAccel"
## filename = dirname + "/parsed_data_output_1.txt"

import re       ## regex used due to extensibility in event of future data parsing of similar form

def parse_data_to_list(fname):
    with open(fname, 'r') as file:
        input_text = ""
        for line in file:
            input_text += line
    timestep_blocks = input_text.split("\n\n")
    accel_x = []
    ax = re.compile(r"Ax:\t(-?.*?)\t")  ## NONE
    accel_y = []
    ay = re.compile(r"Ay:\t(-?.*?)\t")   ## NONE
    accel_z = []
    az = re.compile(r"Az:\t(-?.*)\n")  ## does not require ? due to placement in string
    x_output = []
    xo = re.compile(r"XOutput:\t(-?.*)\n")  ## NONE
    speed_2 = []
    sp2 = re.compile(r"Speed2:\t(\d*)\t")   ## no decimals appear so \d suffices
    speed_4 = []
    sp4 = re.compile(r"Speed4:\t(\d*)")     ## due to .strip() call, no final newline appears
    for text_block in timestep_blocks:
        accel_x.append(ax.search(text_block).group(1))
        accel_y.append(ay.search(text_block).group(1))
        accel_z.append(az.search(text_block).group(1))
        x_output.append(xo.search(text_block).group(1))
        speed_2.append(sp2.search(text_block).group(1))
        speed_4.append(sp4.search(text_block).group(1))
    return [accel_x, accel_y, accel_z, x_output, speed_2, speed_4]
