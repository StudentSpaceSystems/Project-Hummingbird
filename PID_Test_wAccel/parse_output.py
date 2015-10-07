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
    try:
        for text_block in timestep_blocks:
            accel_x.append(ax.search(text_block).group(1))
            accel_y.append(ay.search(text_block).group(1))
            accel_z.append(az.search(text_block).group(1))
            x_output.append(xo.search(text_block).group(1))
            speed_2.append(sp2.search(text_block).group(1))
            speed_4.append(sp4.search(text_block).group(1))
    except AttributeError:
        print("Detected EOF or other anomoly")
    return [accel_x, accel_y, accel_z, x_output, speed_2, speed_4]

def write_data_to_file(fname, oname):
    data_matrix = parse_data_to_list(fname)   ## in format of Ax, Ay, Az, Xo, S2, S4
    data_header = "Acceleration (x)","Acceleration (y)","Acceleration (z)","PID Output Signal (x)","Speed Controller 2","Speed Controller 4"
    with open(oname, 'w') as ofile:
        for data_column_number in range(len(data_matrix)):
            print(data_header[data_column_number],file=ofile)       ## print header name

            for data_element in data_matrix[data_column_number]:    ## iterate over individual data elements in particular column
                print(data_element, file=ofile)                     ## no newline required as print automatically writes newline

            print("----------------",file=ofile)
            print("----------------",file=ofile)                    ## print BOM-like seperator to seperate segments
            print("----------------",file=ofile)

def ascii_coordinate_from_duple(duple,letters):
    return '%s%s'%(letters[duple[1]],duple[0])

def write_data_to_ods_template(fname, oname='-1', tname="./spreadsheet_data_template.ods"):
    if oname == '-1':
        from time import strftime
        oname = './%s.ods'%(strftime('%Y%m%d-%H%M%S'))
    import ezodf        ## requires ezodf (pip install ezodf)
    from string import ascii_uppercase as letters   ## requires string to reformat duple to ascii
    data_matrix = parse_data_to_list(fname)   ## in format of Ax, Ay, Az, Xo, S2, S4
    data_header = [fname,"Acceleration (x)","Acceleration (y)","Acceleration (z)","PID Output Signal (x)","Speed Controller 2","Speed Controller 4"]
    derived_header = ["PID Output Signal (x) (1000)","Speed Controller 2 (1/10000)","Speed Controller 4 (1/10000)"]
    formula_list = ['of:=[.%s]*1000','of:=[.%s]/10000','of:=[.%s]/10000']
    data_range = range(1,len(data_matrix[0])+1)     ## Assumes a rectangular matrix of form n x m (returning m value iterator)
    header_range = range(len(data_header))
    derived_header_range = range(len(data_header),len(data_header)+len(derived_header))
    spreadsheet = ezodf.opendoc(tname)
    target_sheet = spreadsheet.sheets['Sheet1']
    for column_number in header_range:
        target_sheet[(0,column_number)].set_value(data_header[column_number])
    for column_number in derived_header_range:
        target_sheet[(0,column_number)].set_value(derived_header[column_number-len(data_header)])
    for row_number in data_range:
        for column_number in header_range[:-1]:
            try:
                target_sheet[(row_number,column_number+1)].set_value(float(data_matrix[column_number][row_number-1]))
            except IndexError:
                target_sheet.append_rows()
                target_sheet[(row_number,column_number+1)].set_value(float(data_matrix[column_number][row_number-1]))
        target_sheet[(row_number,0)].set_value(row_number)
        for temp_column_number,formula_string in enumerate(formula_list):
            target_sheet[(row_number,column_number+2+temp_column_number)].formula=formula_string%(ascii_coordinate_from_duple((row_number+1,column_number+temp_column_number-1),letters))
    spreadsheet.saveas(oname)
