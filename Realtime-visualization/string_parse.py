import numpy


def parse_newline_data(data, size=1, dtype=int, sep='\t'):
    """
    parse_newline_data(data, size=1)
    size -- data of length n
    data -- ReturnContainer type
    Parses data of n elements separated by newline:
    name data[0] data[1] ... data[n]    #time 0
    name data[0] data[1] ... data[n]    #time 1
    ...
    name data[0] data[1] ... data[n]    #time m
    :param sep: string
    :param dtype: type
    :param size: int
    :param data: string
    :rtype: numpy.array
    """
    return numpy.fromstring(data, size, dtype, sep)