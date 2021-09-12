mazefile = open('m6.txt','r')
number_indeces = [1, 3, 5, 7];
for line in mazefile:
    s_line = line.split("\"")
    outstr = ""
    if len(s_line) > 7:
        for i in number_indeces:
            outstr += str((int(s_line[i])-2)/16)
            outstr += ','
        print outstr[:-1]
