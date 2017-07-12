import sys

readfile = '/Users/bolstadm/Dropbox (HHMI)/Jovian/LeeLab/t-maze_out.txt'
writefile = '/Users/bolstadm/test/t-maze_driver.txt'

if len(sys.argv) > 2:
	readfile = sys.argv[1]
	writefile = sys.argv[2]

print('Using ' + readfile + ' for input')
print('Using ' + writefile + ' for output')
f = open(readfile, 'r')
lines = f.readlines()
f.close()
output = []
for l in lines:
    x = l.split(',')
    output.append( " ".join(x[6:10]) + '\n')

f = open(writefile, 'w')
f.writelines(output)
f.close()