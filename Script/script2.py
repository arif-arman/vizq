import subprocess
import pprint
#pp = pprint.PrettyPrinter(indent=4)

def run_command(command):
    p = subprocess.Popen(command,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT)
    return iter(p.stdout.readline, b'')

command = '3D.exe vcm sample3.txt 1'
response_data = {}
curseg = 1
seg = {}
row = 0
for line in run_command(command):
	line = line.strip()
	line = line.decode('ascii')
	#print(line)
	if "ndisteseg:" in line:
		response_data['nseg'] = int(line[11:])
	elif "maxseg:" in line:
		response_data['maxseg'] = int(line[8:])
	elif "curseg;;" == line:
		row = 0
		seg = {}
	elif ";;curseg" == line:
		response_data[curseg] = seg
		curseg += 1
	else:
		seg[row] = line
		row += 1
			
f = open('vcm_output_json.txt', 'w')
pprint.pprint (response_data, f)
