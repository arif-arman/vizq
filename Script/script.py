import subprocess
import pprint
#pp = pprint.PrettyPrinter(indent=4)

def run_command(command):
    p = subprocess.Popen(command,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT)
    return iter(p.stdout.readline, b'')

command = '3D.exe mvq sample5.txt querypoints.txt 5'
response_data = {}
qp = {}
vp = {}
ip = {}
inv_parts = {}
plane = {}
boundary = {}
qpCount = 0
vpCount = 0
ipCount = 0
point = 0
vis_plane = False
invisible_plane = False
for line in run_command(command):
	line = line.strip()
	line = line.decode('ascii')
	#print(line)
	if "position" in line:
		if qpCount == 0:
			qp['position'] = line[10:]
		if qpCount > 0:
			qp['visible_planes'] = vp
			response_data[qpCount-1] = qp;
			qp = {}
		qpCount+=1
		qp['position'] = line[10:]
	elif "visibility:" in line:
		value = line[12:]
		qp['visibility'] = value
		#print (value)
	elif "visible planes:" in line:
		value = line[16:]
		if int(value) > 0:
			vp = {}
			vpCount = 0
		#qp['visible_planes_count'] = value
		#print (value)
	elif "plane;;" == line:
		vis_plane = True
		plane = {}
		boundary = {}
		point = 0
	elif ";;plane" == line:
		vis_plane = False
		plane['invisible_parts'] = inv_parts
		vp[vpCount] = plane
		vpCount+=1
		#print(plane)
		point = 0
	elif "inv_plane;;" == line:
		#print(line)
		invisible_plane = True
		ip = {}
		boundary = {}
		point = 0
	elif ";;inv_plane" == line:
		#print(line)
		invisible_plane = False
		ip['boundary'] = boundary
		inv_parts[ipCount] = ip
		ipCount+=1
		point = 0
		#print (ip)
	elif "invisible parts:" in line:
		plane['boundary'] = boundary
		value = line[16:]
		inv_parts = {}
		ipCount = 0
		#vp['invisible_planes_count'] = value
	elif "planeid:" in line:
		plane['planeid'] = line[9:]
	else:
		#print (line, vis_plane, invisible_plane)
		if invisible_plane:
			boundary[point] = line
			#print (ip)
			point +=1
		elif vis_plane:
			boundary[point] = line
			#print (plane)
			point+=1
			
			
			
qp['visible_planes'] = vp
response_data[qpCount-1] = qp
f = open('output_json.txt', 'w')
pprint.pprint (response_data, f)
