import subprocess
import pprint
#pp = pprint.PrettyPrinter(indent=4)

def run_command(command):
    p = subprocess.Popen(command, shell=True,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT)
    return iter(p.stdout.readline, b'')

def run_script(params):
	command = '3D.exe ' + params['mode'] + ' ob/' +  params['ob'] + '.txt qp/' + params['qp'] + '.txt ' + params['k']
	#print (command)
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
		elif "planeid:" in line:
			plane['planeid'] = line[9:]
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
	#pprint.pprint (response_data, f)
	f.close()
	return response_data

def vcm_run_script(params):
	command = '3D.exe ' + params['mode'] + ' ob/' +  params['ob'] + '.txt ' + params['dir'] + ' ' + params['x1'] + ' ' + params['y1'] + ' ' + params['z1'] + ' ' + params['x2'] + ' ' + params['y2'] + ' ' + params['z2'] + ' ' + params['text'];
	print (command)
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
		elif "totp:" in line:
			response_data['totp'] = int(line[6:])
		elif "curseg;;" == line:
			row = 0
			seg = {}
		elif "dim:" in line:
			seg['dim'] = line[5:]
		elif "segdist:" in line:
			seg['segdist'] = line[9:]
		elif ";;curseg" == line:
			response_data[curseg] = seg
			curseg += 1
		else:
			seg[row] = line
			row += 1
	f = open('vcm_output_json.txt', 'w')
	#pprint.pprint (response_data, f)
	f.close()
	return response_data
