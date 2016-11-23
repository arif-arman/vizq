from django.shortcuts import render, redirect
from django.http import JsonResponse, HttpResponse
from vq3d_app import script
from .forms import UploadFileForm
from django.contrib import messages
import os

def save_uploaded_file(f, title, dataset):
    with open(dataset +'/'+ title, 'wb+') as destination:
        for chunk in f.chunks():
            destination.write(chunk)

def read_file(request):
	if request.method == 'GET':
		path = request.GET['path'] + ".txt"
		if request.GET['type'] == "ob":
			path = "ob/" + path
			f = open(path, "r")
			lines = f.read().split('\n')
			obj = {}
			counter = 0
			for line in lines:
				if counter > 0:
					obj[counter] = line 
				else:
					obj['size'] = line
				counter+=1
				
			#print(lines)
			f.close()
		elif request.GET['type'] == "qp":
			path = "qp/" + path
			f = open(path, "r")
			lines = f.read().split('\n')
			obj = {}
			counter = 0
			for line in lines:
				obj[counter] = line 
				counter+=1
			f.close()
		
		return JsonResponse(obj)
	return JsonResponse({'foo': 'bar'})

def read_dir(path):
	db = []
	files = os.listdir(path)
	for file in files:
		if ".txt" in file:
			db.append(file[:-4])
	return db;

def vcm(request):
	if request.method == 'GET':
		if request.GET['mode'] == "vcm" or request.GET['mode'] == "tvcm":
			params = {}
			params['mode'] = request.GET['mode']
			params['ob'] = request.GET['ob']
			params['dir'] = request.GET['dir']
			params['x1'] = request.GET['x1']
			params['y1'] = request.GET['y1']
			params['z1'] = request.GET['z1']
			params['x2'] = request.GET['x2']
			params['y2'] = request.GET['y2']
			params['z2'] = request.GET['z2']
			params['text'] = request.GET['text']
			return JsonResponse(script.vcm_run_script(params))
	return JsonResponse({'foo': 'bar'})

def mvq(request):
	if request.method == 'GET':
		if request.GET['mode'] == "mvq":
			params = {}
			params['mode'] = request.GET['mode']
			params['ob'] = request.GET['ob']
			params['qp'] = request.GET['qp']
			params['k'] = request.GET['k']
			return JsonResponse(script.run_script(params))
	return JsonResponse({'foo': 'bar'})

def upload_qp(request):
	if request.method == 'POST':
		form = UploadFileForm(request.POST, request.FILES)
		if form.is_valid():
			save_uploaded_file(request.FILES['file'], request.FILES['file'].name, "qp")
			#form = UploadFileForm()
			
			messages.add_message(request, messages.INFO, request.FILES['file'].name + ' upload successful.')
			return redirect('/')
			#return redirect(index, success='1')

def upload_ob(request):
	if request.method == 'POST':
		form = UploadFileForm(request.POST, request.FILES)
		if form.is_valid():
			save_uploaded_file(request.FILES['file'], request.FILES['file'].name, "ob")
			messages.add_message(request, messages.INFO, request.FILES['file'].name + ' upload successful.')
			return redirect('/')

def index(request):
	response_data = {}
	success = 0
	fontsize = []
	for f in range(8,41,2):
		fontsize.append(f)
	response_data['fontsize'] = fontsize
	if success == 1:
		response_data['success'] = 1
	form = UploadFileForm()
	response_data['form'] = form;
	response_data['qp'] = read_dir("qp/")
	response_data['ob'] = read_dir("ob/")
	return render(request, 'vq3d_app/index.html', response_data)
