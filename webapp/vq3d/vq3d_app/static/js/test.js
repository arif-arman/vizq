function getFileFromServer(url, doneCallBack) {
	var xhr = new XMLHttpRequest();
	xhr.onreadystatechange = handleStateChange;
	xhr.open("GET", url, true);
	xhr.send();

	function handleStateChange() {
		if (xhr.readyState === 4) {
			doneCallBack(xhr.status == 200 ? xhr.responseText : null);
		}
	}
}

function onload() {
	mode = document.getElementById('mode').value;
	$('#mvqViewResults').hide();
	modeChange();
}

function axisChange() {
	//console.log(dir);
	dir = $('#axis').val();
	//console.log(dir);
}

function modeChange() {

	mode = document.getElementById('mode').value;
	if (mode === "vcm") {
		$('#query_locations').slideUp();
		$('#tvcm_settings').slideUp();
		$('#vcm_settings').slideDown();
		///document.getElementById('query_locations').innerHTML = "";
		//document.getElementById('vcm_settings').innerHTML = '<object type="text/html" data="vq3d_app/vcm.html" ></object>';
	}
	else if (mode === "mvq") {

		$('#vcm_settings').slideUp();
		$('#tvcm_settings').slideUp();
		$('#query_locations').slideDown();
		
	}
	else if (mode === "tvcm") {

		$('#vcm_settings').slideDown();
		$('#query_locations').slideUp();
		$('#tvcm_settings').slideDown();
		
	}
}


function init() {

	raycaster = new THREE.Raycaster();
	mouse = new THREE.Vector2();

	
	//container = document.getElementById('gfx_window');
	container = document.createElement( 'div' );
	document.body.appendChild( container );
	/*
	var info = document.createElement( 'div' );
	info.style.position = 'absolute';
	info.style.top = '10px';
	info.style.width = '100%';
	info.style.textAlign = 'center';
	info.innerHTML = '<a href="http://threejs.org" target="_blank">three.js</a> - orthographic view';
	container.appendChild( info );
	*/
	//camera = new THREE.PerspectiveCamera( 40, window.innerWidth / window.innerHeight, 1, 1000 );
	camera = new THREE.PerspectiveCamera( 60, window.innerWidth / window.innerHeight, 1, 1000 );
	
	myCamera = new MyCamera();
	//camera.position.set(myCamera.camPosition.x, myCamera.camPosition.y, myCamera.camPosition.z);
	//camera.lookAt(myCamera.look);
	//camera.up = myCamera.upDir;
	renderer = new THREE.WebGLRenderer({antialias: true});
	renderer.setClearColor( 0xf0f0f0 );
	renderer.setPixelRatio( window.devicePixelRatio );
	renderer.setSize( window.innerWidth,  window.innerHeight);
	container.appendChild( renderer.domElement );
	controls = new THREE.OrbitControls( camera, renderer.domElement );
	//controls.addEventListener( 'change', render ); // add this only if there is no animation loop (requestAnimationFrame)
	camera.position.x = 0;
	camera.position.y = 100;
	camera.position.z = 0;

	controls.enableDamping = true;
	controls.dampingFactor = 0.25;
	controls.enableZoom = true;

	scene = new THREE.Scene();

	// Text
	/*
	textMaterial = new THREE.MultiMaterial( [
					new THREE.MeshPhongMaterial( { color: 0xffffff, shading: THREE.SmoothShading } ), // front
					new THREE.MeshPhongMaterial( { color: 0x0000ff, shading: THREE.SmoothShading } ) // side
				] );
	*/
	textMaterial = new THREE.MeshBasicMaterial({ color: 0xb0bca7, overdraw: true });
	textGroup = new THREE.Group();
	//textGroup.position.y = 100;
	scene.add(textGroup);
	

	// Grid

	var size = 1000, step = 10;
	/*
	var geometry = new THREE.Geometry();

	for ( var i = - size; i <= size; i += step ) {

		geometry.vertices.push( new THREE.Vector3( - size, i, 0 ) );
		geometry.vertices.push( new THREE.Vector3(   size, i, 0 ) );

		geometry.vertices.push( new THREE.Vector3( i, -size, 0 ) );
		geometry.vertices.push( new THREE.Vector3( i, size,   0 ) );

	}

	var material = new THREE.LineBasicMaterial( { color: 0x000000, opacity: 0.1 } );

	var line = new THREE.LineSegments( geometry, material );
	//	scene.add( line );
	*/
	// x axis
	var geometry = new THREE.Geometry();
	geometry.vertices.push(new THREE.Vector3(-size, 0, 0));
	geometry.vertices.push(new THREE.Vector3(size, 0, 0));
	var material = new THREE.LineBasicMaterial( { color: 0xFF0000, opacity: 1 } );
	var line = new THREE.LineSegments( geometry, material);
	scene.add(line);

	// y axis
	var geometry = new THREE.Geometry();
	geometry.vertices.push(new THREE.Vector3(0, -size, 0));
	geometry.vertices.push(new THREE.Vector3(0, size, 0));
	var material = new THREE.LineBasicMaterial( { color: 0x00FF00, opacity: 1 } );
	var line = new THREE.LineSegments( geometry, material);
	scene.add(line);

	// z axis
	var geometry = new THREE.Geometry();
	geometry.vertices.push(new THREE.Vector3(0, 0, -size));
	geometry.vertices.push(new THREE.Vector3(0, 0, size));
	var material = new THREE.LineBasicMaterial( { color: 0x0000FF, opacity: 1 } );
	var line = new THREE.LineSegments( geometry, material);
	scene.add(line);
	
	
	

	// Lights

	var ambientLight = new THREE.AmbientLight( Math.random() * 0x10 );
	scene.add( ambientLight );

	var directionalLight = new THREE.DirectionalLight( Math.random() * 0xffffff );
	directionalLight.position.x = Math.random() - 0.5;
	directionalLight.position.y = Math.random() - 0.5;
	directionalLight.position.z = Math.random() - 0.5;
	directionalLight.position.normalize();
	scene.add( directionalLight );

	var directionalLight = new THREE.DirectionalLight( Math.random() * 0xffffff );
	directionalLight.position.x = Math.random() - 0.5;
	directionalLight.position.y = Math.random() - 0.5;
	directionalLight.position.z = Math.random() - 0.5;
	directionalLight.position.normalize();
	scene.add( directionalLight );


	//renderer = new THREE.CanvasRenderer();
	
	camera2 = camera.clone();
	stats = new Stats();
	//container.appendChild( stats.dom );
	var textureLoader = new THREE.TextureLoader();
	mapA = textureLoader.load( "/static/textures/gmap_red.png");
	mapB = textureLoader.load( "/static/textures/gmap_green.png");
	materialA = new THREE.SpriteMaterial( { map: mapA, color: 0xffffff, fog: false } );
	materialB = new THREE.SpriteMaterial( { map: mapB, color: 0xffffff, fog: false } );
	
	//
	document.addEventListener( 'keydown', onDocumentKeyDown, false );
	window.addEventListener( 'resize', onWindowResize, false );
	document.addEventListener( 'mousedown', onDocumentMouseDown, false );
	//window.addEventListener('move', onKeyDown, false);

	
}


function loadFont() {
	var loader = new THREE.FontLoader();
	loader.load( '/static/fonts/' + fontName + '_' + fontWeight + '.typeface.json', function ( response ) {
		font = response;
		refreshText();
	} );
}


function onDocumentKeyDown(event) {
	//event.preventDefault();
	//console.log(event.keyCode);
	//console.log(mode);
	switch(event.keyCode) {
		case 49: myCamera.pitch(delta); break;	// 1
		case 50: myCamera.pitch(-delta); break;	// 2
		case 51: myCamera.yaw(delta); break;	// 3
		case 52: myCamera.yaw(-delta); break;	// 4
		case 53: myCamera.roll(delta); break;	// 5
		case 54: myCamera.roll(-delta); break;	// 6
		case 83: myCamera.slide(5, 1); break; //s down
		case 87: myCamera.slide(5, 2); break; //w up
		case 65: myCamera.slide(5, 3); break; //a left
		case 68: myCamera.slide(5, 4); break; //d right
		case 38: myCamera.slide(5, 5); break; //up arrow
		case 40: myCamera.slide(5, 6); break; //down arrow
		case 81: 
			if (mode === "mvq") {
				prevqp = qp; qp++; qp = qp % querypoints.length; qpDraw(qp); $('#mvqViewResults').show();
			}   
			break;	// q
		case 86: 
			if (mode !== "mvq") {
				curseg++; curseg = curseg % nseg; vcmDraw(curseg); 
			} 
			break;	// v
		case 66: 
			if (mode !== "mvq") { 
				curseg--; curseg = (curseg + nseg) % nseg; vcmDraw(curseg); 
			}  
			break;	// b
		case 79:
			toggleObstacles(); break;
	}
	var keyDownEvent = event || window.event,
	keycode = (keyDownEvent.which) ? keyDownEvent.which : keyDownEvent.keyCode;
    //return false;

}


function vcmDraw(seg) {
	//if (vcmGroup) scene.remove(vcmGroup);
	if (vcmMesh) scene.remove(vcmMesh);
	console.log("curseg: " + curseg);
	var dim = vcmArray[curseg]['dim'];
	var delta = dim/maxseg;
	var lx = -dim / 2.0;
	var ly = -dim / 2.0;
	var z = vcmArray[curseg]['segdist'];
	//console.log("dim: " + dim);
	//console.log("segdist: " + z);
		
	$('#logger').html('Segment: ' + curseg + '<br>Dimension: ' + dim + '<br>Distance: ' + z);

	var group = new THREE.Group();
	if (!(dir & 1)) z = -z;
	var p = "";
	var geometry = new THREE.BufferGeometry();
	var numCells = maxseg * maxseg;
	var positions = new Float32Array( numCells * 2 * 3 * 3 );
	var normals   = new Float32Array( numCells * 2 * 3 * 3 );
	var colors    = new Float32Array( numCells * 2 * 3 * 3 );
	var uvs       = new Float32Array( numCells * 2 * 3 * 2 );
	var color = new THREE.Color();
	var x1, y1, z1;
	for (var i=0;i<maxseg;i++) {
		var row = vcmArray[curseg][i].split(' ');
		//console.log(row.length);
		for (var j=0;j<row.length;j++) {
			var cell = parseInt(row[j]);
			p += cell + " ";
			cell = Math.min(cell, totp);
			var col = 255 - (255 * (cell * 1.0 / totp));
			col /= 255;
			color.setRGB(col,col,col);
			var index = maxseg * i * 18 + j * 18; // koto number box
			//console.log(index);
			//console.log(col);

			//var geometry = new THREE.Geometry();  
			if (dir <= 1) {
				/*
				geometry.vertices.push(
			    new THREE.Vector3(z, lx, ly),//vertex0
			    new THREE.Vector3(z, lx+delta, ly),
			    new THREE.Vector3(z, lx+delta, ly+delta),
			    new THREE.Vector3(z, lx, ly+delta)
			    );
			    */
			    //t1
			    //v0
			    positions[index] = z;
			    positions[index+1] = lx;
			    positions[index+2] = ly;
			    //v1
			    positions[index+3] = z;
			    positions[index+4] = lx+delta;
			    positions[index+5] = ly;
			    //v2
			    positions[index+6] = z;
			    positions[index+7] = lx+delta;
			    positions[index+8] = ly+delta;
			    //t2
			    //v0
			    positions[index+9] = z;
			    positions[index+10] = lx+delta;
			    positions[index+11] = ly+delta;
			    //v1
			    positions[index+12] = z;
			    positions[index+13] = lx;
			    positions[index+14] = ly+delta;
			    //v2
			    positions[index+15] = z;
			    positions[index+16] = lx;
			    positions[index+17] = ly;

			    for (var m=0;m<18;m++)
			    	colors[index+m] = col;
			    //uvs[index] = 0.5; // just something...
				//uvs[index + 1] = 0.5;

			}
			else if (dir == 2 || dir == 3) {
				geometry.vertices.push(
			    new THREE.Vector3(lx, z, ly),//vertex0
			    new THREE.Vector3(lx+delta, z, ly),
			    new THREE.Vector3(lx+delta, z, ly+delta),
			    new THREE.Vector3(lx, z, ly+delta)
			    );
			}
			else {
				geometry.vertices.push(
			    new THREE.Vector3(lx, ly, z),//vertex0
			    new THREE.Vector3(lx+delta, ly, z),
			    new THREE.Vector3(lx+delta, ly+delta, z),
			    new THREE.Vector3(lx, ly+delta, z)
			    );
			}

			
		    

			/*
		    geometry.faces.push(
		    new THREE.Face3(0,1,2),//use vertices of rank 2,1,0
		    new THREE.Face3(2,3,0)//vertices[3],1,2...
	    	);
			var material = new THREE.MeshBasicMaterial( {side: THREE.DoubleSide, alphaTest: 0.5} );
			material.color.setRGB(col, col, col);
			var plane = new THREE.Mesh(geometry, material);
			plane.translateX(target['ox']);
			plane.translateY(target['oy']);
			plane.translateZ(target['oz']);
			group.add(plane);
			*/
			lx += delta;
		}
		p += "\n";
		ly += delta;
		lx = -dim / 2.0;
	}
	geometry.addAttribute( 'position', new THREE.BufferAttribute( positions, 3 ) );
	geometry.addAttribute( 'normal', new THREE.BufferAttribute( normals, 3 ) );
	geometry.addAttribute( 'color', new THREE.BufferAttribute( colors, 3 ) );
	geometry.addAttribute( 'uv', new THREE.BufferAttribute( uvs, 2 ) );
	geometry.computeBoundingBox();
	geometry.computeBoundingSphere();
	geometry.computeVertexNormals(); // computed vertex normals are orthogonal to the face for non-indexed BufferGeometry
    // material
    var material = new THREE.MeshBasicMaterial( {side: THREE.DoubleSide, alphaTest: 0.5, vertexColors: THREE.VertexColors} );
    /*
    var material = new THREE.MeshPhongMaterial( {
        color: 0xffffff, 
        shading: THREE.FlatShading,
        vertexColors: THREE.VertexColors,
        side: THREE.DoubleSide
    } );
    */
    // mesh
    vcmMesh = new THREE.Mesh( geometry, material );
    vcmMesh.translateX(target['ox']);
    vcmMesh.translateY(target['oy']);
    vcmMesh.translateZ(target['oz']);
    //group.add(vcmMesh);
	scene.add(vcmMesh);
	/*
	var geometry = new THREE.Geometry();  
    geometry.vertices.push(
    new THREE.Vector3(target['ox'], target['oy'], target['oz']),//vertex0
    new THREE.Vector3(z, -dim/2.0, -dim/2.0),
    new THREE.Vector3(z, -dim/2.0, dim/2.0),
    new THREE.Vector3(z, dim/2.0, dim/2.0),
    new THREE.Vector3(z, dim/2.0, -dim/2.0)
    );


    geometry.faces.push(
    new THREE.Face3(0,1,2),
    new THREE.Face3(0,2,3),
    new THREE.Face3(0,3,4),
    new THREE.Face3(0,4,1)
   
	);
	var material = new THREE.MeshBasicMaterial( {color: 0x00ff00, side: THREE.DoubleSide, alphaTest: 0.5, transparent: true, opacity: 0.5});
	var plane = new THREE.Mesh(geometry, material);
	group.add(plane);
	scene.add(group);
	vcmGroup = group;
	*/

}

function qpDraw(q) {
	//console.log(k);
	//console.log(q + " " + prevqp);

	// if q -1 then draw all objects
	if (querypoints.length == 0) return;
	if (q == -1) {
		for (q = 0; q< querypoints.length; q++) {
			var p = querypoints[q]['position'].split(' ');
			//console.log(p);
			var x = parseInt(p[0]);
			var y = parseInt(p[1]);
			var z = parseInt(p[2]);

			//console.log(p);
			var group = new THREE.Group();
			/*
			var geometry = new THREE.Geometry();  
		    geometry.vertices.push(
		    new THREE.Vector3(0,0,0),//vertex0
		    new THREE.Vector3(1, 2, 1),
		    new THREE.Vector3(-1, 2, 1),
		    new THREE.Vector3(-1, 2, -1),
		    new THREE.Vector3(1, 2, -1)
		    );

		    geometry.faces.push(
		    new THREE.Face3(0,1,2),
		    new THREE.Face3(0,2,3),
		    new THREE.Face3(0,3,4),
		    new THREE.Face3(0,4,1),
		    new THREE.Face3(1,3,2),
		    new THREE.Face3(1,4,3)//use vertices of rank 2,1,0
		    //new THREE.Face3(0,1,3),
		    //new THREE.Face3(0,2,3),
		    //new THREE.Face3(1,2,3)//vertices[3],1,2...
			);
			var object = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial( { color: 0x000000, opacity: 1 } ) );
			object.name = q;
			object.position.x = x;
			object.position.y = y;
			object.position.z = z;
			*/

			//var materialA = new THREE.SpriteMaterial( { map: mapA, color: 0xffffff, fog: false } );
			var material = materialA.clone();
			//material.color.setHSL( 0.5 * Math.random(), 0.75, 0.5 );
			//material.map.offset.set( -0.5, -0.5 );
			//material.map.repeat.set( 2, 2 );
			var object = new THREE.Sprite( material );
			object.position.set( x, y+1.5, z );
			object.scale.set( 3, 3, 3 );
			//sprite.position.normalize();
			//sprite.position.multiplyScalar( 15 );
			//scene.add(sprite);
			object.name = q;


			
			for (var j=0; j< Object.keys(querypoints[q]['visible_planes']).length; j++) {
				var p1 = querypoints[q]['visible_planes'][j]['boundary'][0].split(' ');
				var p2 = querypoints[q]['visible_planes'][j]['boundary'][1].split(' ');
				var p3 = querypoints[q]['visible_planes'][j]['boundary'][2].split(' ');
				var p4 = querypoints[q]['visible_planes'][j]['boundary'][3].split(' ');
				for (var k=0; k<3; k++) {
					p1[k] = parseFloat(p1[k]);
					p2[k] = parseFloat(p2[k]);
					p3[k] = parseFloat(p3[k]);
					p4[k] = parseFloat(p4[k]);
				}
				var ch = 0.1;
				var dx = 0, dy = 0, dz = 0;
				var planeid = querypoints[q]['visible_planes'][j]['planeid'];
				//console.log(planeid);
				if (planeid == 0) dz = -ch;
				else if (planeid == 1) dz = ch;
				else if (planeid == 2) dx = ch;
				else if (planeid == 3) dx = -ch;
				else if (planeid == 4) dy = ch;
				else if (planeid == 5) dy = -ch;
				
				var geometry = new THREE.Geometry();  
			    geometry.vertices.push(
			    new THREE.Vector3(p1[0]+dx, p1[1]+dy, p1[2]+dz),//vertex0
			    new THREE.Vector3(p2[0]+dx, p2[1]+dy, p2[2]+dz),
			    new THREE.Vector3(p3[0]+dx, p3[1]+dy, p3[2]+dz),
			    new THREE.Vector3(p4[0]+dx, p4[1]+dy, p4[2]+dz)
			    );


			    geometry.faces.push(
			    new THREE.Face3(0,1,2),//use vertices of rank 2,1,0
			    new THREE.Face3(2,3,0)//vertices[3],1,2...
		    	);
				var material = new THREE.MeshBasicMaterial( {color: 0xffffff, side: THREE.DoubleSide, alphaTest: 0.5} );
				var plane = new THREE.Mesh(geometry, material);
				group.add(plane);
				

				var geometry = new THREE.Geometry();  
			    geometry.vertices.push(
			    new THREE.Vector3(p1[0], p1[1], p1[2]),//vertex0
			    new THREE.Vector3(p2[0], p2[1], p2[2]),
			    new THREE.Vector3(p3[0], p3[1], p3[2]),
			    new THREE.Vector3(p4[0], p4[1], p4[2]),
			    new THREE.Vector3(x, y, z)
			    );


			    geometry.faces.push(
			    new THREE.Face3(0,1,4),
			    new THREE.Face3(1,2,4),
			    new THREE.Face3(2,3,4),
			    new THREE.Face3(3,0,4)
			   
		    	);
				var material = new THREE.MeshBasicMaterial( {color: 0x00ff00, side: THREE.DoubleSide, alphaTest: 0.5, transparent: true, opacity: 0.5});
				var plane = new THREE.Mesh(geometry, material);
				group.add(plane);

				for (var k=0; k<Object.keys(querypoints[q]['visible_planes'][j]['invisible_parts']).length; k++) {
					var p1 = querypoints[q]['visible_planes'][j]['invisible_parts'][k]['boundary'][0].split(' ');
					var p2 = querypoints[q]['visible_planes'][j]['invisible_parts'][k]['boundary'][1].split(' ');
					var p3 = querypoints[q]['visible_planes'][j]['invisible_parts'][k]['boundary'][2].split(' ');
					var p4 = querypoints[q]['visible_planes'][j]['invisible_parts'][k]['boundary'][3].split(' ');
					for (var l=0; l<3; l++) {
						p1[l] = parseFloat(p1[l]);
						p2[l] = parseFloat(p2[l]);
						p3[l] = parseFloat(p3[l]);
						p4[l] = parseFloat(p4[l]);
					}
					var ch = 0.2;
					var dx = 0, dy = 0, dz = 0;
					//console.log(planeid);
					if (planeid == 0) dz = -ch;
					else if (planeid == 1) dz = ch;
					else if (planeid == 2) dx = ch;
					else if (planeid == 3) dx = -ch;
					else if (planeid == 4) dy = ch;
					else if (planeid == 5) dy = -ch;
					var geometry = new THREE.Geometry();  
				    geometry.vertices.push(
				    new THREE.Vector3(p1[0]+dx, p1[1]+dy, p1[2]+dz),//vertex0
				    new THREE.Vector3(p2[0]+dx, p2[1]+dy, p2[2]+dz),
				    new THREE.Vector3(p3[0]+dx, p3[1]+dy, p3[2]+dz),
				    new THREE.Vector3(p4[0]+dx, p4[1]+dy, p4[2]+dz)
				    );


				    geometry.faces.push(
				    new THREE.Face3(0,1,2),//use vertices of rank 2,1,0
				    new THREE.Face3(2,3,0)//vertices[3],1,2...
			    	);
					var material = new THREE.MeshBasicMaterial( {color: 0x000000, side: THREE.DoubleSide, alphaTest: 0.5} );
					var plane = new THREE.Mesh(geometry, material);
					group.add(plane);
				}

			}
			if (q < k_ans) {
				// rank marker
				var textureLoader = new THREE.TextureLoader();
				var mapR = textureLoader.load( "/static/textures/numbers_blue/number_" + (q+1) + ".png");
				var materialR = new THREE.SpriteMaterial( { map: mapR, color: 0xffffff, fog: false } );
				var material = materialR.clone();
				var rankMark = new THREE.Sprite( material );
				rankMark.position.set( x, y+5, z );
				rankMark.scale.set( 2, 2, 2 );
				rankMark.name = q;
				scene.add(rankMark);
				rankMarkRendered.push(rankMark);
			}
			
			scene.add(object);
			qpRendered.push(object);
			qpGroupRendered.push(group);
		}
		//console.log(qpRendered);
	}
	// else remove that object
	else {
		//var prev = ((q-1) + querypoints.length ) % querypoints.length;
		if (prevqp >= 0) {
			scene.remove(qpRendered[prevqp]);
			scene.remove(qpGroupRendered[prevqp]);
			//qpRendered[prevqp].material.color.setHex(0x000000);
			qpRendered[prevqp].material = materialA.clone();
			scene.add(qpRendered[prevqp]);
		}

		scene.remove(qpRendered[q]);
		//qpRendered[q].material.color.setHex(0xff0000);
		qpRendered[q].material = materialB.clone();
		scene.add(qpRendered[q]);
		scene.add(qpGroupRendered[q]);
		document.getElementById('logger').innerHTML = "Query location picked: (" + qpRendered[q].position.x + ", " + (qpRendered[q].position.y-1.5) + ", " + qpRendered[q].position.z + ")" +
		"<br>Rank: " + (q+1) + "<br>Visibility measure: " + querypoints[q].visibility ;
	}
	
}

function onWindowResize() {
	camera.aspect = window.innerWidth / window.innerHeight;
	camera.updateProjectionMatrix();

	renderer.setSize( window.innerWidth, window.innerHeight );

}

function onDocumentMouseDown( event ) {
	//console.log(prevPick);
	//event.preventDefault();

	mouse.x = ( event.clientX / renderer.domElement.clientWidth ) * 2 - 1;
	mouse.y = - ( event.clientY / renderer.domElement.clientHeight ) * 2 + 1;

	raycaster.setFromCamera( mouse, camera );

	var intersects = raycaster.intersectObjects( qpRendered );
	if ( intersects.length > 0 ) {
		//intersects[ 0 ].object.material.color.setHex( 0xff0000 );
		//console.log(intersects[0].object.name);
		if (prevQPPick !== null) {
			if (prevQPPick === intersects[0].object) return;
			//prevQPPick.material.color.setHex( 0x000000);
		}
		prevQPPick = intersects[0].object;
		$('#mvqViewResults').show();
		//document.getElementById('logger').innerHTML = "Query location picked: (" + prevQPPick.position.x + ", " + (prevQPPick.position.y-1.5) + ", " + prevQPPick.position.z + ")" +
		//"<br>Rank: " + prevQPPick.name + "<br>Visibility measure: " + querypoints[prevQPPick.name].visibility ;
		prevqp = qp;
		qp = prevQPPick.name;
		//console.log(qp + " " + prevqp)
		qpDraw(qp);
		return;
		//targetSet = true;
		
	}

	if (event.shiftKey) {
		var intersects = raycaster.intersectObjects( objects );

		if ( intersects.length > 0 ) {
			intersects[ 0 ].object.material.color.setHex( 0x000000 );
			console.log(intersects[0].object.name);
			if (prevPick !== null) {
				if (prevPick === intersects[0].object) return;
				prevPick.material.color.setHex( Math.random() * 0xffffff);
			}
			prevPick = intersects[0].object;
			var temp = intersects[0].object.name.split(' ');
			target['x1'] = parseFloat(temp[0]);
			target['y1'] = parseFloat(temp[1]);
			target['z1'] = parseFloat(temp[2]);
			target['x2'] = parseFloat(temp[3]);
			target['y2'] = parseFloat(temp[4]);
			target['z2'] = parseFloat(temp[5]);

			target['ox'] = intersects[0].object.position.x;
			target['oy'] = intersects[0].object.position.y;
			target['oz'] = intersects[0].object.position.z;

			//console.log(target);
			document.getElementById('logger').innerHTML = "Target set: (" + target['x1'] + ", " + target['y1'] + ", " + target['z1'] + "), (" + target['x2'] + ", " + target['y2'] + ", " + target['z2'] + ")";
			targetSet = true;

		}
	}
	
	
	


	//update();
}


function animate() {
	requestAnimationFrame( animate );
	controls.update(); // required if controls.enableDamping = true, or if controls.autoRotate = true
	stats.update();
	//update();
	render();

}

function update() {
	var virtual_z = 20;

	for (var i=0;i<tempqpRendered.length;i++) {
		var sprite = tempqpRendered[i];
		var v=sprite.position.clone().applyMatrix4(camera.matrixWorldInverse);
		var scale = Math.abs(v.z)/virtual_z;
		//if (camera.position.z > 0) scale=Math.abs(v.z-camera.position.z)/virtual_z;
		//else scale=Math.abs(camera.position.z+v.z)/virtual_z;
		if (scale < 3) scale = 3;
		//console.log(scale);
		sprite.scale.set(scale,scale,scale);
	}
	for (var i=0;i<qpRendered.length;i++) {
		var sprite = qpRendered[i];
		var v=sprite.position.clone().applyMatrix4(camera.matrixWorldInverse);
		var scale = Math.abs(v.z)/virtual_z;
		//if (camera.position.z > 0) scale=Math.abs(v.z-camera.position.z)/virtual_z;
		//else scale=Math.abs(camera.position.z+v.z)/virtual_z;
		if (scale < 3) scale = 3;
		//console.log(scale);
		sprite.scale.set(scale,scale,scale);
	}
	for (var i=0;i<rankMarkRendered.length;i++) {
		var sprite = rankMarkRendered[i];
		var v=sprite.position.clone().applyMatrix4(camera.matrixWorldInverse);
		var scale = Math.abs(v.z)/virtual_z;
		//if (camera.position.z > 0) scale=Math.abs(v.z-camera.position.z)/virtual_z;
		//else scale=Math.abs(camera.position.z+v.z)/virtual_z;
		if (scale < 2) scale = 2;
		//console.log(scale);
		sprite.scale.set(scale,scale,scale);
	}
}

function compareCameras(c1, c2) {
	if (c1.position.z != c2.position.z || c1.position.y != c2.position.y || c1.position.x != c2.position.x) return true;
	return false;
}

function render() {
	/*
	camera.position.set(myCamera.camPosition.x, myCamera.camPosition.y, myCamera.camPosition.z);
	camera.lookAt(myCamera.look);
	camera.up = myCamera.upDir;
	*/
	var cameraChanged = compareCameras(camera, camera2);
	if (cameraChanged) update();
	raycaster.setFromCamera( mouse, camera );	
	renderer.render( scene, camera );
	camera2.copy(camera);

}

function get(url, params, doneCallBack) {
	var xhr = new XMLHttpRequest();
	xhr.onreadystatechange = handleStateChange;
	url += "?" + params;
	console.log(url);
	xhr.open("GET", url, true);
	xhr.send();

	function handleStateChange() {
		if (xhr.readyState === 4) {
			doneCallBack(xhr.status == 200 ? xhr.responseText : null);
		}
	}
}

function runMVQ() {

	if (!qpLoaded) {
		document.getElementById('logger').innerHTML = "Query locations not set"; return;
	}
	else if (!dbLoaded) {
		document.getElementById('logger').innerHTML = "Obstacle set not loaded"; return;
	}
	k_ans = document.getElementById('k').value;
	var params = "mode=" + document.getElementById('mode').value;
	params += "&ob=" + document.getElementById('ob').value;
	params += "&qp=" + document.getElementById('qp').value;
	params += "&k=" + k_ans;
	get('http://127.0.0.1:8000/mvq', params, function(text) {
		if (text != null) {	
			var json_obj = JSON.parse(text);
			//console.log(json_obj);
			var size = Object.keys(json_obj).length;
			//console.log(size);
			for (var i=0;i<size;i++) {
				querypoints.push(json_obj[i]);
			}
			//console.log(querypoints);
			for (var i=0;i<tempqpRendered.length; i++) {
				scene.remove(tempqpRendered[i]);
			}
			tempqpRendered = [];
			qpDraw(-1);
			s = '<table class="table table-condensed">';
			s += '<tr><th>Rank</th><th>Location</th><th>Visibility</th></tr>'
			for (var i=0;i<k_ans;i++) {
				s += '<tr><td>' + (i+1) + '</td>';
				s += '<td>' + querypoints[i].position + '</td>';
				s += '<td>' + querypoints[i].visibility + '</td>';
			}
			s += '</table>';
			mvqHTML = s;
			//console.log(s);
			$('#logger').html(mvqHTML);
			//console.log(qpRendered);
			//console.log(querypoints);
		}
		else console.log("test");
	});
}

function loadQP() {
	for (var i=0;i<tempqpRendered.length; i++) {
		scene.remove(tempqpRendered[i]);
	}
	tempqpRendered = [];
	tempqp = [];
	for (var i=0;i<qpRendered.length;i++) {
		scene.remove(qpRendered[i]);
	}
	qpRendered = [];
	for (var i=0;i<Object.keys(qpGroupRendered).length;i++) {
		scene.remove(qpGroupRendered[i]);
	}
	qpGroupRendered = [];

	
	var params = "path=" + document.getElementById('qp').value;
	params += "&type=qp";
	
	get('http://127.0.0.1:8000/read_file', params, function(text) {
		if (text === null) {
		}
		else {
			var json_obj = JSON.parse(text)
			//var size = json_obj['size'];
			for (var line = 0; line <Object.keys(json_obj).length; line++) {
				var p = json_obj[line].split(' ');
				var x = parseFloat(p[1]);
				var y = parseFloat(p[2]);
				var z = parseFloat(p[3]);
				/*
				//console.log(p);
				//console.log(x1 + " " + x2 + " " + y1 + " " + y2 + " " + z1 + " " + z2);
				//var geometry = new THREE.BoxGeometry(1,1,1);
				var geometry = new THREE.Geometry();  
			    geometry.vertices.push(
			    new THREE.Vector3(0,0,0),//vertex0
			    new THREE.Vector3(1, 2, 1),
			    new THREE.Vector3(-1, 2, 1),
			    new THREE.Vector3(-1, 2, -1),
			    new THREE.Vector3(1, 2, -1)
			    );

			    geometry.faces.push(
			    new THREE.Face3(0,1,2),
			    new THREE.Face3(0,2,3),
			    new THREE.Face3(0,3,4),
			    new THREE.Face3(0,4,1),
			    new THREE.Face3(1,3,2),
			    new THREE.Face3(1,4,3)//use vertices of rank 2,1,0
			    //new THREE.Face3(0,1,3),
			    //new THREE.Face3(0,2,3),
			    //new THREE.Face3(1,2,3)//vertices[3],1,2...
				);
				var object = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial( { color: 0x000000, opacity: 1 } ) );
				*/
				//var materialA = new THREE.SpriteMaterial( { map: mapA, color: 0xffffff, fog: false } );
				var material = materialA.clone();
				//material.color.setHSL( 0.5 * Math.random(), 0.75, 0.5 );
				//material.map.offset.set( -0.5, -0.5 );
				//material.map.repeat.set( 2, 2 );
				var object = new THREE.Sprite( material );
				object.position.set( x, y+1.5, z );
				object.scale.set( 3, 3, 3 );
				//sprite.position.normalize();
				//sprite.position.multiplyScalar( 15 );
				//scene.add(sprite);
				
				object.name = json_obj[line];
				/*
				object.position.x = x;
				object.position.y = y;
				object.position.z = z;
				*/
				scene.add(object);
				tempqpRendered.push(object);
			}
			qpLoaded = true;
		}
		//console.log(text);
	});
	var geometry = new THREE.BoxGeometry(10,10,10);
	var object = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial( { color: 0x000000, opacity: 1 } ) );
	scene.add(object);

}


function loadText() {
	if (!targetSet) {
		document.getElementById('logger').innerHTML = "Target not set"; return;
	}
	else if (!dbLoaded) {
		document.getElementById('logger').innerHTML = "Obstacle set not loaded"; return;
	}
	dir = $('#axis').val();
	text = $('#vcm_text').val();
	console.log(dir + " " + text);
	loadFont();
}

function loadDB() {
	for (var i=0;i<qpRendered.length;i++) {
		scene.remove(qpRendered[i]);
	}
	qpRendered = [];
	for (var i=0;i<Object.keys(qpGroupRendered).length;i++) {
		scene.remove(qpGroupRendered[i]);
	}
	qpGroupRendered = [];
	for (var i=0;i<objects.length; i++) {
		scene.remove(objects[i]);
	}
	objects = [];

	var params = "path=" + document.getElementById('ob').value;
	params += "&type=ob";
	
	get('http://127.0.0.1:8000/read_file', params, function(text) {
		if (text === null) {
		}
		else {
			var json_obj = JSON.parse(text)
			var size = json_obj['size'];
			for (var line = 1; line <=size; line++) {
				var p = json_obj[line].split(' ');
				var x1 = parseFloat(p[0]);
				var x2 = parseFloat(p[3]);
				var y1 = parseFloat(p[1]);
				var y2 = parseFloat(p[4]);
				var z1 = parseFloat(p[2]);
				var z2 = parseFloat(p[5]);

				//console.log(p);
				//console.log(x1 + " " + x2 + " " + y1 + " " + y2 + " " + z1 + " " + z2);
				var geometry = new THREE.BoxGeometry(1,1,1);
				var object = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial( { color: Math.random() * 0xffffff, opacity: 1 } ) );
				object.name = json_obj[line];
				object.position.x = (x1+x2)/2;
				object.position.y = (y1+y2)/2;
				object.position.z = (z1+z2)/2;


				//console.log(object.position);
				object.scale.x =  Math.abs(x2-x1);
				object.scale.y = Math.abs(y2-y1);
				object.scale.z = Math.abs(z2-z1) ;
				//console.log(object.position);
				scene.add(object);
				objects.push(object);
			}
			dbLoaded = true;
		}
		//console.log(text);
	});
}

function toggleObstacles() {
	if (objectsHidden) {		
		for (var i=0;i<objects.length;i++)
			scene.add(objects[i]);
		objectsHidden = false;
	}
	else {
		for (var i=0;i<objects.length;i++)
			scene.remove(objects[i]);
		objectsHidden = true;
	}
}

function runVCM() {
	// clear MVQ stuff
	renderer.setClearColor(0x999999);
	if (!targetSet) {
		document.getElementById('logger').innerHTML = "Target not set"; return;
	}
	else if (!dbLoaded) {
		document.getElementById('logger').innerHTML = "Obstacle set not loaded"; return;
	}

	dir = document.getElementById('axis').value;

	var params = "mode=" + document.getElementById('mode').value;
	params += "&ob=" + document.getElementById('ob').value;
	params += "&dir=" + dir;// + document.getElementById('dir').value;
	params += "&x1=" + target['x1'] + "&y1=" + target['y1'] + "&z1=" + target['z1'] + "&x2=" + target['x2'] + "&y2=" + target['y2'] + "&z2=" + target['z2'];
	if (mode === "tvcm") params += "&text=" + $('#fontsize').val();
	else params += "&text=-1";
	get('http://127.0.0.1:8000/vcm', params, function(text) {
		if (text != null) {	
			var json_obj = JSON.parse(text);
			vcmArray = json_obj;
			totp = json_obj['totp'];
			maxseg = json_obj['maxseg'];
			nseg = json_obj['nseg'];
			//console.log(vcmArray);
			//console.log(json_obj);
			$('#logger').html("VCM generated");
		}
		else console.log("test");
	});
}

function run() {

	$('#logger').html('<img src="/static/gears.gif" width="50px" style="display: block; margin-left:auto; margin-right:auto;">');
	
	if (mode === "mvq") {
		runMVQ();
	}
	else if (mode === "vcm" || mode === "tvcm") {
		runVCM();
	}
	
	
	
}

function createText() {
	textGeo = new THREE.TextGeometry( text, {
		font: font,
		size: size,
		height: height,
		curveSegments: curveSegments,
		bevelThickness: bevelThickness,
		bevelSize: bevelSize,
		bevelEnabled: bevelEnabled,
		material: 0,
		extrudeMaterial: 1
	});
	textGeo.computeBoundingBox();
	textGeo.computeVertexNormals();
	// "fix" side normals by removing z-component of normals for side faces
	// (this doesn't work well for beveled geometry as then we lose nice curvature around z-axis)
	if ( ! bevelEnabled ) {
		var triangleAreaHeuristics = 0.1 * ( height * size );
		for ( var i = 0; i < textGeo.faces.length; i ++ ) {
			var face = textGeo.faces[ i ];
			if ( face.materialIndex == 1 ) {
				for ( var j = 0; j < face.vertexNormals.length; j ++ ) {
					face.vertexNormals[ j ].z = 0;
					face.vertexNormals[ j ].normalize();
				}
				var va = textGeo.vertices[ face.a ];
				var vb = textGeo.vertices[ face.b ];
				var vc = textGeo.vertices[ face.c ];
				var s = THREE.GeometryUtils.triangleArea( va, vb, vc );
				if ( s > triangleAreaHeuristics ) {
					for ( var j = 0; j < face.vertexNormals.length; j ++ ) {
						face.vertexNormals[ j ].copy( face.normal );
					}
				}
			}
		}
	}
	var centerOffset = -0.5 * ( textGeo.boundingBox.max.x - textGeo.boundingBox.min.x );
	textMesh1 = new THREE.Mesh( textGeo, textMaterial );
	//console.log(dir + " " + text);
	if (dir == 0) {
		textMesh1.position.x = target['x1'] - 0.1;
		textMesh1.position.y = target['oy'] ;
		textMesh1.position.z = target['oz']+ centerOffset;
		textMesh1.rotation.y = -Math.PI / 2;
	}
	else if (dir == 1) {
		textMesh1.position.x = target['x2'] + 0.1;
		textMesh1.position.y = target['oy'];
		textMesh1.position.z = target['oz'] - centerOffset;
		textMesh1.rotation.y = Math.PI / 2;
	}
	else if (dir == 2) {
		textMesh1.position.x = target['ox'] + centerOffset;
		textMesh1.position.y = target['y1'] - 0.1;
		textMesh1.position.z = target['oz'];
		textMesh1.rotation.x = Math.PI / 2;
	}
	else if (dir == 3) {
		textMesh1.position.x = target['ox'] + centerOffset;
		textMesh1.position.y = target['y2'] + 0.1;
		textMesh1.position.z = target['oz'] ;
		textMesh1.rotation.x = -Math.PI / 2;
	}
	else if (dir == 4) {
		textMesh1.position.x = target['ox'] - centerOffset;
		textMesh1.position.y = target['oy'];
		textMesh1.position.z = target['z1'] - 0.1;
		textMesh1.rotation.y = Math.PI;
	}
	else if (dir == 5) {
		textMesh1.position.x = target['ox'] + centerOffset;
		textMesh1.position.y = target['oy'];
		textMesh1.position.z = target['z2'] + 0.1;
		//textMesh1.rotation.y = Math.PI / 2;
	}

	textGroup.add( textMesh1 );


}

function refreshText() {
	
	textGroup.remove( textMesh1 );
	if ( !text ) return;
	createText();
}

function mvqViewResults() {
	if (mvqHTML) {
		$('#logger').html(mvqHTML);
		$('#mvqViewResults').hide();
	}
}

function reset() {
	$('#logger').html("Settings reset");
	var delta = 0.1;
	objects = [];
	tempqp = [];
	querypoints = [];
	qpLoaded = false;
	qp = -1;	// which query point drawing now
	prevqp = null;
	tempqpRendered = [];
	qpRendered = [];
	rankMarkRendered = [];
	qpGroupRendered = [];

	vcmArray = [];
	vcmGroup = null;

	target = [];
	targetSet = false;
	dbLoaded = false;

	prevPick = null;
	prevQPPick = null;
	init();
	animate();
	//scene.remove(scene.children);

}

function getTextDiv() {
	var ht = '<div class="col-lg-2">Text</div>' + 
				'<div class="col-lg-7">' +
					'<input type="text" name="vcm_text" id="vcm_text" class="form-control" />' +
				'</div>' +
				'<div class="col-lg-3" style="float:right;">' +
					'<select name="fontsize" id="fontsize" class="form-control" ">' + 
						'{% for f in fontsize %}' +
							'<option value="{{ f }}">{{ f }}</option>' + 
						'{% endfor %}' +
					'</select>' +
				'</div>';
	return ht;
}

function addText() {
	$textID++;
	var $t = "t" + $textID;
	var $div = $("<div>", {id: $t, class: "row custrow"});
	$div.append($('#t1').html());
	//console.log(n);
	$('#texts').append($div);
}

function removeText() {
	if ($textID == 1) return;
	var $t = "t" + $textID;
	//console.log($t);
	$textID--;
	$("#" + $t).remove();
}