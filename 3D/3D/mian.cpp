#pragma warning  ( disable : 4786)
#include <vector>
#include <list>
#include <map>
#include <set>
#include <deque>
#include<queue>
#include <stack>
#include <bitset>
#include <algorithm>
#include <functional>
#include <numeric>
#include <utility>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <queue>
#include <cassert>

#include<GL/glut.h>
#include<GL/glui.h>

#include "camera.h"
#include "myCamera.h"
#include "macros.h"
#include "gpc.h"
#include "monotone.h"
#include "text3d.h"
#include "vcm.h"
#include "mvq.h"

//R-tree stuffs
#include "./rtree/rtree.h"
#include "./rtree/rtnode.h"
#include "./rtree/entry.h"
#include "./blockfile/blk_file.h"
#include "./blockfile/cache.h"
#include "./linlist/linlist.h"
#include "./rtree/rtree_cmd.h"
#include "./rtree/distance.h"
#include "./his/histogram.h"
#include "tinyfiledialogs.h"

using namespace std;

#define INTRO 0
#define MVQ_S 1
#define VCM_S 2
#define TXT_S 3

int mode = 1;
	
VCM vcm;
MVQ mvq;

/* camera vars */
myCamera myCam;

//

#define WINDOW_WIDTH 1300
#define WINDOW_HEIGHT 800

//grahpics stuff

bool isRotating = false;
bool isFrameRotating = false;
int cameraRadius = 100;
int rotatingCameraHeight;

int cameraX;
int cameraY;
int cameraZ;
bool changeCameraPos = false;

int lookAtX = 0;
int lookAtY = 0;
int lookAtZ = 0;
bool changeLookAtPos = false;
///////////////////for ne camera variable
//animation variable
double lookatX, lookatY, lookatZ;
double  cameraAngle, cameraHeight, cameraDelta, cameraMove, cameraVertical;

Camera cam;
int window;
GLUI *glui;

int vr = 0;

void cleanup() {
	t3dCleanup();
}

void animate(){
	if (glutGetWindow() != window)
		glutSetWindow(window);
	glutPostRedisplay();
	glui->sync_live();
	//codes for any changes in Models, Camera
}



void initRendering() {
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);
	glShadeModel(GL_SMOOTH);
	t3dInit();
}


void init(void)
{
	//for surface mesh
	/*
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 4,
	0, 1, 12, 4, &ctrlpoints[0][0][0]);
	glEnable(GL_MAP2_VERTEX_3);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);
	initlights();

	*/
	/* kaysar vai's cam */

	initRendering(); 
	cameraX = 300;
	cameraY = 0;
	cameraZ = 30;

	lookatX = 0;
	lookatY = 0;
	lookatZ = 0;

	cameraRadius = 100;
	cameraHeight = 0;
	cameraAngle = 0;
	cameraDelta = 0.025;
	cameraMove = 1.8;
	cameraVertical = 0;

	/* my cam */
	glMatrixMode(GL_PROJECTION);

	//initialize the matrix
	glLoadIdentity();

	//give PERSPECTIVE parameters
	gluPerspective(70,	1,	0.1,	10000.0);
	// gluNurbsCallback(theNurb, GLU_ERROR, nurbsError);
}

void display(void)
{
	int i, j;
	if (vcm.text.empty()) glClearColor(0, 0, 0, 0); //color black
	else glClearColor(0.7,0.7,0.7, 0);	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);

	glLoadIdentity();

//	cameraRadius=abs(cameraX-lookAtX);
//	gluLookAt(cameraRadius*cos(cameraAngle), cameraRadius*sin(cameraAngle), cameraZ,		lookAtX,lookAtY,lookAtZ,		0,0,1);
	

	//gluLookAt(  CO(cam.eye) , CO( cam.eye.add(  cam.n ) ) , CO(cam.v) );
//	gluLookAt(  CO(cam.eye) , CO( cam.eye.add(  cam.n ) ) , CO(cam.v) );

//	gluLookAt( 0, 50, 0  , 0,0,0 , 0,0,1 );
	//gluLookAt( cam.cameraRadius*cos(cam.cameraAngle), cam.cameraRadius*sin(cam.cameraAngle), cam.cameraHeight  , 0,0,0 , 0,0,1 );
	/* kaysar vai's cam */
	// gluLookAt(CO(cam.eye), CO(cam.eye.add(cam.n)), CO(cam.v));
	/* my cam */
	gluLookAt(CO(myCam.camPosition), CO(myCam.look), CO(myCam.upDir));
	//gluLookAt(  CO(cam.eye) , 0,0,0 , CO(cam.v) );

	//again select MODEL-VIEW
	glMatrixMode(GL_MODELVIEW);
	/*
	glPushMatrix();
	glColor3ub(0,200,0);
	glPointSize(5.0);
	glScalef(10,10,10);
	for (j = 0; j <= 8; j++) {
	glBegin(GL_POINTS);//GL_LINE_STRIP

	for (i = 0; i <= 30; i++)
	glEvalCoord2f((GLfloat)i/30.0, (GLfloat)j/8.0);
	glEnd();
	glBegin(GL_LINE_STRIP);
	for (i = 0; i <= 30; i++)
	glEvalCoord2f((GLfloat)j/8.0, (GLfloat)i/30.0);
	glEnd();
	}
	glPopMatrix();
	*/

	//draw_axis();
//	debug1();

	//debug4();
	if (mode == VCM_S) vcm.debug5();
	else if (mode == MVQ_S) mvq.debug();
	
	glFlush();
	glutSwapBuffers();
}
void keyboardListener(unsigned char key, int x,int y){
	double dd=0.05;

	switch (key)
        {
                // camera slide controls
               
					/*
                case 'w': cam.slide(0, 0, -ds); break; // slide camera forward
                case 's': cam.slide(0, 0, ds); break; // slide camera back
                case 'q': cam.slide(0, ds, 0); break; // slide camera up
                case 'z': cam.slide(0, -ds, 0); break; // slide camera down
                case 'a': cam.slide(-ds, 0, 0); break; // slide camera left
                case 'd': cam.slide(ds, 0, 0); break; // slide camera right
					*/

			/*
                // camera pitch controls
                case 'i': cam.rotate(Vector3(1, 0, 0), -1); break;
                case 'I': cam.rotate(Vector3(1, 0, 0), 1); break;
                // camera yaw controls
                case 'j': cam.rotate(Vector3(0, 1, 0), -1); break;
                case 'J': cam.rotate(Vector3(0, 1, 0), 1); break;
                // camera roll controls
                case 'k': cam.rotate(Vector3(0, 0, 1), 1); break;
                case 'K': cam.rotate(Vector3(0, 0, 1), -1); break;

					*/
					
				case 'n':	vcm.showid = (vcm.showid + 1) % vcm.targetPoints.size(); break;
				case 'm':	vcm.curSegId = (vcm.curSegId + 1) % vcm.NDISTESEG; break;
				case 'M':	vcm.curSegId = (vcm.curSegId - 1 + vcm.NDISTESEG) % vcm.NDISTESEG; break;
				case 'r':	vcm.showDistRec ^= 1; break;
				case 'p':	vcm.showProjPoly ^= 1; break;
				case 'o':	
					if (mode==VCM_S) vcm.showObstacle ^= 1; 
					else if (mode == MVQ_S) mvq.showObstacles = !mvq.showObstacles;
					break;
				case 'a':	vcm.showAll ^= 1; break;
				case 'g':	vcm.drawg ^= 1; break;

				case 'v': mvq.vr = (mvq.vr + 1) % mvq.queryPoints.size(); break;

				// camera pitch controls
                case '1':	myCam.pitch(dd); break;
                case '2':	myCam.pitch(-dd); break;
                // camera yaw controls
                case '3':	myCam.yaw(dd); break;
                case '4':	myCam.yaw(-dd); break;
                // camera roll controls
                case '5':	myCam.roll(dd); break;
                case '6':	myCam.roll(-dd); break;
				
				case 'b': glClearColor(0, 0, 0, 1.0f); break;
                case ESCAPE:	glutDestroyWindow(window); exit(0); break;
        }
        glutPostRedisplay(); // draws it again
}

void specialKeyListener(int key, int x,int y){

//	double ds=100;//london
	
	double ds=10;


	switch(key)
	{
		
		case GLUT_KEY_DOWN: myCam.slide(ds, 1); break; // slide camera forward
		case GLUT_KEY_UP: myCam.slide(ds, 2); break; // slide camera back
		case GLUT_KEY_LEFT: myCam.slide(ds, 3); break; // slide camera left
		case GLUT_KEY_RIGHT: myCam.slide(ds, 4); break; // slide camera right
		case GLUT_KEY_PAGE_UP: myCam.slide(ds, 5); break; // slide camera up
		case GLUT_KEY_PAGE_DOWN: myCam.slide(ds, 6); break; // slide camera down
		
	/*
		case GLUT_KEY_DOWN: cam.increaseradius(+10); break;
		case GLUT_KEY_UP: cam.increaseradius(-10); break;
		case GLUT_KEY_PAGE_UP: cam.increaseheight(10); break;
		case GLUT_KEY_PAGE_DOWN: cam.increaseheight(-10); break;
		case GLUT_KEY_LEFT: cam.rotateclock(cam.cameraAngleDelta); break;
		case GLUT_KEY_RIGHT: cam.rotateclock(-cam.cameraAngleDelta); break;
	*/
	}
	
}

void mouseMove(int x, int y) {return;}






void GlutReshape(int x, int y)
{
	float xy_aspect = (float)x / (float)y;
	glViewport(0, 0, x, y);

	glutPostRedisplay();
}


GLUI_Button *browse, *quit, *reset;
#define BROWSE_ID 1
#define QUIT_ID 2
#define VCM_ID 3
#define TEXTBOX_ID 4
#define EDITTEXT_ID 5
#define FONTSIZE_ID 6

#define MBROWSE_ID 7
#define MQUERY_ID 8
#define MVQK_ID 9
#define MVQ_RUN 10

void gl_callback(GLUI_Control *control) {
	if (control->get_id() == BROWSE_ID) {
		vcm.console_index = LOADINGDB;
		const char *filename = tinyfd_openFileDialog(
			"Load Database",
			"",
			2,
			vcm.lFilterPatterns,
			NULL,
			0);
		if (filename) {
			//puts(filename);
			//vcm.f = strdup(filename);
			vcm.setup();
			glutPostRedisplay();
		}
		vcm.console_index = DBLOADED;
	}
	else if (control->get_id() == MBROWSE_ID) {
		const char *filename = tinyfd_openFileDialog(
			"Load Database",
			"",
			2,
			vcm.lFilterPatterns,
			NULL,
			0);
		if (filename) {
			//puts(filename);
			//mvq.f = strdup(filename);
			mvq.setup();
			glutPostRedisplay();
		}
	}
	else if (control->get_id() == MQUERY_ID) {
		const char *filename = tinyfd_openFileDialog(
			"Load Query Locations",
			"",
			2,
			vcm.lFilterPatterns,
			NULL,
			0);
		if (filename) {
			//puts(filename);
			//mvq.queryPath = strdup(filename);
			mvq.loadQueryPoints();
			glutPostRedisplay();
		}
	}
	else if (control->get_id() == VCM_ID) {
		if (vcm.isTargetSet) {
			vcm.console_index = GENGVCM;
			//vcm.run2();
			vcm.console_index = VCMGEND;
		}
		else vcm.console_index = TARGETNOTSET;
	}		
	else if (control->get_id() == QUIT_ID) {
		//printf("Exiting ... \n");
		exit(1);
	}
	else if (control->get_id() == MVQ_RUN) {
		//mvq.setup();
		mvq.Mov_exp3D(10);
	}
	
}

// this is the mouse event handler
// the x and y parameters are the mouse coordinates when the button was pressed
void mouse(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		/*
		cam.eye.print();
		cam.u.print();
		cam.n.print();
		cam.v.print();
		*/
		//cout << x << " " << y << endl;

		GLint viewport[4];
		GLdouble modelview[16];
		GLdouble projection[16];
		GLfloat winX, winY, winZ;
		GLdouble posX, posY, posZ;

		glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
		glGetDoublev(GL_PROJECTION_MATRIX, projection);
		glGetIntegerv(GL_VIEWPORT, viewport);

		winX = (float)x;
		winY = (float)viewport[3] - (float)y;
		glReadPixels(x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
		gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
		//cout << posX << " " << posY << " " << posZ << endl;
		if (vcm.changeTarget(posX, posY, posZ))
			glutPostRedisplay();
	}
}



void mainGui() {
	if (glui) delete glui;
	glui = GLUI_Master.create_glui_subwindow(window,
		GLUI_SUBWINDOW_RIGHT);
	GLUI_Panel *rootpanel = new GLUI_Panel(glui, "", GLUI_PANEL_NONE);
	GLUI_Panel *titlepanel = new GLUI_Panel(rootpanel, "");
	new GLUI_StaticText(titlepanel, "Visibility Query");
	/* database panel */
	new GLUI_Separator(rootpanel);
	GLUI_Panel *mvqpanel = new GLUI_Panel(rootpanel, "MVQ Settings");
	new GLUI_Button(mvqpanel, "Load Database", MBROWSE_ID, gl_callback);
	new GLUI_Separator(mvqpanel);
	new GLUI_Button(mvqpanel, "Load Query Locations", MQUERY_ID, gl_callback);
	new GLUI_Separator(mvqpanel);
	new GLUI_Button(mvqpanel, "Run MVQ", MVQ_RUN, gl_callback);
	new GLUI_Separator(mvqpanel);
	//new GLUI_Column(edittextpanel);
	vcm.fontspinner = new GLUI_Spinner(mvqpanel, "k: ", &vcm.fontsize, FONTSIZE_ID, gl_callback);
	vcm.fontspinner->set_int_limits(1, 32);

	new GLUI_Separator(rootpanel);

	GLUI_Panel *dbpanel = new GLUI_Panel(rootpanel, "VCM Settings");
	new GLUI_Button(dbpanel, "Load Database", BROWSE_ID, gl_callback);
	new GLUI_Separator(dbpanel);
	new GLUI_Button(dbpanel, "Generate VCM", VCM_ID, gl_callback);
	new GLUI_Separator(dbpanel);
	/**** Add listbox ****/
	new GLUI_StaticText(glui, "");
	GLUI_Listbox *list = new GLUI_Listbox(dbpanel, "Select Axis: ", &vcm.curr_string);
	int i;
	for (i = 0; i<6; i++)
		list->add_item(i, vcm.string_list[i]);

	new GLUI_StaticText(glui, "");
	new GLUI_Separator(dbpanel);
	GLUI_Panel *edittextpanel = new GLUI_Panel(dbpanel, "");
	vcm.edittext = new GLUI_EditText(edittextpanel, "Text: ", vcm.text, EDITTEXT_ID, gl_callback);
	//new GLUI_Column(edittextpanel);
	vcm.fontspinner = new GLUI_Spinner(edittextpanel, "Font Size: ", &vcm.fontsize, FONTSIZE_ID, gl_callback);
	vcm.fontspinner->set_int_limits(8, 32);
	//dbpanel->add_control(edittext);

	new GLUI_Separator(rootpanel);
	/* instruction panel */
	GLUI_Panel *inspanel = new GLUI_Panel(rootpanel, "Instructions");
	new GLUI_Separator(inspanel);
	new GLUI_StaticText(inspanel, "Camera Pitch");
	new GLUI_StaticText(inspanel, "Camera Yaw");
	new GLUI_StaticText(inspanel, "Camera Roll");
	new GLUI_StaticText(inspanel, "Camera Slide");
	new GLUI_StaticText(inspanel, "");
	new GLUI_StaticText(inspanel, "Show VCM");
	new GLUI_StaticText(inspanel, "All Obstacles");
	new GLUI_Column(inspanel, true);
	new GLUI_Separator(inspanel);
	new GLUI_StaticText(inspanel, "1, 2");
	new GLUI_StaticText(inspanel, "3, 4");
	new GLUI_StaticText(inspanel, "5, 6");
	new GLUI_StaticText(inspanel, "ARROWS");
	new GLUI_StaticText(inspanel, "PG UP/DN");
	new GLUI_StaticText(inspanel, "m");
	new GLUI_StaticText(inspanel, "a");

	new GLUI_Separator(rootpanel);
	GLUI_Panel *textboxPanel = new GLUI_Panel(rootpanel, "Console");
	vcm.textbox = new GLUI_TextBox(textboxPanel, false, TEXTBOX_ID, gl_callback);
	vcm.textbox->set_w(200);
	vcm.textbox->set_text("	");
	vcm.textbox->disable();

	/***** A 'quit' button *****/
	new GLUI_Separator(rootpanel);
	new GLUI_Button(rootpanel, "Quit", QUIT_ID, gl_callback);
	new GLUI_Separator(rootpanel);
	/**** Link windows to GLUI, and register idle callback ****/
	glui->set_main_gfx_window(window);
}

int main(int argc, char **argv) {
	/*
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowPosition(50, 50);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	window = glutCreateWindow("Visibility Query");
	glutDisplayFunc(display);
	GLUI_Master.set_glutReshapeFunc(GlutReshape);
	GLUI_Master.set_glutKeyboardFunc(keyboardListener);
	GLUI_Master.set_glutSpecialFunc(specialKeyListener);
	GLUI_Master.set_glutMouseFunc(mouse);
	init();
	glEnable(GL_DEPTH_TEST);
	mainGui();
	GLUI_Master.set_glutIdleFunc(animate);
	glutMainLoop();		//The main loop of OpenGL
	*/
	
	if (strcmp(argv[1], "mvq") == 0) {
		if (argc < 5) {
			cout << "Too few arguments" << endl;
			return 0;
		}
		strcat(mvq.f, argv[2]);
		strcat(mvq.queryPath, argv[3]);
		mvq.setup();
		mvq.loadQueryPoints();
		mvq.Mov_exp3D(atoi(argv[4]));
	}
	else if (strstr(argv[1], "vcm")) {
		if (argc < 11) {
			cout << "Too few arguments" << endl;
			return 0;
		}
		strcpy(vcm.f, argv[2]); // obstacle set
		double x1 = atoi(argv[4]);
		double y1 = atoi(argv[5]);
		double z1 = atoi(argv[6]);
		double x2 = atoi(argv[7]);
		double y2 = atoi(argv[8]);
		double z2 = atoi(argv[9]);
		vcm.fontsize = atoi(argv[10]);
		if (vcm.fontsize != -1) vcm.argContainsText = true;
		vcm.setup();
		Box b(Vector3(x1, y1, z1), Vector3(x2, y2, z2));
		vcm.setTarget(b);
		//vcm.target = b;
		//vcm.target.setPlanes();
		
		vcm.run2(atoi(argv[3]));	// direction
	}
	

	return 0;
}
