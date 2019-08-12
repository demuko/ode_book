// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// プチプロ5.3 ロボカップのピットとゴールキーパー pk.cpp by Kosei Demura (2007-5-18)
// rキーを押すとボールがランダムに発射されます．ただし，キーパーの行動が組み込まれていな
// のでキーパーは微動だにしません．simLoop関数にその処理を実装してください．
//
//
// This program is a sample program of my book as follows
//“Robot Simulation - Robot programming with Open Dynamics Engine,
// (260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd.,
// Tokyo, 2007)” by Kosei Demura, which is written in Japanese (sorry).
// http://demura.net/simulation
// Please use this program if you like. However it is no warranty.
// pk.cpp by Kosei Demura (2007-5-18)
//
// 更新履歴　(change log)
// 更新履歴
// 2008-8-21: 新しい衝突検出関数ではエラーがでるのでロボットデザインを変更　makeBase2(), drawBase2()の変更
// 2008-7-7: dInitODE(),dCloseODE()の追加
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <time.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#define WHEEL_NUM 4
#define GOAL_PARTS_NUM 7

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawConvex   dsDrawConvexD
#define dsDrawLine     dsDrawLineD
#endif

static dWorldID    world;
static dSpaceID    space;
static dGeomID     ground;
static dGeomID     goal_parts[GOAL_PARTS_NUM];
static dGeomID     goal_parts2[GOAL_PARTS_NUM];

static dJointGroupID contactgroup;
static dsFunctions fn;
static int PAUSE;

typedef struct {
	dBodyID  body;
	dGeomID  geom;
	dJointID joint;
  dReal    v;           // angular velocity
	dReal    m,r,h,x,y,z; // mass, radius,height, pos(x,y,z)
  dReal    lx,ly,lz;    // sizes[3]
} MyObject;

static MyObject wheel[WHEEL_NUM],base,base2,base3,camera,ball;

static dReal  X = - 5.75, Y = 0, Z = 0.1;  // initial Robot position x,y,z
static dReal  L = 0.45, H = 0.05; // base length,width, height
static dReal  SIDE[3]  = {L, L, 0.05};
static dReal  SIDE2[3] = {0.8*L, 0.8*L,  0.05};
static dReal  SIDE3[3] = {L/12, L/12, 0.3};
static dReal  M = L/12;
static dReal  offset_z = 0.0; // 0.1
static int    POWER = 100;

// 0:right panel, 1:left panel, 2:back panel, 3;bar,
// 4: right wall paper, 5: left wall paper, 6: back wall paper
static const dReal GOAL_SIDES[GOAL_PARTS_NUM][3] = {{0.635, 0.125, 1.0},
	    {0.635, 0.125, 1.0}, {0.125, 2.01,   1.0}, {0.125,  2.27,  0.125},
	    {0.5, 0.01, 1.0}  , {0.5, 0.01, 1.0}, {0.01, 2.01, 1.0}};
static const dReal FIELD_SIDES[3] = {14,10,offset_z};


const unsigned int planecount=6;


const unsigned int pointcount=8;
unsigned int polygons[] = { //Polygons for a cube (6 squares)
    4,0,2,6,4, // positive X
    4,1,0,4,5, // positive Y
    4,0,1,3,2, // positive Z
    4,3,1,5,7, // negative X 3,1,5,7
    4,2,3,7,6, // negative Y 2,3,7,6
    4,5,4,6,7, // negative Z
};
//----> Convex Object

//<---- Convex Object2
dReal planes2[]= { // planes for a cube
    1.0f ,0.0f ,1.0f ,SIDE2[0]/2, // rear
    0.0f ,1.0f ,1.0f ,SIDE2[0]/2, // left
    0.0f ,0.0f ,1.0f ,0,          // bottom ok
    0.0f ,0.0f ,-1.0f,SIDE3[2]/2, // top ok
    0.0f ,-1.0f,1.0f ,SIDE2[0]/2, // right
    -1.0f,0.0f ,1.0f ,SIDE2[0]/2  // front
};

dReal points2[]= { // points for a cube
    SIDE3[0]/2, SIDE3[1]/2,  SIDE3[2]/2,  // point 0
   -SIDE3[0]/2, SIDE3[1]/2,  SIDE3[2]/2,  // point 1
    SIDE3[0]/2,-SIDE3[1]/2,  SIDE3[2]/2,  // point 2
   -SIDE3[0]/2,-SIDE3[1]/2,  SIDE3[2]/2,  // point 3
    SIDE2[0]/2, SIDE2[1]/2, -SIDE3[2]/2,  // point 4
   -SIDE2[0]/2, SIDE2[1]/2, -SIDE3[2]/2,  // point 5
    SIDE2[0]/2,-SIDE2[1]/2, -SIDE3[2]/2,  // point 6
   -SIDE2[0]/2,-SIDE2[1]/2, -SIDE3[2]/2,  // point 7
};
//----> Convex Object2

static void makeBase()
{
  dMass mass;
	base.m = 9.0;
  base.x = X;
	base.y = Y;
	base.z = Z;
	base.lx = SIDE[0];
  base.ly = SIDE[1];
  base.lz = SIDE[2];

  base.body  = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,base.m, base.lx, base.ly, base.lz);
  dBodySetMass(base.body,&mass);

  base.geom = dCreateBox(space,base.lx, base.ly, base.lz);
  dGeomSetBody(base.geom, base.body);
  dBodySetPosition(base.body, base.x, base.y, base.z);
}

static void makeBase2()
{
  dMass mass;
  dReal k = 0.8;
  base2.m = 1.0;
 	base2.lx = SIDE[0];
  base2.ly = SIDE[1];
  base2.lz = SIDE2[2];
  base2.x = X;
  base2.y = Y;
	base2.z = Z + SIDE[2]/2 + SIDE2[2]/2;

  base2.body  = dBodyCreate(world);
	dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,0.8*base.m, 0.8*base.lx, 0.8*base.ly,base.lz);
  dBodySetMass(base2.body,&mass);
	base2.geom = dCreateBox(space,0.8*base.lx, 0.8*base.ly, base.lz);
  dBodySetPosition(base2.body, base2.x, base2.y, base2.z);
  base2.joint = dJointCreateFixed(world,0);
 	dJointAttach(base2.joint, base.body, base2.body);
  dJointSetFixed(base2.joint);
}

static void makeBase3()
{
  dMass mass;
  base3.m = 1.0;
 	base3.lx = SIDE3[0];
  base3.ly = SIDE3[1];
  base3.lz = SIDE3[2];
  base3.x = X;
  base3.y = Y;
	base3.z = Z + SIDE[2]/2 + SIDE2[2] + SIDE3[2]/2;

  base3.body  = dBodyCreate(world);
	dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,base3.m, base3.lx, base3.ly, base3.lz);
  dBodySetMass(base3.body,&mass);
	base3.geom = dCreateConvex (space,planes2,planecount,
					points2,pointcount,polygons);
  dBodySetPosition(base3.body, base3.x, base3.y, base3.z);
  base3.joint = dJointCreateFixed(world,0);
 	dJointAttach(base3.joint, base2.body, base3.body);
  dJointSetFixed(base3.joint);
}

static void makeCamera()
{
  dMass mass;
  camera.m = 0.2;
 	camera.r = 0.1;
  camera.h = 0.05;
  camera.x = X;
  camera.y = Y;
	camera.z = Z + SIDE[2]/2 + SIDE2[2] + SIDE3[2] + (camera.h)/2;

  camera.body  = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, camera.m, 3, camera.r, camera.h);
  dBodySetMass(camera.body,&mass);
	camera.geom =  dCreateCylinder(space, camera.r, camera.h);
  dGeomSetBody(camera.geom,camera.body);
  dBodySetPosition(camera.body, camera.x, camera.y, camera.z);
  camera.joint = dJointCreateFixed(world, 0);
 	dJointAttach(camera.joint, base.body, camera.body);
  dJointSetFixed(camera.joint);
}

void makeWheel()
{
  dMass mass;
  dReal r = 0.06;
	dReal w = 0.024;
	dReal m = 0.15;
  dReal tmp_z = Z;
  dReal d = 0.01;
  dReal wx[WHEEL_NUM] = {X+SIDE[0]/2-w/2-d, X, X- (SIDE[0]/2-w/2-d),  X};
  dReal wy[WHEEL_NUM] = {Y, Y+SIDE[1]/2-w/2-d, Y, Y- (SIDE[1]/2-w/2-d)};
  dReal wz[WHEEL_NUM] = {tmp_z, tmp_z, tmp_z, tmp_z};
  dReal jx[WHEEL_NUM] = {X+SIDE[0]/2, X, X- SIDE[0]/2, X};
  dReal jy[WHEEL_NUM] = {Y, Y+ SIDE[1]/2, Y, Y- SIDE[1]/2};
  dReal jz[WHEEL_NUM] = {tmp_z, tmp_z, tmp_z, tmp_z};

  for (int i = 0; i < WHEEL_NUM; i++) {
	  wheel[i].v    = 0.0;
    wheel[i].body = dBodyCreate(world);
    dMatrix3 R;
    if (i % 2) {
      dRFromAxisAndAngle(R,1,0,0,M_PI/2.0);
    	dBodySetRotation(wheel[i].body,R);

    }
		else {
      dRFromAxisAndAngle(R,0,1,0,M_PI/2.0);
    	dBodySetRotation(wheel[i].body,R);
    }

    dMassSetZero(&mass);
    if (i % 2) {
    	dMassSetCylinderTotal(&mass,m, 1, r, w);
	  }
		else {
 			dMassSetCylinderTotal(&mass,m, 2, r, w);
	  }
    dBodySetMass(wheel[i].body,&mass);

    wheel[i].geom = dCreateCylinder(space,r, w);
    dGeomSetBody(wheel[i].geom,wheel[i].body);
    dBodySetPosition(wheel[i].body, wx[i], wy[i], wz[i]);

    wheel[i].joint = dJointCreateHinge(world,0);
    dJointAttach(wheel[i].joint,base.body,wheel[i].body);
    dJointSetHingeAnchor(wheel[i].joint, jx[i], jy[i], jz[i]);
  }
	dJointSetHingeAxis(wheel[0].joint, 1, 0, 0);
 	dJointSetHingeAxis(wheel[1].joint, 0, 1, 0);
  dJointSetHingeAxis(wheel[2].joint, 1, 0, 0);
  dJointSetHingeAxis(wheel[3].joint, 0, 1, 0);
}

static void makeBall()
{
  dMass mass;
  ball.m = 0.45;	ball.r = 0.11;
  ball.x = -4.0; ball.y = 0.0; ball.z = 0.14 + offset_z;

  ball.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass,ball.m,ball.r);
  dBodySetMass(ball.body,&mass);

  ball.geom = dCreateSphere(space,ball.r);
  dGeomSetBody(ball.geom,ball.body);
  dBodySetPosition(ball.body,ball.x, ball.y, ball.z);
}

static void makeGoal()
{
  // 0:right panel, 1:left panel, 2:back panel, 3;bar,
  // 4: right wall paper, 5: left wall paper, 6: back wall paper

  dReal pos[GOAL_PARTS_NUM][3] = {{-6.318, 1.08, 0.505},{-6.318, -1.08, 0.505},
	  {-6.59, 0, 0.505},{-6.0625, 0, 1.07},
 	  {-6.25, 1.005, 0.505},{-6.25, -1.005, 0.505}, {-6.5055, 0, 0.505}};

  // parts
  for (int i = 0; i < GOAL_PARTS_NUM; i++) {
    goal_parts[i] = dCreateBox(space, GOAL_SIDES[i][0],
				    GOAL_SIDES[i][1], GOAL_SIDES[i][2]);
    dGeomSetPosition(goal_parts[i],pos[i][0], pos[i][1], pos[i][2]);
  }

  dReal pos2[GOAL_PARTS_NUM][3] = {{6.318, 1.08, 0.505},{6.318, -1.08, 0.505},
	  {6.59, 0, 0.505},{6.0625, 0, 1.07},
 	  {6.25, 1.005, 0.505},{6.25, -1.005, 0.505}, {6.5055, 0, 0.505}};
  // parts
  for (int i = 0; i < GOAL_PARTS_NUM; i++) {
    goal_parts2[i] = dCreateBox(space, GOAL_SIDES[i][0],
				    GOAL_SIDES[i][1], GOAL_SIDES[i][2]);
    dGeomSetPosition(goal_parts2[i],pos2[i][0], pos2[i][1], pos2[i][2]+offset_z);
  }
}

static void drawBase()
{
  //dsSetColor(1.3, 1.3, 0.0);
  dsSetColor(0.15, 0.15, 0.15);
  dsDrawBox(dBodyGetPosition(base.body),dBodyGetRotation(base.body),SIDE);
}

static void drawBase2()
{
  dsSetColor(0.15, 0.15, 0.15);
  dsDrawBox(dBodyGetPosition(base2.body),dBodyGetRotation(base2.body),SIDE2);
}

static void drawBase3()
{
   dsSetColor(0.15, 0.15, 0.15);
   dsDrawConvex(dBodyGetPosition(base3.body),
	   dBodyGetRotation(base3.body),planes2,
	   planecount,points2,pointcount,polygons);
}

void drawCamera()
{
  dReal r, h;
  dsSetColorAlpha(1.3, 1.3, 1.3, 0.5);
  //dsSetColor(0, 0, 1);
  dGeomCylinderGetParams(camera.geom, &r, &h);
  dsDrawCylinder(dGeomGetPosition(camera.geom),
             dGeomGetRotation(camera.geom),h, r);
}

void drawWheel()
{
  dReal radius, length;
  dsSetColor(1.2,1.2,1.2);
  for (int i=0; i< WHEEL_NUM; i++) {
    dGeomCylinderGetParams(wheel[i].geom, &radius, &length);
    dsDrawCylinder(dGeomGetPosition(wheel[i].geom),
             dGeomGetRotation(wheel[i].geom),length, radius);
  }
}

void drawBall(dGeomID gID)
{
  if (gID == ball.geom) dsSetColor(1.0,0.0,0.0);
  else                  dsSetColorAlpha(1.1,0.0,0.0,0.4);
  dsDrawSphere(dGeomGetPosition(gID),
	  dGeomGetRotation(gID),dGeomSphereGetRadius(gID));
}

static void drawGoal()
{
  dsSetTexture(DS_NONE);
  for (int i = 0; i < GOAL_PARTS_NUM; i++) {
    if (i < 4)   dsSetColor(1.3, 1.3, 1.3);
    else         dsSetColor(1.3, 1.3, 0.0);
    dsDrawBox(dGeomGetPosition(goal_parts[i]),
	      dGeomGetRotation(goal_parts[i]),GOAL_SIDES[i]);

    if (i < 4)   dsSetColor(1.3, 1.3, 1.3);
    else         dsSetColor(0.0, 0.0, 1.3);
    dsDrawBox(dGeomGetPosition(goal_parts2[i]),
	      dGeomGetRotation(goal_parts2[i]),GOAL_SIDES[i]);
  }
}

static void drawCircle(dReal r,dReal center[2], int angle1, int angle2)
{
  dReal pos1[3],pos2[3],z=offset_z + 0.005;
	pos1[0] = r * cos(angle1 * M_PI/180.0) + center[0];
  pos1[1] = r * sin(angle1 * M_PI/180.0) + center[1];
  pos1[2] = z;
  for (int i = angle1; i < angle2; i++) {
	  pos2[0] = r* cos(i * M_PI/180.0) + center[0];
    pos2[1] = r* sin(i * M_PI/180.0) + center[1];
    pos2[2] = z;
	  dsDrawLine(pos1,pos2);
    pos1[0] = pos2[0]; pos1[1] = pos2[1]; pos1[2] = pos2[2];
  }
}

static void drawLine()
{
	dReal z = offset_z + 0.005;
  dReal center_r = 1.0, corner_r = 0.5;
  dReal penalty_mark_r = 0.03; // 0.0625;
  dReal pos[][3] = {{6.0, 4.0, z},{6.0, -4.0, z},
	{-6.0, -4.0, z},{-6.0, 4.0, z},{0.0,  4.0, z},
	{0.0,  -4.0, z},
  {6.0,   1.5, z},{ 5.5, 1.5, z},{5.5, -1.5, z}, {6.0, -1.5, z},
  {-6.0,  1.5, z},{- 5.5,1.5, z},{-5.5,-1.5, z}, {-6.0,-1.5, z},
  {6.0,   2.0, z},{ 4.5, 2.0, z},{4.5, -2.0, z}, {6.0, -2.0, z},
  {-6.0,  2.0, z},{-4.5, 2.0, z},{-4.5,-2.0, z}, {-6.0,-2.0, z},};
	dsSetColor(1.3,1.3,1.3);
	dsDrawLine(pos[0],pos[1]);	dsDrawLine(pos[1],pos[2]);
	dsDrawLine(pos[2],pos[3]);	dsDrawLine(pos[3],pos[0]);
  dsDrawLine(pos[4],pos[5]);
	dsDrawLine(pos[6],pos[7]); dsDrawLine(pos[7],pos[8]);dsDrawLine(pos[8],pos[9]);
	dsDrawLine(pos[10],pos[11]); dsDrawLine(pos[11],pos[12]);dsDrawLine(pos[12],pos[13]);
	dsDrawLine(pos[14],pos[15]); dsDrawLine(pos[15],pos[16]);dsDrawLine(pos[16],pos[17]);
  dsDrawLine(pos[18],pos[19]); dsDrawLine(pos[19],pos[20]);dsDrawLine(pos[20],pos[21]);

  // center circle
	dReal pos_center[2] = {0,0};
	dReal pos_penalty_mark[2][2] = {{-4,0},{4,0}};
  dReal pos_corner[4][2] = {{-6, -4},{-6,4},{6,4},{6,-4}};
  drawCircle(center_r,pos_center, 0, 360);
	drawCircle(corner_r,pos_corner[0],0,90);
  drawCircle(corner_r,pos_corner[1],-90,0);
	drawCircle(corner_r,pos_corner[3],90,180);
	drawCircle(corner_r,pos_corner[2],-180,-90);
  for (dReal i = 0; i < penalty_mark_r; i += 0.0025) {
    drawCircle(i,pos_penalty_mark[0],0,360);
    drawCircle(i,pos_penalty_mark[1],0,360);
  }
}

static void start()
{
  float xyz[3] = { -3, 0.0, 0.77};
  float hpr[3] = { -180, -14.5, 0.0};
  dsSetViewpoint(xyz,hpr);
	dsSetSphereQuality(3);
}

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  dVector3 tmp_fdir = {0, 0, 0, 0};
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;

  int wheel_flag = 0;
  for (int j = 0; j < WHEEL_NUM; j++) {
	   if ((o1 == wheel[j].geom)||(o2 == wheel[j].geom)) {
				wheel_flag = 1;
        dJointGetHingeAxis(wheel[j].joint,tmp_fdir);
		    break;
     }
  }

  static const int N = 10;
  dContact contact[N];
  int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
	  if (wheel_flag == 1) {
    	for (int i=0; i<n; i++) {
        contact[i].surface.mode = dContactSlip1 | dContactSlip2 |dContactFDir1|
   	         dContactMu2 | dContactSoftERP | dContactSoftCFM;
   			contact[i].fdir1[0] = tmp_fdir[0];
        contact[i].fdir1[1] = tmp_fdir[1];
        contact[i].fdir1[2] = tmp_fdir[2];
    		contact[i].surface.mu =  0.1;          // タイヤの軸方向の摩擦係数
        contact[i].surface.mu2 = dInfinity;    // タイヤの方向の摩擦係数
        contact[i].surface.slip1 = 0.7;        // 軸方向のスリップ
        contact[i].surface.slip2 = 0.01;       // タイヤ方向のスリップ
        contact[i].surface.soft_erp = 0.9;
        contact[i].surface.soft_cfm = 0.0001;

		    dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
      	dJointAttach(c,b1,b2);

  	  }
    }
    else {
      for (int i=0; i<n; i++) {
        contact[i].surface.mode = dContactSoftERP | dContactSoftCFM;
   	    contact[i].surface.mu  = 1.0;
        contact[i].surface.bounce = 0.5;    // 反発係数
        contact[i].surface.bounce_vel = 0.05;
  	    contact[i].surface.soft_erp = 0.8;
   	    contact[i].surface.soft_cfm = 1e-5;
        dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
    	  dJointAttach(c,b1,b2);
	 	  }
    }
  }
}

static void control()
{
  dReal fMax  = 100;

  for (int i = 0; i < WHEEL_NUM; i++) {
 	  dJointSetHingeParam(wheel[i].joint, dParamVel , wheel[i].v);
    dJointSetHingeParam(wheel[i].joint, dParamFMax, fMax);
  }
}

static void command(int cmd)
{
	float speed_y;

  switch (cmd) {
	case '0':
    float xyz1[3],hpr1[3];
    dsGetViewpoint(xyz1,hpr1);
    printf("\n");
    while (1) {
      printf("xyz=%4.2f %4.2f %4.2f ",xyz1[0],xyz1[1],xyz1[2]);
      printf("hpr=%6.2f %6.2f %5.2f \r",hpr1[0],hpr1[1],hpr1[2]);
    }
    break;
	case '+':  if (POWER >= 100) POWER = 100;
			   else              POWER++;break;
 	case '-':  if (POWER <=   0) POWER =  0;
			   else              POWER--;break;
	case 'r':speed_y = 2.0 * ((dReal) rand()/ 32768) - 1; // RAND_MAX = 32768
					 dBodySetLinearVel(ball.body,-6.0,3.0 * speed_y,0.0);
           PAUSE = 0;
           break;
	case 'l': dBodySetLinearVel(ball.body,-6.0,2.5,3.0);
           PAUSE = 0;
           break;
	case 'p': dBodySetLinearVel(ball.body,-6.0,3.0,0);
           PAUSE = 0;
           break;
	case 'o': dBodySetLinearVel(ball.body,-6.0,-3.0,0);
           PAUSE = 0;
           break;
	case 'b': dBodySetPosition(ball.body,-4.0,0.0,0.14+offset_z);
	       dBodySetLinearVel(ball.body,0,0,0);
		   dBodySetAngularVel(ball.body,0,0,0);
           break;
  }
}

static void simLoop(int pause)
{

  if (!pause) {
		dSpaceCollide(space,0,&nearCallback);

    if (!PAUSE) {
      // ここにキーパーの処理を書いてください．
    }
  	dWorldStep(world, 0.01);
  	dJointGroupEmpty(contactgroup);
  }
  drawBase();
  drawBase2();
  drawBase3();
  drawCamera();
  drawWheel();
  drawBall(ball.geom);
  drawGoal();
	drawLine();
}

static void setDrawStuff()
{
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = &command;
  fn.path_to_textures = "../textures";
}

void makeOmni()
{
  makeBase();
  makeBase2();
  makeBase3();
  makeCamera();
	makeWheel();
}


int main(int argc, char *argv[])
{
  PAUSE = 1;

  dInitODE();
  setDrawStuff();
  world = dWorldCreate();

  space        = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  ground       = dCreatePlane(space,0,0,1,0);
  dWorldSetGravity(world, 0, 0, - 9.8);
  dWorldSetCFM(world,1e-3);
  dWorldSetERP(world,0.9);

  makeOmni();
  makeBall();
  makeGoal();
	srand(time(NULL));
  dsSimulationLoop(argc,argv,640, 480,&fn);

  dJointGroupDestroy(contactgroup);
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();
  return 0;
}
