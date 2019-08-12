// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// プログラム 4.1:  差動駆動型ロボット wheel1.cpp by Kosei Demura (2007-5-17)
//
// This program is a sample program of my book as follows
//“Robot Simulation - Robot programming with Open Dynamics Engine,
// (260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd.,
// Tokyo, 2007)” by Kosei Demura, which is written in Japanese (sorry).
// http://demura.net/simulation
// Please use this program if you like. However it is no warranty.
// wheel1.cpp by Kosei Demura (2007-5-18)
//
// 更新履歴　(change log)
// 2008-7-7: dInitODE(),dCloseODE()の追加
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#endif

#define NUM 4

dWorldID world;  // 動力学計算用ワールド
dSpaceID space;  // 衝突検出用スペース
dGeomID  ground; // 地面
dJointGroupID contactgroup;
dsFunctions fn;

typedef struct {       // MyObject構造体
  dBodyID body;        // ボディ(剛体)のID番号（動力学計算用）
  dGeomID geom;        // ジオメトリのID番号(衝突検出計算用）
  double  l,r,m;       // 長さ[m], 半径[m]，質量[kg]
} MyObject;

MyObject wheel[NUM], base, ball;
dJointID joint[NUM];
static const dReal  BALL_R    = 0.11;
static const dReal  BALL_P[3] = {1.0, 1.0, 0.14};
dReal v_r = 0.0,v_l = 0.0 ;                   // 右,左車輪の回転速度
dReal BASE_M    = 9.4;                        // 台車の質量
dReal BASE_S[3] = {0.4,0.4,0.2};              // 台車のサイズ
dReal WH_R0 = 0.05, WH_R1 = 0.101;            // 車輪0,1の半径
dReal WH_W  = 0.02, WH_M0 = 0.1, WH_M1 = 0.2; // 車輪の幅,質量
dReal START_X = 0, START_Y = 0, START_Z=0.20; // 初期位置

static void makeBall() {
  dMass m1;

  dReal  ball_mass   = 0.45;
  ball.body    = dBodyCreate(world);

  dMassSetZero(&m1);
  dMassSetSphereTotal(&m1,ball_mass,BALL_R);
  dMassAdjust(&m1,ball_mass);
  dBodySetMass(ball.body,&m1);

  ball.geom = dCreateSphere(space,BALL_R);
  dGeomSetBody(ball.geom,ball.body);
  dBodySetPosition(ball.body,BALL_P[0],BALL_P[1],BALL_P[2]);
}

static void makeRobot()
{
	dMatrix3 R;
  dMass mass;

  // 台車
  base.body  = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,BASE_M,BASE_S[0],BASE_S[1],BASE_S[2]);
  dBodySetMass(base.body,&mass);

  base.geom = dCreateBox(space,BASE_S[0],BASE_S[1],BASE_S[2]);
  dGeomSetBody(base.geom,base.body);
  dBodySetPosition(base.body,START_X,START_Y,START_Z);

  // 車輪
  dRFromAxisAndAngle(R,0,1,0,M_PI/2.0);
  for (int i = 0; i < NUM; i++) {
    wheel[i].body = dBodyCreate(world);

    dMassSetZero(&mass);
    if ((i % 2) == 0) {
      dMassSetCylinderTotal(&mass,WH_M0, 2, WH_R0, WH_W);
      dBodySetMass(wheel[i].body,&mass);
      wheel[i].geom = dCreateCylinder(space, WH_R0, WH_W);
    }
    else {
      dMassSetCylinderTotal(&mass,WH_M1, 2, WH_R1, WH_W);
      dBodySetMass(wheel[i].body,&mass);
      wheel[i].geom = dCreateCylinder(space, WH_R1, WH_W);
    }
    dGeomSetBody(wheel[i].geom, wheel[i].body);
    dBodySetRotation(wheel[i].body,R);
  }

  dReal w1x = 0.5 * (BASE_S[1]+WH_W);
  dReal w1z = START_Z - 0.5 * BASE_S[2];
  dReal w0y = 0.5 * BASE_S[1] - WH_R0;
  dReal w0z = START_Z - 0.5 * BASE_S[2] - WH_R0;

  dBodySetPosition(wheel[0].body,    0,  w0y, w0z);
  dBodySetPosition(wheel[1].body, -w1x,    0, w1z);
  dBodySetPosition(wheel[2].body,    0, -w0y, w0z);
  dBodySetPosition(wheel[3].body,  w1x,    0, w1z);

  // ヒンジジョイント
  for (int i = 0; i < NUM; i++) {
    joint[i] = dJointCreateHinge(world,0);
    dJointAttach(joint[i], base.body, wheel[i].body);
  }
  dJointSetHingeAxis(joint[0],1, 0, 0);
  dJointSetHingeAxis(joint[1],1, 0, 0);
  dJointSetHingeAxis(joint[2],1, 0, 0);
  dJointSetHingeAxis(joint[3],1, 0, 0);
  dJointSetHingeAnchor(joint[0],    0,  w0y, w0z);
  dJointSetHingeAnchor(joint[1], -w1x,    0, w1z);
  dJointSetHingeAnchor(joint[2],    0, -w0y, w0z);
  dJointSetHingeAnchor(joint[3],  w1x,    0, w1z);
}

static void command(int cmd)
{
  switch (cmd) {
  case 'j': v_r += 0.1; break;     // 右車輪,前進
  case 'f': v_l += 0.1; break;     // 左車輪,前進
  case 'k': v_r -= 0.1; break;     // 右車輪,後進
  case 'd': v_l -= 0.1; break;     // 左車輪,後進
  case 's': v_r = v_l = 0.0;break; // 停  止
  default: printf("\nInput j,f,k,d,l,s \n");break;
  }
}

static void control() {
  double fMax = 100;         // 最大発揮トルク[Nm]

  dJointSetHingeParam(joint[1],  dParamVel, v_l );
  dJointSetHingeParam(joint[1], dParamFMax, fMax);
  dJointSetHingeParam(joint[3],  dParamVel, v_r );
  dJointSetHingeParam(joint[3], dParamFMax, fMax);
}

static void start()
{
  float xyz[3] = {  0.0f,   0.0f, 3.0f};
  float hpr[3] = { 90.0f, -90.0f, 0.0f};

  dsSetViewpoint(xyz,hpr);
  dsSetSphereQuality(2);
}

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i,n;

  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;

  static const int N = 10;
  dContact contact[N];
  n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
        dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu       = dInfinity;
      contact[i].surface.slip1    = 0.1;
      contact[i].surface.slip2    = 0.1;
      contact[i].surface.soft_erp = 0.8;
      contact[i].surface.soft_cfm = 1e-5;
      dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
      dJointAttach(c,b1,b2);
    }
  }
}

static void drawBall()
{
  dsSetColor(1.0,0.0,0.0);
  dsDrawSphere(dGeomGetPosition(ball.geom),
	       dGeomGetRotation(ball.geom),BALL_R);
}

static void drawRobot()
{
  dsSetColor(1.3,1.3,0.0);
  dsDrawBox(dGeomGetPosition(base.geom),
	    dGeomGetRotation(base.geom),BASE_S);

  dsSetColor(1.1,1.1,1.1);
  for (int i=0; i< NUM; i++) {
		if ((i % 2) == 0) {
    	dsDrawCylinder(dBodyGetPosition(wheel[i].body),
             dBodyGetRotation(wheel[i].body),WH_W,WH_R0);
  	}
  	else {
    	dsDrawCylinder(dBodyGetPosition(wheel[i].body),
             dBodyGetRotation(wheel[i].body),WH_W,WH_R1);
		}
  }
}

static void simLoop(int pause)
{
  control();
  dSpaceCollide(space,0,&nearCallback);
  dWorldStep(world, 0.01);
  dJointGroupEmpty(contactgroup);
  drawBall();
  drawRobot();
}

static void setDrawStuff() {
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = &command;
  fn.path_to_textures = "../../drawstuff/textures";
}

int main(int argc, char *argv[])
{
  dInitODE();
  setDrawStuff();

  world        = dWorldCreate();
  space        = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  ground       = dCreatePlane(space,0,0,1,0);

  dWorldSetGravity(world, 0, 0, -9.8);

  makeRobot();
  makeBall();

  dsSimulationLoop(argc,argv,640,480,&fn);

  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();
  return 0;
}
