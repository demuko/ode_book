// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// プログラム 4.3:  差動駆動型ロボット(センサ搭載）wheel2.cpp by Kosei Demura (2007-5-18)
//
// This program is a sample program of my book as follows
//“Robot Simulation - Robot programming with Open Dynamics Engine,
// (260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd.,
// Tokyo, 2007)” by Kosei Demura, which is written in Japanese (sorry).
// http://demura.net/simulation
// Please use this program if you like. However it is no warranty.
// wheel4.cpp by Kosei Demura (2007-5-18)
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

MyObject wheel[NUM], base, neck, camera, ball;
dJointID joint[NUM], neck_joint, camera_joint;
static const dReal  BALL_R    = 0.11;
static const dReal  BALL_P[3] = {1.0, 1.0, 0.14};
static dReal v_r = 0.0,v_l = 0.0 ;                   // 右,左車輪の回転速度
static dReal BASE_M    = 9.4;                        // 台車の質量
static dReal BASE_S[3] = {0.4,0.4,0.1};              // 台車のサイズ
static const dReal  CAMERA_M  = 0.2;
static const dReal  CAMERA_S[3]  = {0.2,0.2,0.05};
static const dReal  NECK_M    = 0.5;
static const dReal  NECK_L    = 0.2;
static const dReal  NECK_R    = 0.025;
static dReal WH_R0 = 0.05, WH_R1 = 0.105;            // 車輪0,1の半径
static dReal WH_W  = 0.02, WH_M0 = 0.1, WH_M1 = 0.2; // 車輪の幅,質量
static dReal START_X = 0, START_Y = 0, START_Z=0.20; // 初期位置
static dReal s_x = 0.0, s_y = 0.0;                   // デッドレコニングの距離

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
  dMass mass, mass2, mass3;

  // 台車
  base.body  = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,BASE_M,BASE_S[0],BASE_S[1],BASE_S[2]);
  dBodySetMass(base.body,&mass);

  base.geom = dCreateBox(space,BASE_S[0],BASE_S[1],BASE_S[2]);
  dGeomSetBody(base.geom,base.body);
  dBodySetPosition(base.body,START_X,START_Y,START_Z);

	// 首
  neck.body  = dBodyCreate(world);
  dMassSetZero(&mass2);
  dMassSetCapsuleTotal(&mass2,NECK_M,2,NECK_R,NECK_L);
  dMassAdjust(&mass2,NECK_M);
  dBodySetMass(neck.body,&mass2);

  neck.geom = dCreateCCylinder(space,NECK_R,NECK_L);
  dGeomSetBody(neck.geom,neck.body);
  dBodySetPosition(neck.body,START_X, START_Y+0.5*BASE_S[1]-NECK_R,
		   START_Z+0.5*NECK_L);
  dGeomDisable(neck.geom);

  // カメラ
  camera.body  = dBodyCreate(world);
  dMassSetZero(&mass3);
  dMassSetBoxTotal(&mass3,CAMERA_M,CAMERA_S[0],CAMERA_S[1],CAMERA_S[2]);
  dMassAdjust(&mass3,CAMERA_M);
  dBodySetMass(camera.body,&mass3);

  camera.geom = dCreateBox(space,CAMERA_S[0],CAMERA_S[1],CAMERA_S[2]);
  dGeomSetBody(camera.geom,camera.body);
  dBodySetPosition(camera.body,START_X, START_Y+0.5*BASE_S[1]-0.1*CAMERA_S[1],
		   START_Z+NECK_L);
  dGeomDisable(camera.geom);


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

// joints
  neck_joint = dJointCreateHinge(world,0);
  dJointAttach(neck_joint,neck.body,base.body);
  dJointSetHingeAxis(neck_joint,0, 0, 1);
  dJointSetHingeAnchor(neck_joint, START_X, START_Y+0.5*BASE_S[1]-NECK_R,
		   START_Z);
  dJointSetHingeParam(neck_joint,dParamLoStop, 0);
  dJointSetHingeParam(neck_joint,dParamHiStop, 0);


  // camera joints
  camera_joint = dJointCreateHinge(world,0);
  dJointAttach(camera_joint,neck.body,camera.body);
  dJointSetHingeAxis(camera_joint,1,0,0);
  dJointSetHingeAnchor(camera_joint,START_X, START_Y+0.5*BASE_S[1]-0.1*CAMERA_S[1],
		   START_Z+NECK_L);
  dJointSetHingeParam(camera_joint,dParamLoStop, 0.05 * M_PI);
  dJointSetHingeParam(camera_joint,dParamHiStop, 0.05 * M_PI);


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
     	contact[i].surface.mu       = dInfinity; // 摩擦係数無限大
      contact[i].surface.slip1    = 0.01;      // 第１摩擦方向の滑り
      contact[i].surface.slip2    = 0.01;      // 第２摩擦方向の滑り
      contact[i].surface.soft_erp = 1.0;       // 地面のERP
      contact[i].surface.soft_cfm = 1e-4;      // 地面のCFM
      dJointID c = dJointCreateContact(world,contactgroup,&contact[i]); //接触ジョイントの生成
      dJointAttach(c,b1,b2);                   // ジョイントの結合
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
	// 台車
  dsSetColor(1.3,1.3,0.0);
  dsDrawBox(dGeomGetPosition(base.geom),
	    dGeomGetRotation(base.geom),BASE_S);

  // 首
  dsDrawCylinder(dBodyGetPosition(neck.body),
             dBodyGetRotation(neck.body),NECK_L,NECK_R);

  // カメラ
  dsDrawBox(dGeomGetPosition(camera.geom),
	    dGeomGetRotation(camera.geom),CAMERA_S);

  // 車輪
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

// フィールド座標系からロボット座標系への変換  プログラム4.2
static void field2RobotPos(const dReal p[2], const dReal c[2], dReal theta,
  dReal pr[2], dReal *r, dReal *phi)
{
  pr[0] =   (p[0]-c[0]) * cos(theta) + (p[1]-c[1]) * sin(theta);
  pr[1] = - (p[0]-c[0]) * sin(theta) + (p[1]-c[1]) * cos(theta);

  *r   = sqrt(pr[0] * pr[0] + pr[1] * pr[1]);  // 物体までの距離
  *phi = atan2(pr[0],pr[1]);                   // 物体の角度
}

/*** ロボットのビジョン関数 ***/
static int vision(dReal *r, dReal *phi, dBodyID obj_body)
{
  const dReal *pa,*pb,*pc;
  dReal pr[2], view_angle = M_PI/8.0;  // 相対位置，視野角

  pa  = dBodyGetPosition(obj_body);    // 物体の絶対座標
  pb  = dBodyGetPosition(base.body);   // 台車の絶対座標
  pc  = dBodyGetPosition(camera.body); // カメラの絶対座標

  dReal theta  = atan2(pc[0]-pb[0],pc[1]-pb[1]); // ロボットの針路
  // 絶対座標（右ねじ）と相対座標（左ねじ）で角度の取り方が逆
	theta = - theta;

  field2RobotPos(pa, pb, theta, pr, r, phi);

  if ((-view_angle <= *phi) && (*phi <= view_angle)) return 1;
  else                                               return 0;
}

/*** 車輪の制御 ***/
static void controlWheel(dReal right, dReal left)
{
  double fMax = 100;  // 最大トルク

  dJointSetHingeParam(joint[1],  dParamVel, left);
  dJointSetHingeParam(joint[1], dParamFMax, fMax);
  dJointSetHingeParam(joint[3],  dParamVel, right);
  dJointSetHingeParam(joint[3], dParamFMax, fMax);
}

/***   その場回転  ***/
static void turn(dReal speed)
{
  controlWheel(speed, -speed);
}

/*** 追跡  phi:物体の方位, speed:車輪の回転速度 ***/
static void track(dReal phi, dReal speed)
{
  dReal k = 0.5;
  if (phi > 0) controlWheel(    speed,  k * speed);
  else         controlWheel(k * speed,      speed);
}

/*** ロボットの針路計算 ***/
dReal heading()
{
	const dReal *pb,*pc;

  pb  = dBodyGetPosition(base.body);     // 台車重心の絶対座標
  pc  = dBodyGetPosition(camera.body);   // カメラ重心の絶対座標

  return atan2(pc[0]-pb[0],pc[1]-pb[1]); // ロボットの針路
}

static void simLoop(int pause)
{
  static dReal pose = 0, pose_old = 0; // ロボットの針路
	static dReal omega_r = 0, omega_l = 0, omega_r_old = 0, omega_l_old = 0;
	const dReal *pb;
	dReal r, phi; // r:物体までの距離, phi:物体の方位
	dReal edges;
	dReal step_size = 0.005;

	if (!pause) {
  	dSpaceCollide (space,0,&nearCallback);
  	if (vision(&r, &phi, ball.body)) {
    	printf("tracking phi=%.2f \n",phi * 180/M_PI);
    	track(phi,4.0);
  	}
  	else {
    	printf("lost the ball phi=%.2f \n",phi * 180/M_PI);
    	turn(4.0);
  	}

	dWorldStep(world, step_size);
  	dJointGroupEmpty (contactgroup);

	// 台車重心の位置
	pb  = dBodyGetPosition(base.body);

	// デッドレコニング
	pose    = heading(); // ロボットの針路
	omega_r = dJointGetHingeAngleRate(joint[3]);
	omega_l = dJointGetHingeAngleRate(joint[1]);
	edges   = WH_R1 * (omega_r_old + omega_l_old) * cos(pose_old)/2.0
						+ WH_R1 * (omega_r     + omega_l    ) * cos(pose)/2.0;
	s_y     += 0.5 * step_size * edges;
	printf("Real y = %f  Dead Reckoning s_y = %f Error = %f\n", pb[1], s_y, pb[1]-s_y);
	omega_r_old = omega_r;
	omega_l_old = omega_l;
	pose_old    = pose;
  }

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
