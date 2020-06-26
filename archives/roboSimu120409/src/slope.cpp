// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// プログラム 3.3: 斜面 slope.cpp by Kosei Demura (2007-5-17)
//
// This program is a sample program of my book as follows
//“Robot Simulation - Robot programming with Open Dynamics Engine,
// (260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd.,
// Tokyo, 2007)” by Kosei Demura, which is written in Japanese (sorry).
// http://demura.net/simulation
// Please use this program if you like. However it is no warranty.
// slope.cpp by Kosei Demura (2007-5-18)
//
// 更新履歴　(change log)
// 2008-7-7: dInitODE(),dCloseODE()の追加
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#endif

static dWorldID world;
static dSpaceID space;
static dGeomID  ground;
static dGeomID  slope;
static dJointGroupID contactgroup;
dsFunctions fn;

typedef struct {
  dBodyID body;
  dGeomID geom;
  dReal r, m;
} MyObject;
MyObject ball;

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  const int N = 10;
  dContact contact[N];

  int isGround = ((ground == o1 || slope == o1)
		           || (ground == o2 || slope == o2));

  int n =  dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
  if (isGround)  {
    for (int i = 0; i < n; i++) {
      contact[i].surface.mode = dContactSoftERP|dContactSoftCFM;
      contact[i].surface.soft_erp = 1.0;
      contact[i].surface.soft_cfm = 0.0;
      dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
      dJointAttach(c,dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
    }
  }
}

static void simLoop(int pause)
{
  const dReal *pos,*R;

  fn.path_to_textures = "../../drawstuff/textures";

  if (!pause) {
    dSpaceCollide(space,0,&nearCallback);
    dWorldStep(world, 0.01);
    dJointGroupEmpty(contactgroup);
  }

  dsSetColor(1.0, 0.0, 0.0);
  pos = dBodyGetPosition(ball.body);
  R   = dBodyGetRotation(ball.body);
  dsDrawSphere(pos, R, ball.r);

  dsSetColor(1.2, 1.2, 0.0);
  pos = dGeomGetPosition(slope);
  R   = dGeomGetRotation(slope);
  dVector3 sides;
  dGeomBoxGetLengths(slope, sides);
  dsDrawBox(pos, R, sides);
}

void start()
{
  static float xyz[3] = {   3.0, 0.0, 1.0};
  static float hpr[3] = {-180.0, 0.0, 0.0};
  dsSetViewpoint(xyz, hpr);
  dsSetSphereQuality(3);
}

void  setDrawStuff()
{
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = NULL;
  fn.stop    = NULL;
  fn.path_to_textures = "../../drawstuff/textures";
}

// ボールの生成
void makeBall()
{
  dMass m1;
  dReal x0 = -10.0, y0 = 0.0, z0 = 2.2;

  ball.r = 0.2;
  ball.m = 1.0;
  ball.body = dBodyCreate(world);
  dMassSetZero(&m1);
  dMassSetSphereTotal(&m1, ball.m, ball.r);
  dBodySetMass(ball.body, &m1);
  dBodySetPosition(ball.body, x0, y0, z0);

  ball.geom = dCreateSphere(space, ball.r);
  dGeomSetBody(ball.geom, ball.body);
}

// 斜面の生成
void makeSlope()
{
  dReal lx = 100, ly = 2, lz = 0.01;         // 絶対座標系x,y,z軸に沿った長さ
  dReal  x = 0,    y = 0,  z = 0;            // 重心の座標
  dReal ax = 0,   ay = 1, az = 0;            // 回転軸ベクトル
  dReal angle = 10.0 * M_PI / 180.0;         // 回転角

  slope  = dCreateBox(space, lx, ly, lz);    // 直方体ジオメトリの生成

  dMatrix3 R;
  dRFromAxisAndAngle(R, ax, ay, az, angle);
  dGeomSetPosition(slope, x, y, z);          // 位置の設定
  dGeomSetRotation(slope, R);                // 姿勢の設定
}

int main(int argc, char *argv[])
{
  dInitODE();
  setDrawStuff();

  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  dWorldSetGravity(world,0,0,-9.8);

  ground = dCreatePlane(space,0,0,1,0);

  makeSlope();
  makeBall();

  dsSimulationLoop(argc,argv,640,480,&fn);

  dWorldDestroy(world);
  dCloseODE();
  return 0;
}
