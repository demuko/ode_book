// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// プログラム 3.5: Aモータ amotor.cpp by Kosei Demura (2007-5-17)
// 2008-7-7: dInitODE(),dCloseODE()の追加
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox    dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCapsule dsDrawCapsuleD
#endif

dsFunctions fn;

static dReal ALPHA = -45, BETA = 30, GAMMA = -45; // オイラー角x,y,z
static int STEPS = 0;
static dWorldID world;
static dBodyID  body[2],axis[3];
static dJointID fixed[4],joint,motor;
static dReal side[3] = {0.15, 0.25, 0.5}; // ボックスの辺長さ(x,y,z軸）


static void control(dReal target, int axis_num)
{
  double kp = 10.0, fmax = 100.0;

  dReal tmp = dJointGetAMotorAngle(motor,axis_num);
  dReal u   = kp * (target - tmp);
  switch (axis_num) {
  case 0:dJointSetAMotorParam(motor,   dParamVel, u   );
         dJointSetAMotorParam(motor,  dParamFMax, fmax); break;
  case 1:dJointSetAMotorParam(motor,  dParamVel2, u   );
         dJointSetAMotorParam(motor, dParamFMax2, fmax); break;
  case 2:dJointSetAMotorParam(motor,  dParamVel3, u   );
         dJointSetAMotorParam(motor, dParamFMax3, fmax); break;
  }

  // オイラー角の取得
  dReal e_angle = dJointGetAMotorAngle(motor,axis_num);
  printf("%.4f ",e_angle*180.0/M_PI);
}

// シミュレーションループ
static void simLoop(int pause)
{
  STEPS++;
  if (!pause) {
    printf("\nSTEPS:%6d ",STEPS);
    control(ALPHA * M_PI/180.0, 0);
    control(BETA  * M_PI/180.0, 1);
    control(GAMMA * M_PI/180.0, 2);
    dWorldStep(world,0.01);
  }

  // 物体の描画
  dsSetColor(1.0,1.0,0);
  dsDrawBox(dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),side);
  dsSetColor(0,0,1);
  dsDrawBox(dBodyGetPosition(body[1]),dBodyGetRotation(body[1]),side);

  dsSetColor(0,0,0);
  dsDrawCapsule(dBodyGetPosition(axis[0]),
                dBodyGetRotation(axis[0]), 1.0, 0.008);
  dsDrawCapsule(dBodyGetPosition(axis[1]),
                dBodyGetRotation(axis[1]), 1.0, 0.008);
  dsDrawCapsule(dBodyGetPosition(axis[2]),
                dBodyGetRotation(axis[2]), 1.0, 0.008);
}

void command(int cmd)
{
  switch(cmd) {
    case 'f': ALPHA++; break;  case 'j': ALPHA--; break;
    case 'd': BETA++;  break;  case 'k': BETA--;  break;
    case 's': GAMMA++; break;  case 'l': GAMMA--; break;
  }
}

static void start()
{
  static float xyz[3] = {  2.0, 0.1, 0.7};
  static float hpr[3] = {175.0, 0.5, 0.0};
  dsSetViewpoint (xyz,hpr);
}

static void  setDrawStuff() {
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = &command;
  fn.stop    = NULL;
  fn.path_to_textures = "../../drawstuff/textures";
}

int main(int argc, char **argv)
{
  dReal anchor_x = 0.0, anchor_y = 0.0, anchor_z = 0.5, m = 1;
  dReal px[2]      = {0.0,  0.0};      // ボックスの位置
  dReal py[2]      = {0.0,  0.0};
  dReal pz[2]      = {0.25, 1.0};
  dReal anch_x[3]  = {0.0, 0.5, 0.0};  // 軸の位置
  dReal anch_y[3]  = {0.0, 0.0, 0.5};
  dReal anch_z[3]  = {1.0, 0.5, 0.5};
  dReal axis_x[3]  = {0.0, 0.0, 1.0};  // 軸の向き
  dReal axis_y[3]  = {0.0, 1.0, 0.0};
  dReal axis_z[3]  = {1.0, 0.0, 0.0};
  dReal angle[3]   = {0, M_PI/2, M_PI/2};

  dInitODE();
  setDrawStuff();
  world = dWorldCreate();
  dWorldSetGravity(world,0,0,-9.8);

  dMass mass;
  dMassSetBoxTotal(&mass, m, side[0], side[1], side[2]);

  // ボックスの生成
  for (int i=0; i< 2;i++) {
    body[i] = dBodyCreate(world);
    dBodySetMass(body[i],&mass);
    dBodySetPosition(body[i], px[i], py[i], pz[i]);
    dQuaternion q;
    dQFromAxisAndAngle(q,0,0,1,0);
    dBodySetQuaternion(body[i],q);
  }

  // 軸表示用物体の生成
  for (int i=0; i< 3;i++) {
    axis[i] = dBodyCreate(world);
    dBodySetMass(axis[i],&mass);
    dBodySetPosition(axis[i], anch_x[i], anch_y[i], anch_z[i]);
    dQuaternion q;
    dQFromAxisAndAngle(q,axis_x[i],axis_y[i],axis_z[i],angle[i]);
    dBodySetQuaternion(axis[i],q);
  }

  // 静的環境とベースの結合
  fixed[0] = dJointCreateFixed(world, 0);
  dJointAttach(fixed[0], body[0], 0);
  dJointSetFixed(fixed[0]);

  // 軸表示用物体とボックスとの結合
  for (int i = 1; i < 4; i++) {
    fixed[i] = dJointCreateFixed(world, 0);
    dJointAttach(fixed[i], body[1], axis[i-1]);
    dJointSetFixed(fixed[i]);
  }

  // ボールジョイントの生成と設定
  joint       = dJointCreateBall(world,0);
  dJointAttach(joint,body[0],body[1]);
  dJointSetBallAnchor(joint,anchor_x,anchor_y,anchor_z);

  // Aモータの生成と設定
  motor  = dJointCreateAMotor(world,0);
  dJointAttach(motor,body[0],body[1]);
  dJointSetAMotorNumAxes(motor,3);
  dJointSetAMotorAxis(motor,0,1,0,0,1);
  dJointSetAMotorAxis(motor,1,2,0,1,0);
  dJointSetAMotorAxis(motor,2,2,1,0,0);
  dJointSetAMotorMode(motor,dAMotorEuler);

  dsSimulationLoop(argc,argv,640,480,&fn);

  dWorldDestroy(world);
  dCloseODE();
  return 0;
}


