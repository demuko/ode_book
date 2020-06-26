// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// プログラム 9.1:  力・トルクセンサ
// sensor.cpp by Kosei Demura (2007-5-19)
//
// This program is a sample program of my book as follows
//“Robot Simulation - Robot programming with Open Dynamics Engine,
// (260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd.,
// Tokyo, 2007)” by Kosei Demura, which is written in Japanese (sorry).
// http://demura.net/simulation
// Please use this program if you like. However it is no warranty.
// sensor4.cpp by Kosei Demura (2007-5-18)
//
// 更新履歴　(change log)
// 2008-7-7: dInitODE(),dCloseODE()の追加
#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawBox     dsDrawBoxD
#define dsDrawLine    dsDrawLineD
#endif

#define NUM 4                  // センサ数

dWorldID      world;           // 動力学計算用のワールド
dSpaceID      space;           // 衝突検出用のスペース
dGeomID       ground;          // 地面
dJointID fixed[NUM];           // センサ用固定ジョイント
dJointGroupID contactgroup;    // 接触点グループ
dsFunctions   fn;              // ドロースタッフの描画関数
dJointFeedback feedback[NUM];  // フィードバック構造体

typedef struct {
  dBodyID body;
  dGeomID geom;
} MyLink;
MyLink box,sensor[NUM];        // ボックス, センサ

static dReal SX = 0, SY = 0, SZ = 0.1; // ボックスの初期位置
static dReal box_l = 0.25, box_w  = 0.15,box_h = 0.05,box_m = 20.0;

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  static const int MAX_CONTACTS = 10;
  int i;

  // ２つのボディがジョイントで結合されていたら何もしないで戻る
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnected (b1,b2)) return;

  dContact contact[MAX_CONTACTS];
  int numc = dCollide(o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact));
  if (numc > 0) {
    for (i=0; i<numc; i++) {
      if ((o1 == ground) || (o2 == ground)) {
        dVector3 End;
        dReal kl = 500000.0;
        End[0] = contact[i].geom.pos[0] + (contact[i].geom.normal[0] * contact[i].geom.depth*kl);
        End[1] = contact[i].geom.pos[1] + (contact[i].geom.normal[1] * contact[i].geom.depth*kl);
        End[2] = contact[i].geom.pos[2] + (contact[i].geom.normal[2] * contact[i].geom.depth*kl);
        dsDrawLine(contact[i].geom.pos,End);
      }

      contact[i].surface.mode  =  dContactSoftCFM | dContactSoftERP;
      contact[i].surface.mu       = dInfinity;
      contact[i].surface.soft_cfm = 1e-8;
      contact[i].surface.soft_erp = 1.0;
      dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
      dJointAttach (c,dGeomGetBody(contact[i].geom.g1),
		      dGeomGetBody(contact[i].geom.g2));
    }
  }
}

void drawObjects()
{
 // 直方体（足のつもり）の描画
  dsSetColorAlpha(1.3, 1.3, 1.3,1.0);
  dVector3 sides1;
  dGeomBoxGetLengths(box.geom,sides1);
  dsDrawBoxD(dBodyGetPosition(box.body),
	     dBodyGetRotation(box.body),sides1);

  // センサの描画
  dsSetColor(1.0,0.0,0.0);
  dVector3 sides2;
  for (int i = 0; i < NUM; i++) {
    dGeomBoxGetLengths(sensor[i].geom,sides2);
    dsDrawBoxD(dBodyGetPosition(sensor[i].body),
	      dBodyGetRotation(sensor[i].body),sides2);
  }
}

static void simLoop(int pause)
{
  dSpaceCollide(space,0,&nearCallback);
  dWorldStep(world,0.01);
  dJointGroupEmpty(contactgroup);

  dJointFeedback *feedback;
  dReal fx[NUM],fy[NUM],fz[NUM],tx[NUM],ty[NUM],tz[NUM];
  for (int i = 0; i < NUM; i++) {
    feedback = dJointGetFeedback(fixed[i]); // フィードバックの取得
    fx[i] = feedback->f1[0];  // 力 x成分
    fy[i] = feedback->f1[1];  // 力 y成分
    fz[i] = feedback->f1[2];  // 力 z成分
    tx[i] = feedback->t1[0];  // トルク x軸まわり
    ty[i] = feedback->t1[1];  // トルク y軸まわり
    tz[i] = feedback->t1[2];  // トルク z軸まわり
    printf("Force fx=%6.2f fy=%6.2f fz=%6.2f \n ",fx[i],fy[i],fz[i]);
  }

  // 力センサの値に比例した直線を表示する
  dVector3 endP;    // 線の終点
  dReal k1 = 0.01;  // 比例定数，これを変更すると線の長さが変わる
  for (int i = 0; i < NUM; i++) {
     const dReal *pos = dBodyGetPosition(sensor[i].body);
     endP[0] = pos[0];  endP[1] = pos[1];
     endP[2] = k1 * (fz[i] + pos[2]);
     dsDrawLine(pos, endP); // posからendPまでの直線を描画
  }

  drawObjects();  // センサや足の描画
}

/*** センサの生成と取り付け ***/
void makeForceSensor()
{
	dReal sensor_l = 0.01, sensor_w = 0.01;
  dReal sensor_h = 0.01, sensor_m = 0.01;
  for (int i = 0; i < NUM; i++) {
    sensor[i].body   = dBodyCreate(world);
    dMass mass;
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,sensor_m,sensor_l,sensor_w,sensor_h);
    dBodySetMass(sensor[i].body,&mass);
    dReal x =  (2 * (int) (i/2) - 1) * (box_l - sensor_l)/2 + SX;
    dReal y =  (2 * (int) (i%2) - 1) * (box_w - sensor_w)/2 + SY;
    dBodySetPosition(sensor[i].body, x, y, SZ - (box_h+sensor_h)/2);
    sensor[i].geom = dCreateBox(space,sensor_l,sensor_w,sensor_h);
    dGeomSetBody(sensor[i].geom,sensor[i].body);

    // 固定ジョイント（センサと足の固定）
    fixed[i] = dJointCreateFixed(world,0);
    dJointAttach(fixed[i],box.body,sensor[i].body);
    dJointSetFixed(fixed[i]);

    dJointSetFeedback(fixed[i],&feedback[i]);   // Feedbackの設定
  }
}

void start()
{
  static float xyz[3] = { 0.0f,-3.0f,1.0f};
  static float hpr[3] = {90.0f, 0.0f,0.0f};
  dsSetViewpoint (xyz,hpr);
}

void  setDrawStuff() {
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = NULL;
  fn.stop    = NULL;
  fn.path_to_textures = "../../drawstuff/textures";
}

int main (int argc, char *argv[])
{
  dInitODE();
  setDrawStuff();
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  dWorldSetGravity(world,0,0,-9.8);
  ground = dCreatePlane(space,0,0,1,0);

  // 直方体(足）
  dMass mass;
  box.body   = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,box_m,box_l,box_w,box_h);
  dBodySetMass(box.body,&mass);
  dBodySetPosition(box.body,SX,SY,SZ);
  box.geom = dCreateBox(space,box_l,box_w,box_h);
  dGeomSetBody(box.geom,box.body);

  makeForceSensor(); // センサの生成

  dsSimulationLoop(argc,argv,800, 600,&fn);
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();
  return 0;
}
