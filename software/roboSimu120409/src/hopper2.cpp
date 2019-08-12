// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// プログラム 2.9:  再実行可能なプログラム hopper2.cpp by Kosei Demura (2007-5-17)
//
// This program is a sample program of my book as follows
//“Robot Simulation - Robot programming with Open Dynamics Engine,
// (260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd.,
// Tokyo, 2007)” by Kosei Demura, which is written in Japanese (sorry).
// http://demura.net/simulation
// Please use this program if you like. However it is no warranty.
// hello2.cpp by Kosei Demura (2007-5-18)
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

dWorldID world;  // 動力学計算用ワールド
dSpaceID space;  // 衝突検出用スペース
dGeomID  ground; // 地面
dJointGroupID contactgroup; // コンタクトグループ
dReal r = 0.2, m  = 1.0;
dsFunctions fn;

typedef struct {       // MyObject構造体
  dBodyID body;        // ボディ(剛体)のID番号（動力学計算用）
  dGeomID geom;        // ジオメトリのID番号(衝突検出計算用）
  double  l,r,m;       // 長さ[m], 半径[m]，質量[kg]
} MyObject;

static MyObject torso,leg[2];    // leg[0]:上脚, leg[1]:下脚
static dJointID h_joint,s_joint; // ヒンジ, スライダー
static int STEPS = 0;            // シミュレーションのステップ数
static dReal S_LENGTH = 0.0;     // スライダー長
static dReal H_ANGLE  = 0.0;     // ヒンジ角

// 衝突検出計算
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int N = 7;     // 接触点数
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  // 2つのボディがジョイントで結合されていたら衝突検出しない
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;

  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (isGround)  {
    for (int i = 0; i < n; i++) {
      contact[i].surface.mode   = dContactBounce | dContactSoftERP |
                                  dContactSoftCFM;
      contact[i].surface.soft_erp   = 0.2;   // 接触点のERP
      contact[i].surface.soft_cfm   = 0.001; // 接触点のCFM
      contact[i].surface.mu     = dInfinity; // 摩擦係数:無限大
      dJointID c = dJointCreateContact(world,
                                       contactgroup,&contact[i]);
      dJointAttach (c,dGeomGetBody(contact[i].geom.g1),
                      dGeomGetBody(contact[i].geom.g2));
    }
  }
}

// ロボットの描画
static void drawMonoBot()
{
  const dReal *pos1,*R1,*pos2,*R2;

  // 胴体部(球)の描画
  dsSetColor(1.0,0.0,0.0);                       // 赤色
  pos1 = dBodyGetPosition(torso.body);
  R1   = dBodyGetRotation(torso.body);
  dsDrawSphere(pos1,R1,torso.r);

  // 脚部(カプセル）の描画
  for (int i = 0; i < 2; i++) {
    pos2 = dBodyGetPosition(leg[i].body);
    R2   = dBodyGetRotation(leg[i].body);
    if (i == 0) {
      dsSetColor(0.0,0.0,1.0);                    // 青色
      dsDrawCylinder(pos2,R2,leg[i].l,leg[i].r);
    }
    else {
      dsSetColor(1.2,1.2,1.2);                   // 白色
      dsDrawCapsule(pos2,R2,leg[i].l,leg[i].r);
    }
  }
}

// ヒンジジョイントの制御
static void controlHinge(dReal target)
{
  static dReal kp = 10.0, fmax = 1000;

  dReal tmp   = dJointGetHingeAngle(h_joint);
  dReal diff  = target - tmp;
  if (diff >=   M_PI) diff -= 2.0 * M_PI; // diffが2πより小さく
  if (diff <= - M_PI) diff += 2.0 * M_PI; // diffが-2πより大きく
  dReal u     = kp * diff;

  dJointSetHingeParam(h_joint, dParamVel,  u);
  dJointSetHingeParam(h_joint, dParamFMax, fmax);
}

// スライダージョイントの制御
static void controlSlider(dReal target)
{
  static dReal max_force = 1000;

  if (target > 0) dJointSetSliderParam(s_joint, dParamVel,  8.0);
  else            dJointSetSliderParam(s_joint, dParamVel, -8.0);
  dJointSetSliderParam(s_joint, dParamFMax, max_force);
}


// シミュレーションループ
static void simLoop(int pause)
{
  if (S_LENGTH >=  0.25) S_LENGTH =   0.25;
  if (S_LENGTH <= -0.25) S_LENGTH =  -0.25;
  if (H_ANGLE  >   M_PI) H_ANGLE  =  -M_PI;
  if (H_ANGLE  <  -M_PI) H_ANGLE  =   M_PI;

  if (!pause) {
    STEPS++;
    controlSlider(S_LENGTH);
    controlHinge(H_ANGLE);
    dSpaceCollide(space,0,&nearCallback);
    dWorldStep(world,0.01);
    dJointGroupEmpty(contactgroup);
  }
  drawMonoBot(); // ロボットの描画
}

// ロボットの生成
void createMonoBot() {
  dMass mass;
  dReal x0 = 0.0, y0 = 0.0, z0 = 1.5;

  // 胴体(球）
  torso.r    = 0.25;
  torso.m    = 14.0;
  torso.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass,torso.m,torso.r);
  dBodySetMass(torso.body,&mass);
  dBodySetPosition(torso.body, x0, y0, z0);
  torso.geom = dCreateSphere(space,torso.r);
  dGeomSetBody(torso.geom,torso.body);

  // 脚(円柱)
  leg[0].l = 0.75;  leg[1].l = 0.75;    // 長さ
  leg[0].r = 0.05;  leg[1].r = 0.03;    // 半径
  for (int i = 0; i < 2; i++) {
    leg[i].m   = 3.0;
    leg[i].body   = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass,leg[i].m,3,leg[i].r,leg[i].l);
    dBodySetMass(leg[i].body,&mass);
    if (i == 0)
      dBodySetPosition(leg[i].body, x0, y0, z0-0.5*leg[0].l);
    else
      dBodySetPosition(leg[i].body, x0, y0, z0-0.5*leg[0].l-0.5);
    leg[i].geom = dCreateCapsule(space,leg[i].r,leg[i].l);
    dGeomSetBody(leg[i].geom,leg[i].body);
  }

  // ヒンジジョイント
  h_joint = dJointCreateHinge(world, 0);
  dJointAttach(h_joint, torso.body,leg[0].body);
  dJointSetHingeAnchor(h_joint, x0, y0, z0);
  dJointSetHingeAxis(h_joint, 1, 0, 0);

  // スライダージョイント
  s_joint = dJointCreateSlider(world, 0);
  dJointAttach(s_joint, leg[0].body,leg[1].body);
  dJointSetSliderAxis(s_joint, 0, 0, 1);
  dJointSetSliderParam(s_joint, dParamLoStop, -0.25);
  dJointSetSliderParam(s_joint, dParamHiStop,  0.25);
}

// ロボットの破壊
void destroyMonoBot()
{
	dJointDestroy(h_joint);   // ヒンジ
  dJointDestroy(s_joint);   // スライダー
  dBodyDestroy(torso.body); // 胴体のボディを破壊
  dGeomDestroy(torso.geom); // 胴体のジオメトリを破壊

  for (int i = 0; i < 2; i++) {
    dBodyDestroy(leg[i].body);  // 脚のボディを破壊
    dGeomDestroy(leg[i].geom);  // 脚のジオメトリを破壊
  }
}

// シミュレーションの再スタート
static void restart()
{
  STEPS    = 0;      // ステップ数の初期化
  S_LENGTH = 0.0;    // スライダ長の初期化
  H_ANGLE  = 0.0;    // ヒンジ角度の初期化

  destroyMonoBot();  // ロボットの破壊
  dJointGroupDestroy(contactgroup);     // ジョイントグループの破壊
  contactgroup = dJointGroupCreate(0);  // ジョイントグループの生成
  createMonoBot();                      // ロボットの生成
}

// キー操作
static void command(int cmd)
{
 switch (cmd) {
   case 'j':S_LENGTH =   0.25; break;
   case 'f':S_LENGTH = - 0.25; break;
   case 'k':H_ANGLE +=   0.25; break;
   case 'd':H_ANGLE -=   0.25; break;
   case 'u':dBodyAddForce(torso.body, 0, 0, 500); break;
   case 'r':restart()                           ; break;
   default :printf("key missed \n")             ; break;
 }
}

static void start()
{
  static float xyz[3] = {   3.5, 0.0, 1.0};
  static float hpr[3] = {-180.0, 0.0, 0.0};
  dsSetViewpoint(xyz,hpr);               // 視点，視線の設定
  dsSetSphereQuality(3);                 // 球の品質設定
}

void setDrawStuff()           /*** 描画関数の設定 ***/
{
  fn.version = DS_VERSION;    // ドロースタッフのバージョン
  fn.start   = &start;        // 前処理 start関数のポインタ
  fn.step    = &simLoop;      // simLoop関数のポインタ
  fn.command = &command;      // キー入力関数へのポインタ
  fn.path_to_textures = "../../drawstuff/textures"; // テクスチャ
}


int main (int argc, char *argv[])
{
  dInitODE();
  setDrawStuff();

  world        = dWorldCreate();
  space        = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  dWorldSetGravity(world, 0,0, -9.8);
  dWorldSetERP(world, 0.9);          // ERPの設定
  dWorldSetCFM(world, 1e-4);         // CFMの設定
  ground = dCreatePlane(space, 0, 0, 1, 0);
  createMonoBot();
  dsSimulationLoop (argc, argv, 640, 480, &fn);
  dWorldDestroy (world);
  dCloseODE();

  return 0;
}
