// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// プログラム 2.5:  hopper.cpp by Kosei Demura (2007-5-17)
//
// This program is a sample program of my book as follows
//“Robot Simulation - Robot programming with Open Dynamics Engine,
// (260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd.,
// Tokyo, 2007)” by Kosei Demura, which is written in Japanese (sorry).
// http://demura.net/simulation
// Please use this program if you like. However it is no warranty.
// hopper.cpp by Kosei Demura (2007-5-18)
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

// コールバック関数
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int N = 10; // 接触点数の最大値
  dContact contact[N];     // 接触点

  // 接触している物体のどちらかが地面ならisGroundに非0をセット
  int isGround = ((ground == o1) || (ground == o2));

  // 衝突情報の生成 nは衝突点数
  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (isGround)  {
     for (int i = 0; i < n; i++) {
       contact[i].surface.mode = dContactBounce; // 接触面の反発性を設定
       contact[i].surface.bounce     = 1.0;      // 反発係数(0.0から1.0)
       contact[i].surface.bounce_vel = 0.0;      // 反発に必要な最低速度
       // 接触ジョイントの生成
       dJointID c = dJointCreateContact(world,contactgroup,
                                                &contact[i]);
       // 接触している２つの剛体を接触ジョイントにより拘束
       dJointAttach(c,dGeomGetBody(contact[i].geom.g1),
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

// スライダの制御 プログラム2.4
static void controlSlider(dReal target)
{
  static dReal kp   = 25.0;                       // 比例定数
  static dReal fmax = 400;                        // 最大力[N]

  dReal tmp  = dJointGetSliderPosition(s_joint);  // スライダの現在位置
  dReal u    = kp * (target - tmp);               // 残差

  dJointSetSliderParam(s_joint, dParamVel,  u);
  dJointSetSliderParam(s_joint, dParamFMax, fmax);
}



// シミュレーションループ
static void simLoop(int pause)
{
  int s = 200;                   // 跳躍する周期(ステップ)

  STEPS++;                      //  ステップ数
  printf("STEPS:%4d\n",STEPS);

  // スライダーの伸縮
  if ((0 <= (STEPS%s)) && ((STEPS%s) <= 10)) controlSlider(0.5);
  else                                       controlSlider(0.0);

  dSpaceCollide(space,0,&nearCallback);  // 衝突検出計算
  dWorldStep(world,0.01);                // 1ステップ進める
  dJointGroupEmpty(contactgroup);

  drawMonoBot();
}

static void start()
{
  static float xyz[3] = { 3.5,0.0,1.0};
  static float hpr[3] = {-180,0,0};
  dsSetViewpoint(xyz,hpr);               // 視点，視線の設定
  dsSetSphereQuality(3);                 // 球の品質設定
}

void setDrawStuff()           /*** 描画関数の設定 ***/
{
  fn.version = DS_VERSION;    // ドロースタッフのバージョン
  fn.start   = &start;        // 前処理 start関数のポインタ
  fn.step    = &simLoop;      // simLoop関数のポインタ
  fn.path_to_textures = "../../drawstuff/textures"; // テクスチャ
}

// ロボットの生成
void createMonoBot() {
  dMass mass;
  dReal x0 = 0.0, y0 = 0.0, z0 = 1.5;

  // 胴体(球）
  torso.r = 0.25;
  torso.m   = 14.0;
  torso.body   = dBodyCreate(world);
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

int main (int argc, char *argv[])
{
  dInitODE();
  setDrawStuff();
  world        = dWorldCreate();
  space        = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  dWorldSetGravity(world,0,0,-9.8);
  dWorldSetERP(world,1.0);          // ERPの設定
  dWorldSetCFM(world,0.0);          // CFMの設定
  ground = dCreatePlane(space,0,0,1,0);
  createMonoBot();
  dsSimulationLoop (argc,argv,640,480,&fn);
  dWorldDestroy (world);
  dCloseODE();

  return 0;
}
