// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// プログラム 1.3:  monoBot.cpp by Kosei Demura (2007-5-17)
//
// This program is a sample program of my book as follows
//“Robot Simulation - Robot programming with Open Dynamics Engine,
// (260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd.,
// Tokyo, 2007)” by Kosei Demura, which is written in Japanese (sorry).
// http://demura.net/simulation
// Please use this program if you like. However it is no warranty.
// monoBot.cpp by Kosei Demura (2007-5-18)
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
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
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

MyObject ball,leg;  // MyObject構造体
dJointID joint;     // ジョイント

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

static void simLoop(int pause)
{
  dSpaceCollide(space,0,&nearCallback);
  dWorldStep(world,0.01);
  dJointGroupEmpty(contactgroup);

  dsSetColor(1.0,0.0,0.0);
  // 球リンクの描画
  dsDrawSphere(dBodyGetPosition(ball.body),
      dBodyGetRotation(ball.body),ball.r);

  // 円柱リンクの描画
  dsDrawCapsule(dBodyGetPosition(leg.body),
      dBodyGetRotation(leg.body),leg.l,leg.r);
}

// オブジェクトの生成
void makeMonoBot()
{
  dMass mass;
  dReal x0 = 0.0, y0 = 0.0, z0 = 2.5;

  // 球
  ball.r = 0.2; ball.m = 1.0;
  ball.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass,ball.m,ball.r);
  dBodySetMass(ball.body,&mass);
  dBodySetPosition(ball.body, x0, y0, z0);
  ball.geom = dCreateSphere(space,ball.r);
  dGeomSetBody(ball.geom,ball.body);

  // カプセル
  leg.r = 0.025;  leg.l = 1.0;  leg.m   = 0.001;
  leg.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetCapsuleTotal(&mass,leg.m,3,leg.r,leg.l);
  dBodySetMass(leg.body,&mass);
  dBodySetPosition(leg.body, x0, y0, z0-ball.r-0.5*leg.l);
  leg.geom = dCreateCapsule(space,leg.r,leg.l);
  dGeomSetBody(leg.geom,leg.body);

  // ヒンジジョイント
  joint = dJointCreateHinge(world, 0);                  // ヒンジの生成
  dJointAttach(joint, ball.body,leg.body);              // ヒンジの取付
  dJointSetHingeAnchor(joint,x0,y0,z0-ball.r);          // 中心点の設定
  dJointSetHingeAxis(joint,1,0,0);                      // 回転軸の設定
  dJointSetHingeParam(joint, dParamLoStop, -0.25*M_PI); // 可動域下限
  dJointSetHingeParam(joint, dParamHiStop,  0.25*M_PI); // 可動域上限
}


void start()                                  /*** 前処理 ***/
{
  static float xyz[3] = {   3.0f, 0.0f, 1.0f}; // 視点の位置
  static float hpr[3] = {-180.0f, 0.0f, 0.0f}; // 視線の方向
  dsSetViewpoint(xyz,hpr);                     // カメラの設定
}

void setDrawStuff()           /*** 描画関数の設定 ***/
{
  fn.version = DS_VERSION;    // ドロースタッフのバージョン
  fn.start   = &start;        // 前処理 start関数のポインタ
  fn.step    = &simLoop;      // simLoop関数のポインタ
  fn.path_to_textures = "../../drawstuff/textures"; // テクスチャ
}


int main(int argc, char **argv)
{
  dInitODE(); // ODEの初期化
  setDrawStuff();
  world = dWorldCreate();                  // 動力学計算ワールドの作成
  space = dHashSpaceCreate(0);             // 衝突計算スペースの作成
  contactgroup = dJointGroupCreate(0);     // ジョイントグループの生成
  dWorldSetGravity(world,0,0,-9.8);        // 重力ベクトルの設定
  ground = dCreatePlane(space,0,0,1,0);    // 地面の作成

  makeMonoBot();                           // monoBotの作成

  dsSimulationLoop(argc,argv,640,480,&fn); // シミュレーションループ
  dSpaceDestroy(space);                    // スペースの破壊
  dWorldDestroy(world);                    // ワールドの破壊
  dCloseODE();
  return 0;
}

