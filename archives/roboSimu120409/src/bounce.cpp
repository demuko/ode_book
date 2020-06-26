// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// プログラム 1.2:  ボールの跳ね返り bounce.cpp by Kosei Demura (2007-5-18)
//
// This program is a sample program of my book as follows
//“Robot Simulation - Robot programming with Open Dynamics Engine,
// (260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd.,
// Tokyo, 2007)” by Kosei Demura, which is written in Japanese (sorry).
// http://demura.net/simulation
// Please use this program if you like. However it is no warranty.
// bounce.cpp by Kosei Demura (2007-5-18)
//
// 更新履歴　(change log)
// 2008-7-7: dInitODE(),dCloseODE()の追加
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE                      // 単精度と倍精度の両方に対応する
#define dsDrawSphere dsDrawSphereD  // ためのおまじない
#endif
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
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
MyObject ball;

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
       contact[i].surface.bounce = 1.0;          // 反発係数(0.0から1.0)
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

// シミュレーションループ
static void simLoop(int pause)
{
  dSpaceCollide(space,0,&nearCallback);  // 衝突検出関数

  dWorldStep(world,0.01);
  dJointGroupEmpty(contactgroup); // ジョイントグループを空にする

  dsSetColor(1.0,0.0,0.0);
  dsDrawSphere(dBodyGetPosition(ball.body),
                dBodyGetRotation(ball.body),ball.r);
}

// ボールの生成
static void makeBall()
{
  dReal x0 = 0.0, y0 = 0.0, z0 = 2.0;
  dMass mass;

  ball.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass,m,r);
  dBodySetMass(ball.body,&mass);
  dBodySetPosition(ball.body, x0, y0, z0);
  ball.r      = r;
  ball.geom   = dCreateSphere(space,ball.r); // 球ジオメトリの生成
  dGeomSetBody(ball.geom,ball.body);         // ボディとジオメトリの関連付け
}

void start()                                  /*** 前処理　***/
{
  static float xyz[3] = {3.0,0.0,1.0};         // 視点の位置
  static float hpr[3] = {-180, 0, 0};          // 視線の方向
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
  setDrawStuff();

  dInitODE(); // ODEの初期化
  world = dWorldCreate();
  dWorldSetGravity(world,0,0,-0.5);

  space        = dHashSpaceCreate(0);   // 衝突用空間の創造
  contactgroup = dJointGroupCreate(0);  // ジョイントグループの生成
  ground = dCreatePlane(space,0,0,1,0); // 平面ジオメトリの生成
  makeBall(); 				// 球の作成

  dsSimulationLoop(argc,argv,640, 480,&fn);
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE(); // ODEの終了
  return 0;
}
