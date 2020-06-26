// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// プログラム 1.1:  ボールの自由落下 hello.cpp by Kosei Demura (2007-5-18)
//
// This program is a sample program of my book as follows
//“Robot Simulation - Robot programming with Open Dynamics Engine,
// (260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd.,
// Tokyo, 2007)” by Kosei Demura, which is written in Japanese (sorry).
// http://demura.net/simulation
// Please use this program if you like. However it is no warranty.
// hello.cpp by Kosei Demura (2007-5-18)
//
// 更新履歴　(change log)
// 2008-7-7: dInitODE(),dCloseODE()の追加
#include <ode/ode.h>                // ODE用ヘッダーファイル (ODE header file)
#include <drawstuff/drawstuff.h>    // 描画用ヘッダーファイル (drawstuff header file)
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

dWorldID world;                     // 動力学の世界 (a dynamics world)
dBodyID  apple;                     // リンゴ (an apple)
dReal    r = 0.2, m = 1.0;          // リンゴの半径,質量 (radius, mass)
dsFunctions fn;                     // ドロースタッフ用の構造体 (drawstuff structure)

void simLoop(int pause)           /***  シミュレーションループ (simulation loop)　***/
{
  dWorldStep(world,0.01);        // シミュレーションを1ステップ進める (step a simulation)

  dsSetColor(1.0,0.0,0.0);                     // 赤色の設定(r,g,b, set red color)
  const dReal *pos = dBodyGetPosition(apple);  // リンゴの位置を取得 (get the position of an apple)
  const dReal *R   = dBodyGetRotation(apple);  // リンゴの姿勢を取得 (get the rotation matrix of an apple)
  dsDrawSphereD(pos,R,r);                      // リンゴの描画 (draw an apple)
}

void start()                                  /*** 前処理　(start funciton) ***/
{
  static float xyz[3] = {3.0,0.0,1.0};         // 視点の位置 (view point)
  static float hpr[3] = {-180, 0, 0};          // 視線の方向 (view direction)
  dsSetViewpoint(xyz,hpr);                     // カメラの設定 (set the view point and direction)
}

void setDrawStuff()           /*** 描画関数の設定 (set a drawstuff) ***/
{
  fn.version = DS_VERSION;    // ドロースタッフのバージョン (version of drawstuff)
  fn.start   = &start;        // 前処理 start関数のポインタ (start function)
  fn.step    = &simLoop;      // simLoop関数のポインタ (simLoop function)
  fn.path_to_textures = "../../drawstuff/textures"; // テクスチャ (texture path)
}

int main(int argc, char **argv)         /*** main関数 (main function) ***/
{
  dInitODE();                              // ODEの初期化 (initialize ODE)
  setDrawStuff();                          // 描画関数の設定 (set drawstuff)
  world = dWorldCreate();                  // 世界の創造 (create a world)
  dWorldSetGravity(world,0,0,-0.2);        // 重力設定 (set gravity)

  apple = dBodyCreate(world);              // ボールの生成 (make an apple)
  dMass mass;                              // 構造体massの宣言 (declare mass parameter)
  dMassSetZero(&mass);                     // 構造体massの初期化(initialize mass parameter)
  dMassSetSphereTotal(&mass,m,r);          // 構造体massに質量を設定 (calculate mass parameter)
  dBodySetMass(apple,&mass);               // ボールにmassを設定 (set mass parameter to an apple)
  dBodySetPosition(apple, 0.0, 0.0, 2.0);  // ボールの位置(x,y,z)を設定 (set position of an apple)

  dsSimulationLoop(argc,argv,320, 240,&fn); // シミュレーションループ (simulation loop)
  dWorldDestroy(world);                    // 世界の終焉 (destroy the world)
  dCloseODE();                             // ODEの終了 (close ODE)
  return 0;
}
