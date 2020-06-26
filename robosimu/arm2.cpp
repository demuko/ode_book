// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// プログラム 6.3:  ３自由度ロボットアーム(運動学実装）
//
// This program is a sample program of my book as follows
//“Robot Simulation - Robot programming with Open Dynamics Engine,
// (260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd.,
// Tokyo, 2007)” by Kosei Demura, which is written in Japanese (sorry).
// http://demura.net/simulation
// Please use this program if you like. However it is no warranty.
// arm2.cpp by Kosei Demura (2007-5-18)
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
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawBox      dsDrawBoxD
#endif
#define NUM 4                          // リンク数

dWorldID      world;                   // 動力学計算用のワールド
dSpaceID      space;                   // 衝突検出用のスペース
dGeomID       ground;                  // 地面のジオメトリID番号
dJointGroupID contactgroup;            // 接触点グループ
dJointID      joint[NUM];              // ジョイントのID番号
dsFunctions   fn;                      // ドロースタッフの描画関数

typedef struct {
  dBodyID body;                        // ボディのID番号
  dGeomID geom;                        // ジオメトリのID番号
} MyObject;                            // MyObject構造体

MyObject rlink[NUM];                   // リンク
dReal  THETA[NUM] = {0.0};             // 関節の目標角度[rad]
double P[3],a[3],b[3];                 // 先端の位置，有顔ベクトル(a,b)

dBodyID       sensor;        // センサ用のボディID
dJointID      sensor_joint;  // センサ固定用の関節

// センサの作成
void makeSensor()
{
  dMass mass;
  double sx = 0.0, sy = 0.0, sz = 3.0;  // センサの初期座標[m]
  double size = 0.01, weight = 0.00001; // センサのサイズ[m]と重量[kg]

  sensor = dBodyCreate(world);          // センサの生成
  dBodySetPosition(sensor,sx,sy,sz);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,weight,size,size,size);
  dBodySetMass(sensor, &mass);

  sensor_joint = dJointCreateFixed(world, 0); // 固定ジョイントの生成
  dJointAttach(sensor_joint, rlink[NUM-1].body, sensor); // 先端リンクと結合
  dJointSetFixed(sensor_joint);
}

// センサ位置の表示
void printSensorPosition()
{
  double *pos = (double *) dBodyGetPosition(sensor);
  printf("P*: x=%5.2f y=%5.2f z=%5.2f \n",pos[0],pos[1],pos[2]);
  printf("P : x=%5.2f y=%5.2f z=%5.2f \n",P[0],P[1],P[2]);
}

// センサ姿勢(有顔ベクトル)の表示
void printSensorRotation()
{
  dReal a3[3],b3[3];
  dReal a0[3] = {0,0,1},b0[3] = {1,0,0};
  const dReal *R   = (const dReal *) dBodyGetRotation(sensor);

  dMultiply0((dReal *) a3, R, a0, 3, 3, 1);
  dMultiply0((dReal *) b3, R, b0, 3, 3, 1);
  // printf("a*: x=%5.2f y=%5.2f z=%5.2f",    a[0],a[1],a[2]);
  // printf("b*: x=%5.2f y=%5.2f z=%5.2f \n", b[0],b[1],b[2]);
  // printf("a : x=%5.2f y=%5.2f z=%5.2f",   a3[0],a3[1],a3[2]);
  // printf("b : x=%5.2f y=%5.2f z=%5.2f \n",b3[0],b3[1],b3[2]);
}

/*** ロボットアームの生成 ***/
void  makeArm()
{
  dMass mass;                                    // 質量パラメータ
  dReal x[NUM]      = {0.00, 0.00, 0.00, 0.00};  // 重心 x
  dReal y[NUM]      = {0.00, 0.00, 0.00, 0.00};  // 重心 y
  dReal z[NUM]      = {0.05, 0.50, 1.50, 2.50};  // 重心 z
  dReal length[NUM] = {0.10, 0.90, 1.00, 1.00};  // 長さ
  dReal weight[NUM] = {9.00, 2.00, 2.00, 2.00};  // 質量
  dReal r[NUM]      = {0.20, 0.04, 0.04, 0.04};  // 半径
  dReal c_x[NUM]    = {0.00, 0.00, 0.00, 0.00};  // 関節中心点 x
  dReal c_y[NUM]    = {0.00, 0.00, 0.00, 0.00};  // 関節中心点 y
  dReal c_z[NUM]    = {0.00, 0.10, 1.00, 2.00};  // 関節中心点 z
  dReal axis_x[NUM] = {0, 0, 0, 0};              // 関節回転軸 x
  dReal axis_y[NUM] = {0, 0, 1, 1};              // 関節回転軸 y
  dReal axis_z[NUM] = {1, 1, 0, 0};              // 関節回転軸 z

  // リンクの生成
  for (int i = 0; i < NUM; i++) {
    rlink[i].body = dBodyCreate(world);
    dBodySetPosition(rlink[i].body, x[i], y[i], z[i]);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass,weight[i],3,r[i],length[i]);
    dBodySetMass(rlink[i].body, &mass);
    rlink[i].geom = dCreateCapsule(space,r[i],length[i]);
    dGeomSetBody(rlink[i].geom,rlink[i].body);
  }

  // ジョイントの生成とリンクへの取り付け
  joint[0] = dJointCreateFixed(world, 0);  // 固定ジョイント
  dJointAttach(joint[0], rlink[0].body, 0);
  dJointSetFixed(joint[0]);
  for (int j = 1; j < NUM; j++) {
    joint[j] = dJointCreateHinge(world, 0); // ヒンジジョイント
    dJointAttach(joint[j], rlink[j].body, rlink[j-1].body);
    dJointSetHingeAnchor(joint[j], c_x[j], c_y[j], c_z[j]);
    dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j],axis_z[j]);
  }
}

/*** ロボットアームの描画 ***/
void drawArm()
{
	dReal r,length;

	for (int i = 0; i < NUM; i++ ) {       // カプセルの描画
		dGeomCapsuleGetParams(rlink[i].geom, &r,&length);
		if (i != NUM -1)
			dsDrawCapsule(dBodyGetPosition(rlink[i].body),
								    dBodyGetRotation(rlink[i].body),length,r);
		else
			dsDrawCylinder(dBodyGetPosition(rlink[i].body),
								     dBodyGetRotation(rlink[i].body),length,r);
	}
}

void drawSensor()
{
   dReal sides[] = {0.01,0.01,0.01};

   dsSetColor(1,0,0);
   dBodyGetRotation(sensor);
   dsDrawBox(dBodyGetPosition(sensor),dBodyGetRotation(sensor),sides);
}

/*** P制御 ***/
void Pcontrol()
{
  dReal k =  10.0, fMax = 100.0;                   // 比例ゲイン，最大トルク

  for (int j = 1; j < NUM; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);     // 関節角の取得
    dReal z = THETA[j] - tmp;                      // 残差
    dJointSetHingeParam(joint[j],dParamVel, k*z);  // 角速度の設定
    dJointSetHingeParam(joint[j],dParamFMax,fMax); // トルクの設定
  }
}

/*** 視点と視線の設定 ***/
void start()
{
  float xyz[3] = {    3.0f, 1.3f, 0.8f};          // 視点[m]
  float hpr[3] = { -160.0f, 4.5f, 0.0f};          // 視線[°]
  dsSetViewpoint(xyz, hpr);                       // 視点と視線の設定
}

/*** キー入力関数 ***/
void command(int cmd)
{
  switch (cmd) {
    case 'j': THETA[1] += M_PI/180; break;     // jキー
    case 'f': THETA[1] -= M_PI/180; break;     // fキー
    case 'k': THETA[2] += M_PI/180; break;     // kキー
    case 'd': THETA[2] -= M_PI/180; break;     // dキー
    case 'l': THETA[3] += M_PI/180; break;     // lキー
    case 's': THETA[3] -= M_PI/180; break;     // sキー
  }
}

void directKinematics()
{
  int x = 0, y = 1, z = 2;
  double l[4] = { 0.10, 0.90, 1.00, 1.00};      // リンクの長さ[m]
  double angle[4];                              // 関節の角度[rad]

  angle[1] = dJointGetHingeAngle(joint[1]);     // 第1関節角度の取得
  angle[2] = dJointGetHingeAngle(joint[2]);     // 第2関節角度の取得
  angle[3] = dJointGetHingeAngle(joint[3]);     // 第3関節角度の取得

  // アーム先端位置
  P[x] = cos(angle[1]) * sin(angle[2]) * l[2]               // x座標
       + cos(angle[1]) * sin(angle[2] + angle[3]) * l[3];
  P[y] = sin(angle[1]) * sin(angle[2]) * l[2]               // y座標
       + sin(angle[1]) * sin(angle[2] + angle[3]) * l[3];
  P[z] = l[0] + l[1] + cos(angle[2]) * l[2]                 // z座標
       + cos(angle[2] + angle[3]) * l[3];

  // 有顔ベクトル
  a[x] =  cos(angle[1]) * sin(angle[2] + angle[3]);    // 主軸:x座標
  a[y] =  sin(angle[1]) * sin(angle[2] + angle[3]);    // 主軸:y座標
  a[z] =                  cos(angle[2] + angle[3]);    // 主軸:z座標
  b[x] =  cos(angle[1]) * cos(angle[2] + angle[3]);    // 副軸:x座標
  b[y] =  sin(angle[1]) * cos(angle[2] + angle[3]);    // 副軸:y座標
  b[z] =                - sin(angle[2] + angle[3]);    // 副軸:z座標
  // printf("P: x=%5.2f y=%5.2f z=%5.2f,",  P[0],P[1],P[2]);
  // printf("a: x=%5.2f y=%5.2f z=%5.2f,",  a[0],a[1],a[2]);
  // printf("b: x=%5.2f y=%5.2f z=%5.2f \n",b[0],b[1],b[2]);
}


/*** シミュレーションループ ***/
void simLoop(int pause)
{
  Pcontrol();                                  // P制御
  directKinematics();
  printSensorPosition();
  printSensorRotation();
	dWorldStep(world, 0.01);                     // 動力学計算
  drawArm();                                   // ロボットの描画
	drawSensor();
}

/*** ドロースタッフの設定 ***/
void setDrawStuff()
{
  fn.version = DS_VERSION;                     // バージョン番号
  fn.start   = &start;                         // start関数
  fn.step    = &simLoop;                       // simLoop関数
  fn.command = &command;                       // command関数
  fn.path_to_textures = "../../drawstuff/textures";
}

int main(int argc, char **argv)
{
  dInitODE(); // ODEの初期化
  setDrawStuff();
  world        = dWorldCreate();                  // ワールドの生成
  space        = dHashSpaceCreate(0);             // スペースの生成
  contactgroup = dJointGroupCreate(0);            // 接触グループの生成
  ground       = dCreatePlane(space, 0, 0, 1, 0); // 地面の生成
  dWorldSetGravity(world, 0, 0, -9.8);            // 重力の設定
  makeArm();                                      // アームの生成
  makeSensor();                                   // センサの生成
  dsSimulationLoop(argc, argv, 640, 480, &fn);    // シミュレーションループ
  dSpaceDestroy(space);                           // スペースの破壊
  dWorldDestroy(world);                           // ワールドの破壊
  dCloseODE();
  return 0; // ODEの終了
}
