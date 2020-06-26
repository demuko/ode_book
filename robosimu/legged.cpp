// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// プログラム 8.1:  4脚ロボット歩行プログラム
// This program is a sample program of my book as follows
//“Robot Simulation - Robot programming with Open Dynamics Engine,
// (260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd.,
// Tokyo, 2007)” by Kosei Demura, which is written in Japanese (sorry).
// http://demura.net/simulation
// Please use this program if you like. However it is no warranty.
// legged.cpp by Kosei Demura (2007-2008)
// 更新履歴 (Change log)
// 2008-10-3: dWorldSetCFM(world, 1e-3),dWorldSetERP(world, 0.8)の追加 (add)
// 2008-7-7: dInitODE(),dCloseODE()の追加 (add)
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

dWorldID      world;        // 動力学計算用のワールド (for dynamics)
dSpaceID      space;        // 衝突検出用のスペース (for collision)
dGeomID       ground;       // 地面 (ground)
dJointGroupID contactgroup; // 接触点グループ (contact group for collision)
dsFunctions   fn;           // ドロースタッフの描画関数 (function of drawstuff)

#define LINK_NUM 3  // 全リンク数 (total number of links)
#define JT_NUM   3  // 全ジョイント数 (total number of joints)
#define LEG_NUM  4  // 全脚数 (total number of legs)

typedef struct {
  dBodyID  body;
  dGeomID  geom;
  dJointID joint;
  dReal    m,r,x,y,z; // 質量(weight)，半径(radius)，位置(positin:x,y,z)
} MyLink;

MyLink leg[LEG_NUM][LINK_NUM],torso; // 脚(leg)，胴体(torso)

dReal  THETA[LEG_NUM][LINK_NUM] = {{0},{0},{0},{0}}; // 目標角度(target angle)
dReal  SX = 0, SY = 0, SZ = 0.65;           // 胴体重心の初期位置(initial positon of COG)
dReal  gait[12][LEG_NUM][JT_NUM] ;          // 目標角度 (target angle of gait)
dReal  l1 = 1.5*0.05, l2 = 0.3, l3  = 0.3;  // リンク長 0.25 (lenth of links)

dReal  lx = 0.5, ly= 0.3, lz = 0.05;         // body sides
dReal  r1 = 0.02, r2 = 0.02, r3 = 0.02 ;     // leg radius
dReal  cx1 = (lx-r1)/2, cy1 = (ly+l1)/2;     // 一時変数 (temporal variable)
dReal  c_x[LEG_NUM][LINK_NUM] = {{ cx1, cx1, cx1},{-cx1,-cx1,-cx1}, // 関節中心x座標(center of joints)
                                 {-cx1,-cx1,-cx1},{ cx1, cx1, cx1}};
dReal  c_y[LEG_NUM][LINK_NUM] = {{ cy1, cy1, cy1},{ cy1, cy1, cy1}, // 関節中心y座標(center of joints)
                                 {-cy1,-cy1,-cy1},{-cy1,-cy1,-cy1}};
dReal  c_z[LEG_NUM][LINK_NUM] =  {{0, 0, -l2},                      // 関節中心z座標(center of joints)
																	{0, 0, -l2},{0, 0, -l2},{0, 0, -l2}};

/*** ロボットの生成 ***/
void  makeRobot()
{
  dReal torso_m = 10.0;                    // 胴体の質量 (weight of torso)
  dReal  l1m = 0.005,l2m = 0.5, l3m = 0.5; // リンクの質量 (weight of links)

  dReal x[LEG_NUM][LINK_NUM] = {{ cx1, cx1, cx1},{-cx1,-cx1,-cx1},// 各リンクの位置(link position, x座標)
                                {-cx1,-cx1,-cx1},{ cx1, cx1, cx1}};
  dReal y[LEG_NUM][LINK_NUM] = {{ cy1, cy1, cy1},{ cy1, cy1, cy1},// 各リンクの位置(link position, y座標)
                                {-cy1,-cy1,-cy1},{-cy1,-cy1,-cy1}};
  dReal z[LEG_NUM][LINK_NUM] = {                                  // 各リンクの位置(link position, z座標)
																{c_z[0][0],(c_z[0][0]+c_z[0][2])/2,c_z[0][2]-l3/2},
                                {c_z[0][0],(c_z[0][0]+c_z[0][2])/2,c_z[0][2]-l3/2},
                                {c_z[0][0],(c_z[0][0]+c_z[0][2])/2,c_z[0][2]-l3/2},
                                {c_z[0][0],(c_z[0][0]+c_z[0][2])/2,c_z[0][2]-l3/2}};
  dReal r[LINK_NUM]          =  { r1, r2, r3}; // リンクの半径 (radius of links)
  dReal length[LINK_NUM]     =  { l1, l2, l3}; // リンクの長さ (length of links)
  dReal weight[LINK_NUM]     =  {l1m,l2m,l3m}; // リンクの質量 (weight of links)
  dReal axis_x[LEG_NUM][LINK_NUM] = {{ 0,1, 0},{ 0,1,0},{ 0, 1, 0},{ 0, 1, 0}};
  dReal axis_y[LEG_NUM][LINK_NUM] = {{ 1,0, 1},{ 1,0,1},{ 1, 0, 1},{ 1, 0, 1}};
  dReal axis_z[LEG_NUM][LINK_NUM] = {{ 0,0, 0},{ 0,0,0},{ 0, 0, 0},{ 0, 0, 0}};

  // 胴体の生成 (crate a torso)
	dMass mass;
	torso.body  = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,torso_m, lx, ly, lz);
  dBodySetMass(torso.body,&mass);
  torso.geom = dCreateBox(space,lx, ly, lz);
  dGeomSetBody(torso.geom, torso.body);
  dBodySetPosition(torso.body, SX, SY, SZ);

  // 脚の生成 (create 4 legs)
  dMatrix3 R;                          // 回転行列
  dRFromAxisAndAngle(R,1,0,0,M_PI/2);  // 90度回転し地面と平行
  for (int i = 0; i < LEG_NUM; i++) {
    for (int j = 0; j < LINK_NUM; j++) {
      leg[i][j].body = dBodyCreate(world);
      if (j == 0) dBodySetRotation(leg[i][j].body,R);
      dBodySetPosition(leg[i][j].body, SX+x[i][j], SY+y[i][j], SZ+z[i][j]);
      dMassSetZero(&mass);
      dMassSetCapsuleTotal(&mass,weight[j],3,r[j],length[j]);
      dBodySetMass(leg[i][j].body, &mass);
      leg[i][j].geom = dCreateCapsule(space,r[j],length[j]);
      dGeomSetBody(leg[i][j].geom,leg[i][j].body);
    }
  }

  // ジョイントの生成とリンクへの取り付け (create links and attach legs to the torso)
  for (int i = 0; i < LEG_NUM; i++) {
    for (int j = 0; j < LINK_NUM; j++) {
      leg[i][j].joint = dJointCreateHinge(world, 0);
      if (j == 0)
        dJointAttach(leg[i][j].joint, torso.body, leg[i][j].body);
      else
        dJointAttach(leg[i][j].joint, leg[i][j-1].body, leg[i][j].body);
      dJointSetHingeAnchor(leg[i][j].joint, SX+c_x[i][j], SY+c_y[i][j],SZ+c_z[i][j]);
      dJointSetHingeAxis(leg[i][j].joint, axis_x[i][j], axis_y[i][j],axis_z[i][j]);
    }
  }
}

/*** ロボットの描画 (draw a robot) ***/
void drawRobot()
{
   dReal r,length;
   dVector3 sides;

   // 胴体の描画
   dsSetColor(1.3,1.3,1.3);
   dGeomBoxGetLengths(torso.geom,sides);
   dsDrawBox(dBodyGetPosition(torso.body),
	 					 dBodyGetRotation(torso.body),sides);

   // 脚の描画
   for (int i = 0; i < LEG_NUM; i++) {
     for (int j = 0; j < LINK_NUM; j++ ) {
       dGeomCapsuleGetParams(leg[i][j].geom, &r,&length);
			 if (j== 0) dsDrawCapsule(dBodyGetPosition(leg[i][j].body),
						          dBodyGetRotation(leg[i][j].body),0.5*length,1.2*r);
       else       dsDrawCapsule(dBodyGetPosition(leg[i][j].body),
						          dBodyGetRotation(leg[i][j].body),length,r);
     }
   }
}

static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
  dBodyID b1 = dGeomGetBody(o1), b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;
  // if ((o1 != ground) && (o2 != ground)) return;

  static const int N = 20;
  dContact contact[N];
  int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (int i=0; i<n; i++) {
      contact[i].surface.mode = dContactSoftERP | dContactSoftCFM;
      contact[i].surface.mu   = dInfinity; //2.0;
      contact[i].surface.soft_erp = 0.9;
      contact[i].surface.soft_cfm = 1e-5;
	    dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
      dJointAttach(c,b1,b2);
    }
  }
}

/*** 逆運動学の計算 (calculate inverse kinematics ***/
void  inverseKinematics(dReal x, dReal y, dReal z,
                     dReal *ang1, dReal *ang2, dReal *ang3,int posture)
{
  dReal l1a = 0, l3a = l3 + r3/2;

  double c3 = (x*x + z*z + (y-l1a)*(y-l1a) - (l2*l2+l3a*l3a))/(2*l2*l3a);
  double s2 = (y-l1a) / (l2 + l3a*c3);
  double c2 = sqrt(1 - s2 * s2);
  double c1 = (l2 + l3a*c3)*c2/sqrt(x*x+z*z);
	// printf("c3=%f s2=%f c2=%f c1=%f \n", c3,s2,c2,c1);
	if (sqrt(x*x+y*y+z*z) > l2 + l3) {
  	printf(" Target point is out of range \n");
	}

  switch (posture) {
  case 1: // 姿勢１ (posture 1)
    *ang1 =   atan2(x,-z) - atan2(sqrt(1 - c1*c1),c1);
    *ang2 = - atan2(s2,c2);
    *ang3 =   atan2(sqrt(1-c3*c3),c3); break;
  case 2: // 姿勢２ (posture 2)
    *ang1=   atan2(x,-z) + atan2(sqrt(1 - c1*c1),c1);
    *ang2= - atan2(s2,c2);
    *ang3= - atan2(sqrt(1-c3*c3),c3); break;
  case 3:  // 姿勢３ (posture 3)
    *ang1 =   M_PI + (atan2(x,-z) - atan2(sqrt(1 - c1*c1),c1));
    *ang2 = - M_PI +  atan2(s2,c2);
    *ang3 = - atan2(sqrt(1-c3*c3),c3); break;
  case 4:  // 姿勢４ (posture 4)
    *ang1 =  M_PI + atan2(x,-z) + atan2(sqrt(1 - c1*c1),c1);
    *ang2 = -M_PI + atan2(s2,c2);
    *ang3 =  atan2(sqrt(1-c3*c3),c3); break;
  }
}

/*** P制御 (P control) ***/
void Pcontrol()
{
  dReal kp = 2.0, fMax = 100.0;

  for (int i = 0; i < LEG_NUM; i++) {
    for (int j = 0; j < LINK_NUM; j++) {
      dReal tmp = dJointGetHingeAngle(leg[i][j].joint);
      dReal diff = THETA[i][j] - tmp;
      dReal u = kp * diff;
      dJointSetHingeParam(leg[i][j].joint,  dParamVel, u);
      dJointSetHingeParam(leg[i][j].joint, dParamFMax, fMax);
    }
  }
}

/*** 歩行制御 (gait control) ***/
void walk()
{
  static int t = 0, steps = 0;
  int interval = 50;

  if ((steps++ % interval)==0){
	  t++;
	}
  else {  // 目標関節角度の設定 (set target gait angles)
    for (int leg_no = 0; leg_no < LEG_NUM; leg_no++) {
      for (int joint_no = 0; joint_no < JT_NUM; joint_no++) {
        THETA[leg_no][joint_no] = gait[t%12][leg_no][joint_no];
      }
    }
  }
  Pcontrol(); // P制御 (P control)
}

/*** シミュレーションループ (Simulation Loop) ***/
void simLoop(int pause)
{
  if (!pause) {
    walk();                               // 歩行制御 (gait control)
    dSpaceCollide(space,0,&nearCallback); // 衝突検出 (collision detection)
    dWorldStep(world, 0.01);              // ステップ更新 (step a simulation)
    dJointGroupEmpty(contactgroup);       // 接触点グループを空 (empty jointgroup)
  }
  drawRobot();                            // ロボットの描画 (draw a robot)
}

/*** 視点と視線の設定 (Set view point and direction) ***/
void start()
{
  float xyz[3] = {  1.0f,  -1.2f, 0.5f};  // 視点[m] (View point)
  float hpr[3] = {121.0f, -10.0f, 0.0f};  // 視線[°] (View direction)
  dsSetViewpoint(xyz,hpr);                // 視点と視線の設定 (Set View point and direction)
  dsSetSphereQuality(3);
  dsSetCapsuleQuality(6);
}

/*** ドロースタッフの設定 (Set drawstuff) ***/
void setDrawStuff() {
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = NULL;
  fn.path_to_textures = "../../drawstuff/textures";
}

void calcAngle()        /*** 目標角度の計算 (Calculate target angles) ***/
{
  dReal z0 = -0.4,z1 = -0.37; // z0:地面までの高さ(height to the ground)，z1:最高到達点 (highest point)
  dReal y1 = 0.05, fs = 0.2;  // y1:左右の変位(defference between right and left)，fs:歩幅 (foot step)
  dReal f1 = fs/4, f2 = fs/2, f3 = 3 * fs/4, f4 = fs;  // 一時変数 (temporal variables)
  dReal  traj[12][LEG_NUM][3] = { // 目標軌道点 (trajectory points)
    // leg0: left fore leg,  leg1: left rear leg
    // leg2: right rear leg, leg3: right fore leg
    // 左前leg0遊脚 左後leg1    右後 leg2    右前leg3
    {{ 0, y1,z0},{  0, y1,z0},{  0, y1,z0},{  0, y1,z0}},// 0 重心移動(move COG)
    {{f2, y1,z1},{  0, y1,z0},{  0, y1,z0},{  0, y1,z0}},// 1 離地(takeoff)
    {{f4, y1,z0},{  0, y1,z0},{  0, y1,z0},{  0, y1,z0}},// 2 着地(touchdown)
    {{f3,-y1,z0},{-f1,-y1,z0},{-f1,-y1,z0},{-f1,-y1,z0}},// 3 重心移動(move COG)
    //    leg0        leg1     leg2 遊脚(swing)  leg3
    {{f3,-y1,z0},{-f1,-y1,z0},{ f1,-y1,z1},{-f1,-y1,z0}},// 4 離地(takeoff)
    {{f3,-y1,z0},{-f1,-y1,z0},{ f3,-y1,z0},{-f1,-y1,z0}},// 5 着地(touchdown)
    {{f2,-y1,z0},{-f2,-y1,z0},{ f2,-y1,z0},{-f2,-y1,z0}},// 6 重心移動(move COG)
    //    leg0        leg1      leg2       leg3 遊脚(swing)
    {{f2,-y1,z0},{-f2,-y1,z0},{ f2,-y1,z0},{  0,-y1,z1}},// 7 離地(takeoff)
    {{f2,-y1,z0},{-f2,-y1,z0},{ f2,-y1,z0},{ f2,-y1,z0}},// 8 着地(touchdown)
    {{f1, y1,z0},{-f3, y1,z0},{ f1, y1,z0},{ f1, y1,z0}},// 9 重心移動(move COG)
    //   leg0    leg1 遊脚(swing)  leg2         leg3
    {{f1,y1, z0},{-f1, y1,z1},{ f1, y1,z0},{ f1, y1,z0}},// 10 離地(takeoff)
    {{f1,y1, z0},{ f1, y1,z0},{ f1, y1,z0},{ f1, y1,z0}} // 11 着地(touchdown)
  };

  dReal angle1, angle2, angle3;
  int posture = 2;                                       // 姿勢(posture)
  for (int i = 0; i < 12; i++) {
    for (int k = 0; k < LEG_NUM; k++) {
      inverseKinematics(traj[i][k][0],traj[i][k][1],     // 逆運動学(inverse kinematics)
          traj[i][k][2],&angle1, &angle2, &angle3,posture);
      gait[i][k][0] = angle1;
      gait[i][k][1] = angle2;
      gait[i][k][2] = angle3;
    }
  }
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
  dWorldSetCFM(world, 1e-3); // CFMの設定 (global CFM)
  dWorldSetERP(world, 0.9);  // ERPの設定 (global ERP)
  makeRobot();
  calcAngle();
  dsSimulationLoop(argc,argv,800,480,&fn);
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();
  return 0;
}
