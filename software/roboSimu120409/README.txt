これらのサンプルプログラムは以下の書籍用です．

簡単！実践！ロボットシミュレーション - Open Dynamics Engineによるロボットプログラミング-
出村公成著, 森北出版 (2007)
サポートサイト http://demura.net/

本サンプルプログラムのビルド，コンパイル並びに実行は自己責任で行ってください．特に，本物の
ロボットに適用するための安全性などを全く考慮していませんので，そのような用途には向きません．
また，教育目的のシミュレータであるため，精度を必要とするような研究の用途にも向きません．
あくまで，教育目的とお考えください．著者及び森北出版は本プログラムによって生じたあらゆる
結果についての責任を負いかねます．以上の点をご了承した方だけこれらのサンプルプログラムを
お使いください．まだ十分にテストしきれていないので再配布はお止めください．

2008年7月22日

出村公成

更新ログ
2012-04-09: ODE0.12用にpremake.luaとroboSimu.luaを変更
2009-06-14: Visual C++で発生するWarning, Errorを修正．ODE0.11のデモプログラムに合わせてtexturepath.hをインクルードするようソースコードに追加．
2008-08-28: omni.cppのsqrt()の引数を2から2.0に変更　(Yasuさん指摘）
2008-08-21: ODE0.10.0のOPCODEの新しい衝突検出関数ではエラーがでるのでロボットデザインを変更した．
　　　　　　　　　 pk.cppのmakeBase2(), drawBase2()の変更
            omni.cppの225,226行目drawCircle()のpos1,pos2の式を修正　（LWさん指摘）
2008-07-22: wheel2.cppの83行目dMassSetCapsuleTotal()の3番目引数をカプセル長軸方向である3に訂正
2008-07-07: ODE0.10.0からビルド，コンパイル方法が変更になり，それに対応した．同時に，2007年5月19日から今まで個別に配布していたODE本のサンプルプログラムを１つのパッケージとしてまとめた．
　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　
　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　
-------------------

/*** Robot Simulation Sample Programs by Kosei Demura ***/
These sample programs are for the following my book. 

“Robot Simulation - Robot programming with Open Dynamics Engine , 
(260pages, ISBN:978-4627846913, Morikita Publishing Co. Ltd., Tokyo, 2007)” 
by Kosei Demura, which is written in Japanese (Sorry).

URL is http://demura.net/simulation.

Please build, compile, and execute these sample programs at your own risk. 
Especially, the safety issues, to apply to a real robots are not considered at all.

These programs are no warranty of any kind. The author and Morikita publishing co. ltd.
are not responsible for any result caused by using these sample programs.

Please do not redistribute these programs, it seems there are some bugs.

Anyway, enjoy my sample program and the world of Open Dynamics Engine.


2008-10-03

Kosei Demura



