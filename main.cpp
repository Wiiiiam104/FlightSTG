// ver 1.0.0 beta
// g++ main.cpp -lglut -lglu32 -lopengl32 -fconcepts -lwinmm
#include <bits/stdc++.h>
#include <GL/glut.h>
#include <windows.h>
#include <mmsystem.h>
using namespace std;
#define PI (3.1415926535897932384626432)
#define RAND_DOUBLE ((double)rand()/RAND_MAX)
#define RAND_QUAT(a) quat{0,RAND_DOUBLE*2*a-a,RAND_DOUBLE*2*a-a,RAND_DOUBLE*2*a-a}
int windowWidth=800;
int windowHeight=450;
int maxNumOfBullet=10000;
int numOfStages=2;
int totalFrame=0;
int score=0;
int phaseFrame=0;
//0:inGame, 1:menu, 2:gamecompleted, 3:gameOver, 4:stageComplete
int gameStatus=1;
int stageId=0;
//0:easy, 1:normal, 2:hard, 3:???
int difficulty=0;
vector<int> numsOfPhese{1,1,3,3,3,3};

class Bullet;
list<Bullet> bullets;
deque<std::_List_iterator<Bullet>> bulletItrs;

//四元数
struct quat{
  //q:=w+xi+yj+zk
  double w, x, y, z;

  quat operator+(const quat q){
    return quat{w+q.w, x+q.x, y+q.y, z+q.z};
  }
  quat operator-(const quat q){
    return quat{w-q.w, x-q.x, y-q.y, z-q.z};
  }
  quat operator*(const quat q){
    double rw=w*q.w - x*q.x - y*q.y - z*q.z;
    double rx=w*q.x + x*q.w + y*q.z - z*q.y;
    double ry=w*q.y - x*q.z + y*q.w + z*q.x;
    double rz=w*q.z + x*q.y - y*q.x + z*q.w;
    return quat{rw, rx, ry, rz};
  }
  void operator=(const quat q){
    w=q.w; x=q.x; y=q.y; z=q.z;
  }
  bool operator==(const quat q){
    return w==q.w && x==q.x && y==q.y && z==q.z;
  }
  bool operator!=(const quat q){
    return !(w==q.w && x==q.x && y==q.y && z==q.z);
  }
  friend quat operator+(double r,const quat& q){
    return (quat{r,0,0,0})+q;
  }
  friend quat operator*(double r,const quat& q){
    return (quat{r,0,0,0})*q;
  }
  friend ostream& operator<<(ostream& os, const quat& q){
    return os<<q.w<<" + "<<q.x<<"i + "<<q.y<<"j + "<<q.z<<"k";
  }

  //共役
  quat conjugation(){
    return quat{w, -x, -y, -z};
  }
  //大きさ(ノルム)
  double abs(){
    return sqrt(w*w+x*x+y*y+z*z);
  }
  //単位四元数
  quat unit(){
    return quat{w/abs(), x/abs(), y/abs(), z/abs()};
  }
  //逆数
  quat inverse(){
    return quat{w/abs()/abs(), -x/abs()/abs(), -y/abs()/abs(), -z/abs()/abs()};
  }
  
  //ベクトル部の内積
  static double innerProduct(const quat& q0, const quat& q1){
    return q0.x*q1.x + q0.y*q1.y + q0.z*q1.z;
  }
  //ベクトル部の外積
  static quat clossProduct(const quat& q0, const quat& q1){
    double rx=q0.y*q1.z - q0.z*q1.y;
    double ry=q0.z*q1.x - q0.x*q1.z;
    double rz=q0.x*q1.y - q0.y*q1.x;
    return quat{0, rx, ry, rz};
  }
  //ベクトル部のなす角[rad](区間[0,PI])
  static double formedAngle(quat& q0, quat& q1){
    return acos(quat::innerProduct(q0, q1)/q0.abs()/q1.abs());
  }

  //単位純四元数のベクトル部周りにtheta[rad]だけ回転させた点
  quat rotate(quat axis, double theta){
    quat p={0, x, y, z};
    quat q=cos(theta/2)+sin(theta/2)*axis;
    return (q*p)*q.conjugation();
  }
  //回転後のq0のベクトル部が回転前のq1のベクトル部に一致するように回転させたときの,thisの回転後の点
  quat rotateToMatchQuats(quat q0, quat q1){
    if(quat::clossProduct(q0, q1).abs()>1e-15)return quat{0,x,y,z}.rotate(quat::clossProduct(q0, q1).unit(), quat::formedAngle(q0,q1));
    if((q0.unit()-q1.unit()).abs()<1e-15)return quat{0, x, y, z};
    if((q0.unit()-quat{0,1,0,0}).abs()>1e-15&&(q0.unit()-quat{0,-1,0,0}).abs()>1e-15)return quat{0,x,y,z}.rotate(quat::clossProduct(q0, {0,1,0,0}).unit(), PI);
    return quat{0,x,y,z}.rotate(quat::clossProduct(q0, {0,0,1,0}).unit(), PI);
  }

  //ベクトル部の画面への透視投影(z軸に垂直)
  //cf.https://en.wikipedia.org/wiki/3D_projection#Perspective_projection
  vector<int> projection(auto& player){
    //スカラー部が非0ならば投影しない
    if(::abs(w)>1e-15){return vector<int>{1<<20, 1<<20};}
    //playerからみた相対位置
    quat relativePosition=quat{0, x, y, z}-player.displacement;
    //player.directionがz軸に一致するように回転
    quat rotatedPointInTheFirstRotation=relativePosition.rotateToMatchQuats(player.direction, quat{0,0,0,-1});
    quat rotatedYawAxis=player.yawAxis.rotateToMatchQuats(player.direction, quat{0,0,0,-1});
    //rotatedYawAxisがy軸に一致するように回転
    quat rotatedPoint=rotatedPointInTheFirstRotation.rotateToMatchQuats(rotatedYawAxis, quat{0,0,1,0});
    //hfov:90degrees
    double W=windowWidth, H=windowHeight;
    return rotatedPoint.z<0? vector<int>{int(-(W/2)*rotatedPoint.x/rotatedPoint.z+W/2), int(-(H*16/9/2)*rotatedPoint.y/rotatedPoint.z+H/2)}: vector<int>{1<<20, 1<<20};
  }
};

class Shot{
  public:
  //exe起動からbullet生成までのフレーム数
  int initailFrame;
  //関数 変位(生成時を0とする時刻[frame])
  function<quat(int)> displacementFunc;
  //計算量の軽減のためのframe中の変位を保持
  quat displacement;
  //玉の半径(当たり判定/グラフィックに使用)
  double radius;
  //色(RGB表現/vector<double>{red, green, blue})}
  vector<double> color;
  //敵に付与するダメージの計算に使用
  float power;
  quat initialDisplacement;

  void updateDisplacement(int frame){
    displacement=(displacementFunc)(frame-initailFrame);
  }

  //Shotがenemyに当たったか判定
  bool hasHitEnemy(auto& enemy){
    if(::abs(displacement.w)>1e-15){return false;}
    if((displacement-enemy.displacement).abs()<1){
      return true;
    }
    return false;
  }

  //Shotを描画
  void display(auto& player){
    if(::abs(displacement.w)>1e-15){return;}
    vector<int> p=displacement.projection(player);
    double d=(displacement-player.displacement).abs();
    glColor3d(color[0], color[1], color[2]);
    glRectd(p[0]-windowWidth*9/16*radius/d-0.5, p[1]-windowHeight*radius/d-0.5, p[0]+windowWidth*9/16*radius/d+1, p[1]+windowHeight*radius/d+1);
  }
};

vector<Shot> shots;

class Player{
  public:
  //変位(純四元数のベクトル部で表現)
  quat displacement;
  //進行方向(単位純四元数のベクトル部で表現)
  quat direction;
  //速度(固有値/direction方向を正とする実数)
  double velocity;
  //左右の旋回時の回転軸(単位純四元数のベクトル部で表現/directionと垂直/機体の上部を正)
  quat yawAxis;
  //旋回の角速度[rad/frame](固有値/機体の旋回性能)
  double omega;
  //残基の個数
  int life;
  //ボムの個数
  int bomb;
  //ショットの間隔
  int shotInterval;
  //次のショットまでのクールダウン[frame]
  int shotCoolDown;
  //残りの無敵時間[frame]
  int invincible;

  void init(quat displacement_, quat direction_, double velocity_, quat yawAxis_, double omega_, int life_, int bomb_, int shotInterval_, int shotCoolDown_, int invincible_){
    displacement=displacement_;
    direction=direction_;
    velocity=velocity_;
    yawAxis=yawAxis_;
    omega=omega_;
    life=life_;
    bomb=bomb_;
    shotInterval=shotInterval_;
    shotCoolDown=shotCoolDown_;
    invincible=invincible_;
  }

  //コントローラの入力の偏角[rad]
  void cameraRotate(double argument){
    quat axis=yawAxis.rotate(direction, 3*PI-argument).unit();
    direction=direction.rotate(axis, omega);
    yawAxis=yawAxis.rotate(axis, omega);
  }

  void pressedLeftKey(){
    displacement=displacement-velocity*quat::clossProduct(direction, yawAxis).unit();
  }
  void pressedRightKey(){
    displacement=displacement+velocity*quat::clossProduct(direction, yawAxis).unit();
  }
  void pressedUpKey(){
    displacement=displacement+velocity*yawAxis;
  }
  void pressedDownKey(){
    displacement=displacement-velocity*yawAxis;
  }
  void pressedAdvanceKey(){
    displacement=displacement+velocity*direction;
  }
  void pressedRetreatKey(){
    displacement=displacement-velocity*direction;
  }
  void pressedShotKey(){
    if(shotCoolDown<=0){
      quat p=displacement,q=direction,r=yawAxis;
      function<quat(int)> f=[p,q,r](int t){return t*60.0/60.0*q+p-r;};
      shots.push_back(Shot{totalFrame, f, displacement, 0.03, {1,1,1}, 1, p});
      shotCoolDown=shotInterval-1;
    }
  }
  void pressedBombKey(){
    if(bomb>0 && invincible<=0){
      bomb--;
      invincible=120;
      bullets.clear();
      bulletItrs.clear();
    }
  }

  //bulletが当たった時の処理
  void death(){
    if(life>0){
      life--;
      bomb=3;
      invincible=120;
      bullets.clear();
      bulletItrs.clear();
    }else{
      gameStatus=3;
    }
  }

  string status(){
    string res="LIFE [ ";
    for(int i=0;i<8;i++){
      if(life>i){
        res+='#';
      }else{
        res+='.';
      }
    }
    res+=" ]  BOMB [ ";
    for(int i=0;i<8;i++){
      if(bomb>i){
        res+='#';
      }else{
        res+='.';
      }
    }
    res+=" ]";
    return res;
  }
};

Player player={{0,0,0,0},{0,0,0,-1},0.07,{0,0,1,0},PI/240,3,3,6,0,60};

class Bullet{
  public:
  //exe起動からbullet生成までのフレーム数
  int initailFrame;
  //関数 変位(生成時を0とする時刻[frame])
  function<quat(int, Player&)> displacementFunc;
  //計算量の軽減のためのframe中の変位を保持
  quat displacement;
  //玉の半径(当たり判定/グラフィックに使用)
  double radius;
  double displayRadius;
  //色(RGB表現/vector<double>{red, green, blue})}
  vector<double> color;

  Bullet(int initailFrame_, function<quat(int, Player&)> displacementFunc_, double radius_, double displayRadius_, vector<double> color_) : initailFrame(initailFrame_), displacementFunc(displacementFunc_), displacement((displacementFunc_)(0,player)), radius(radius_), displayRadius(displayRadius_), color(color_) {}

  void updateDisplacement(int frame, Player& player){
    displacement=(displacementFunc)(frame-initailFrame, player);
  }

  //bulletがplayerに当たったか判定
  bool hasHitPlayer(auto& player){
    if(::abs(displacement.w)>1e-15){return false;}
    if((displacement-player.displacement).abs()<radius){
      return true;
    }
    return false;
  }
  
  //bulletを描画
  void display(auto& player){
    if(::abs(displacement.w)>1e-15){return;}
    vector<int> p=displacement.projection(player);
    double d=(displacement-player.displacement).abs();
    if(d==0)return;
    //距離による色の変化
    double H=min({d*d,240.0});
    double max=0.8,min=0.5;
    double mid=(1.0-abs(fmod(H/60.0, 2.0)-1.0))*(max-min)+min;
    H<60 ? color={max,mid,min} : H<120 ? color={mid,max,min} : H<180 ? color={min,max,mid} : H<240 ? color={min,mid,max} : H<300 ? color={mid,min,max} : color={max,min,mid};
    glColor3d(color[0], color[1], color[2]);
    glRectd(p[0]-windowWidth*9/16*displayRadius/d-0.5, p[1]-windowHeight*displayRadius/d-0.5, p[0]+windowWidth*9/16*displayRadius/d+1, p[1]+windowHeight*displayRadius/d+1);
  }
};

bool operator<(const Bullet& b1, const Bullet& b2){
  quat d1=b1.displacement,d2=b2.displacement;
  return (d1-player.displacement).abs() < (d2-player.displacement).abs();
};

class Enemy{
  public:
  //変位(純四元数のベクトル部で表現)
  quat displacement;
  //移動速度[/frame]
  quat velocity;
  //移動の継続時間[frame]
  int movingFrame;
  //体力
  double hp;
  double radius;
  double difense;
  //フェーズが進むごとに減少させ-1でステージを移行
  int phase;

  void init(quat displacement_, quat velocity_, int movingFrame_, double hp_, double radius_, double difense_, int phase_){
    displacement=displacement_;
    velocity=velocity_;
    movingFrame=movingFrame_;
    hp=hp_;
    radius=radius_;
    difense=difense_;
    phase=phase_;
  }

  void damage(Shot& shot){
    double d=(displacement-shot.initialDisplacement).abs();
    //ダメージを 距離の3乗+定数 に反比例させることでプレイヤーが距離を取りすぎることを防止
    hp-=10000*shot.power/difense/(d*d*d+10000);
  }

  void nextPhase(){
    phase--;
    hp+=30;
    bullets.clear();
    bulletItrs.clear();
    shots.clear();
    phaseFrame=0;
  }

  void nextFrame(){
    if(movingFrame>0){
      displacement=displacement+velocity;
    }
    //todo erase
    if(difficulty!=3)difficulty=1;
    switch(difficulty*100+stageId*10+phase){
      case 300 ... 399:
        difense=8;
        if(phaseFrame%300==0){
          movingFrame=60;
          velocity=(1.0/60.0)*(player.displacement+10*player.direction-player.yawAxis-displacement+RAND_QUAT(1.5));
        }if(phaseFrame%300>60)
        for(int _=0;_<10;_++){
          quat q=RAND_QUAT(5);
          quat d=displacement+RAND_QUAT(1);
          function<quat(int, Player&)> f=[q,d](int t, Player& player){return t/60.0*q+d;};
          bullets.push_back(Bullet{totalFrame,f,0.2,0.1,{1,0.2,0.2}});
          bulletItrs.push_back(--bullets.end());
        }
        break;
      case 101:
        difense=6;
        if(phaseFrame%600==0){
          movingFrame=60;
          velocity=(1.0/60.0)*(player.displacement+10*player.direction-player.yawAxis-displacement+RAND_QUAT(1.5));
        }
        if(phaseFrame%600==300){
          movingFrame=60;
          velocity=(1.0/60.0)*(player.displacement+12.5*player.direction-player.yawAxis-displacement+RAND_QUAT(1.5));
        }
        if(phaseFrame%600>=90 && phaseFrame%600<234 && (phaseFrame-90)%48==0){
          for(int i=0;i<12;i++){
            quat d=displacement+(phaseFrame%600-90+40)/40*player.yawAxis.rotate(player.direction,PI*i/6);
            int p=(phaseFrame%900-90)/48+i*4;
            function<quat(int, Player&)> f=[d,p](int t, Player& player){
              if(t<p)return quat{1<<20,1<<20,1<<20,1<<20};
              if(t<160)return d;
              if(t>160)return quat{1<<20,1<<20,1<<20,1<<20};
              quat q=(player.displacement-d).unit();
              function<quat(int, Player&)> g=[q,d](int t, Player& player){return t/(4.0)*q+d;};
              bullets.push_back(Bullet{totalFrame,g,0.2,0.1,{1,0.2,0.2}});
              bulletItrs.push_back(--bullets.end());
              return quat{1<<20,1<<20,1<<20,1<<20};
            };
            bullets.push_back(Bullet{totalFrame,f,0.2,0.1,{1,0.2,0.2}});
            bulletItrs.push_back(--bullets.end());
          }
        }
        if(phaseFrame%600>=390 && phaseFrame%600<510 && (phaseFrame-390)%40==0){
          for(int _=0;_<10;_++){
            quat d=displacement;
            quat v=(player.displacement-displacement).unit()+quat{0,RAND_DOUBLE*0.2-0.1,RAND_DOUBLE*0.2-0.1,RAND_DOUBLE*0.2-0.1};
            function<quat(int, Player&)> f=[v,d](int t, Player& player){
              return t/4.0*v+d;
            };
            bullets.push_back(Bullet{totalFrame,f,0.17,0.07,{1,0.2,0.2}});
            bulletItrs.push_back(--bullets.end());
          }
        }
        break;
      case 100:
        difense=9;
        if(phaseFrame%300==10){
          movingFrame=60;
          velocity=(1.0/60.0)*(player.displacement+13*player.direction-player.yawAxis-displacement+RAND_QUAT(1.5));
        }
        if(phaseFrame%300>=100){
          quat d=displacement;
          quat v=(player.displacement-displacement).unit();
          if((phaseFrame-100)/40%3==0) v=v.rotate(quat{0,1,0,0},((phaseFrame-100)%40)*PI/20);
          else if((phaseFrame-100)/40%3==1) v=v.rotate(quat{0,0,1,0},((phaseFrame-100)%40)*PI/20);
          else v=v.rotate(quat{0,0,0,1},((phaseFrame-100)%40)*PI/20);
          function<quat(int, Player&)> f=[v,d](int t, Player& player){
            return t/4.0*v+d;
          };
          bullets.push_back(Bullet{totalFrame,f,0.17,0.07,{1,0.2,0.2}});
          bulletItrs.push_back(--bullets.end());
        }
        break;
      case 110:
        difense=8;
        if(phaseFrame%300==0){
          movingFrame=60;
          velocity=(1.0/60.0)*(player.displacement+10*player.direction-player.yawAxis-displacement+RAND_QUAT(1.5));
        }if(phaseFrame%300>60)
        for(int _=0;_<10;_++){
          quat q=RAND_QUAT(5);
          quat d=displacement+RAND_QUAT(1);
          function<quat(int, Player&)> f=[q,d](int t, Player& player){return t/60.0*q+d;};
          bullets.push_back(Bullet{totalFrame,f,0.2,0.1,{1,0.2,0.2}});
          bulletItrs.push_back(--bullets.end());
        }
        break;
      //case 110:
        if(phaseFrame%15!=13&&phaseFrame%15!=14){
          quat q=(player.displacement-displacement).unit().rotate(player.yawAxis,PI/24*((vector<int>{-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,0,0,8,7,6,5,4,3,2,1,0,-1,-2,-3,-4,0,0})[phaseFrame%30]));
          quat d=displacement;
          for(int i=-10;i<=10;i++){
            function<quat(int, Player&)> f=[q,d,i](int t, Player& player){return t/10.0*q+d+i/5.0*player.yawAxis;};
            bullets.push_back(Bullet{totalFrame,f,0.12,0.02,{1,0.2,0.2}});
            bulletItrs.push_back(--bullets.end());
          }
        }
        break;
      default:
        difense=4;
        quat q=(player.displacement-displacement).unit();
        quat d=displacement;
        function<quat(int, Player&)> f=[q,d](int t, Player& player){return t/10.0*q+d;};
        bullets.push_back(Bullet{totalFrame,f,0.1,0.1,{1,0.2,0.2}});
        bulletItrs.push_back(--bullets.end());
        break;
    }
  }

  string status(){
    string res="Stage : "+to_string(stageId+1)+" /"+to_string(numOfStages)+"  HP [ ";
    for(int i=0;i<30;i++){
      if(hp>i){
        res+='#';
      }else{
        res+='.';
      }
    }
    res+=" ] * ";
    res+=to_string(phase);
    return res;
  }

  void display(auto& player){
    vector<int> p=displacement.projection(player);
    double d=(displacement-player.displacement).abs();
    glColor3d(0.8,0.8,0.8);
    glRectd(p[0]-windowWidth*9/16*radius/d-0.5, p[1]-windowHeight*radius/d-0.5, p[0]+windowWidth*9/16*radius/d+1, p[1]+windowHeight*radius/d+1);
  }
  void putDirectionMarker(auto& player){
    vector<int> p=displacement.projection(player);
    glColor3d(0,0,0);
    glRectd(p[0]-1, p[1]-1, p[0]+1, p[1]+1);
  }
};

Enemy enemy{{0,0,-1,-10},{0,0,0,0},0,30,0.2,10,1};

class Stage{
  public:
  static void nextFrame(){
    if(enemy.hp<=0){
      if(enemy.phase==0){
        score+=(stageId+1)*(stageId+1)*sqrt(difficulty+1)*350000000/totalFrame;
        if(stageId+1>=numOfStages)gameStatus=2; else gameStatus=4;
      }else{
        enemy.nextPhase();
      }
    }
    totalFrame++;
    phaseFrame++;
    player.shotCoolDown--;
    player.invincible--;
    enemy.movingFrame--;
    int firstNumOfBullets=bullets.size();
    for(Bullet& bullet:bullets)bullet.updateDisplacement(totalFrame, player);
    for(Shot& shot:shots)shot.updateDisplacement(totalFrame);

    enemy.nextFrame();
    //todo
    for(int i=1, end=bullets.size(); i<end; i++){
      if(i<1)continue;
      //sort
      for(int j=i; j>0&& (bulletItrs[j-1]->displacement-player.displacement).abs() < (bulletItrs[j]->displacement-player.displacement).abs(); j--)swap(bulletItrs[j-1],bulletItrs[j]);
    }
    //処理落ち防止のため敵弾をmaxNumOfBullet発に制限
    for(;bulletItrs.size()>maxNumOfBullet;){
      bullets.erase(*(bulletItrs.begin()));
      bulletItrs.erase(bulletItrs.begin());
    }
  }

  //ゲーム画面を表示
  static void display(){
    glClear(GL_COLOR_BUFFER_BIT);
    //敵を表示
    enemy.display(player);
    //敵弾を表示,接触を判定
    for(std::_List_iterator<Bullet> bulletItr:bulletItrs){
      if(bulletItr->hasHitPlayer(player)&&player.invincible<=0){
        player.death();
        break;
      }
      bulletItr->display(player);
    }
    //自機弾の表示,接触を判定
    for(int i=0, end=shots.size(); i<end; i++){
      Shot shot=shots[i];
      if(shot.hasHitEnemy(enemy)){
        shots.erase(shots.begin()+i);
        i--;end--;
        enemy.damage(shot);
      }
      shot.display(player);
    }
    enemy.putDirectionMarker(player);
    //ステータスの表示
    glColor3d(1, 1, 1);
    glRasterPos2i(10,10);
    for(char c:player.status()){
      glutBitmapCharacter(GLUT_BITMAP_9_BY_15, c);
    }
    glRasterPos2i(10,windowHeight-30);
    for(char c:enemy.status()){
      glutBitmapCharacter(GLUT_BITMAP_9_BY_15, c);
    }
    glutSwapBuffers();
  }

  static void nextStage(){
    stageId++;
    player.displacement={0,0,0,0};
    player.direction={0,0,0,-1};
    player.yawAxis={0,0,1,0};
    enemy=Enemy{{0,0,-1,-10},{0,0,0,0},0,30,0.2,10,numsOfPhese[stageId]};
    init();
    player.invincible=60;
  }

  static void init(){
    totalFrame=0;
    phaseFrame=0;
    shots.clear();
    bullets.clear();
    bulletItrs.clear();
  }
};

vector<bool> areKeysPressed(255, false);

class GameRoot{
  public:
  void initGame(int difficulty_){
    gameStatus=0;
    stageId=0;
    score=0;
    difficulty=difficulty_;
    player.init({0,0,0,0},{0,0,0,-1},0.07,{0,0,1,0},PI/240,0,3,6,0,60);
    enemy.init({0,0,-1,-10},{0,0,0,0},0,30,0.2,10,1);
    Stage::init();
  }

  void display(){
    if(gameStatus==0){
      Stage::display();
    }else if(gameStatus==1){
      glClear(GL_COLOR_BUFFER_BIT);
      glColor3d(1, 1, 1);
      vector<string> menuStrings={"Product-alpha","        created by Wiiiiam","","","    Press '0' to start","","    Press 'esc' to quit"};
      for(int i=0;i<menuStrings.size();i++){
        glRasterPos2i(windowWidth/2-150,windowHeight/2+100-i*30);
        for(char c:menuStrings[i]){
          glutBitmapCharacter(GLUT_BITMAP_9_BY_15, c);
        }
      }
      glutSwapBuffers();
      
      if(areKeysPressed['0']){initGame(0);return;}//\
      if(areKeysPressed['1']){initGame(1);return;}\
      if(areKeysPressed['2']){initGame(2);return;}\
      if(areKeysPressed['3']){initGame(3);return;}
    }else if(gameStatus==2 || gameStatus==3){
      glClear(GL_COLOR_BUFFER_BIT);
      if(gameStatus==2) Stage::display();
      glColor3d(1, 1, 1);
      vector<string> menuStrings={gameStatus==2?"Game Completed":"Game Over","","score : "+to_string(score+sqrt(difficulty+1)*10000000*player.life),"","","Press 'c' to continue..."};
      for(int i=0;i<menuStrings.size();i++){
        glRasterPos2i(windowWidth/2-150,windowHeight/2+100-i*30);
        for(char c:menuStrings[i]){
          glutBitmapCharacter(GLUT_BITMAP_9_BY_15, c);
        }
      }
      glutSwapBuffers();
      if(areKeysPressed['c']){gameStatus=1;return;}
    }else if(gameStatus==4){
      glClear(GL_COLOR_BUFFER_BIT);
      Stage::display();
      glColor3d(1, 1, 1);
      vector<string> menuStrings={"Stage "+to_string(stageId+1)+" Completed","","score : "+to_string(score),"","","Press 'c' to continue..."};
      for(int i=0;i<menuStrings.size();i++){
        glRasterPos2i(windowWidth/2-150,windowHeight/2+100-i*30);
        for(char c:menuStrings[i]){
          glutBitmapCharacter(GLUT_BITMAP_9_BY_15, c);
        }
      }
      glutSwapBuffers();
      if(areKeysPressed['c']){
        gameStatus=0;
        Stage::nextStage();
        return;
      }
    }
  }

};

GameRoot game;

class Gl{
  static void display(){
    game.display();
  }

  //windowサイズが変更された時の処理
  static void reshape(int width, int height){
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, (double)width, 0.0, (double)height);
    windowWidth=width;
    windowHeight=height;
  }

  static void keyDown(unsigned char keyCode,int x,int y){
    areKeysPressed[keyCode]=true;
  }
  static void keyUp(unsigned char keyCode,int x,int y){
    areKeysPressed[keyCode]=false;
  }
  static void FancCalledEveryFrame(int v){
    //timerの設定
    LARGE_INTEGER freq,start,now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&start);
    
    if(gameStatus==0){
      Stage::nextFrame();
      if(areKeysPressed['4'])player.cameraRotate(-PI);
      if(areKeysPressed['6'])player.cameraRotate(0.0);
      if(areKeysPressed['8'])player.cameraRotate(PI/2);
      if(areKeysPressed['5'])player.cameraRotate(PI*3/2);
      if(areKeysPressed['j'])player.cameraRotate(-PI);
      if(areKeysPressed['l'])player.cameraRotate(0.0);
      if(areKeysPressed['i'])player.cameraRotate(PI/2);
      if(areKeysPressed['k'])player.cameraRotate(PI*3/2);
      if(areKeysPressed['a'])player.pressedLeftKey();
      if(areKeysPressed['d'])player.pressedRightKey();
      if(areKeysPressed['w'])player.pressedUpKey();
      if(areKeysPressed['s'])player.pressedDownKey();
    //if(areKeysPressed['e'])player.pressedAdvanceKey();
    //if(areKeysPressed['q'])player.pressedRetreatKey();
      player.pressedShotKey();
      if(areKeysPressed['x'])player.pressedBombKey();
      if(areKeysPressed[32])player.pressedBombKey();
    }
    if(areKeysPressed[27])exit(0);
    Gl::display();
    //60分の1秒が経過するまで待機
    QueryPerformanceCounter(&now);
    double elapsedTime=static_cast<double>(now.QuadPart-start.QuadPart)/freq.QuadPart;
    printf("%f %d\n",elapsedTime*60.0,bullets.size());
    for(;elapsedTime<1.0/60.1;){
      QueryPerformanceCounter(&now);
      elapsedTime=static_cast<double>(now.QuadPart-start.QuadPart)/freq.QuadPart;
    }
    glutTimerFunc(0, Gl::FancCalledEveryFrame, 0);
  }

  public:
  static void init(int argc, char **argv){
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE);
    glutGameModeString("1600x900:32@60");
  	glutEnterGameMode();
    glutSetCursor(GLUT_CURSOR_NONE);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glutDisplayFunc(Gl::display);
    glutReshapeFunc(Gl::reshape);
    glutKeyboardFunc(Gl::keyDown);
    glutKeyboardUpFunc(Gl::keyUp);
    Gl::FancCalledEveryFrame(0);
    glutMainLoop();
  }
};

signed main(int argc, char **argv){
  std::cin.tie(0)->sync_with_stdio(0);
  PlaySound(TEXT("src\\bgm.wav"), NULL, SND_ASYNC|SND_FILENAME|SND_NOSTOP|SND_NODEFAULT|SND_LOOP);
  Gl::init(argc, argv);
  return 0;
}