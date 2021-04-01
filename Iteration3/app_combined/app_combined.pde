/**
 **********************************************************************************************************************
 * @file       sketch_4_Wall_Physics.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V4.1.0
 * @date       08-January-2021
 * @brief      wall haptic example using 2D physics engine 
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
import processing.sound.*;

import java.util.*;
import controlP5.*;


SoundFile file;
ControlP5 cp5;
/* end library imports *************************************************************************************************/



/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/

boolean DEBUG = false;
boolean DEBUGPOS = false;
boolean DEBUGREL = false;
boolean DEBUGSPEED = true;
boolean DEBUGAUDIO = false;
boolean DEBUGPOP = true;

/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                      = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/
/* elements definition *************************************************************************************************/
/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* end effector radius in meters */
float             rEE                                 = 0.006;

/* virtual wall parameter  */
float             kWall                               = 2000;
PVector           fWall                               = new PVector(0, 0);
PVector           penWall                             = new PVector(0, 0);
PVector           posWall                             = new PVector(0.01, 0.08);

/* pantagraph link parameters in meters */
float             l                                   = 0.07;
float             L                                   = 0.09;


/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

final int         worldPixelWidth                     = 1280;
final int         worldPixelHeight                    = 820;
PShape pGraph, joint, endEffector;

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 18.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;


/* Initialization of elements */
FCircle           circle1, bbody;
FBox            anchor1, anchor2;
FDistanceJoint    joint1, joint2;
FCircle select, balloon, sqCirc1, sqCirc2;
FBlob squish1, squish2;

PShape wall;
FCircle[] bubbles = new FCircle[28];
float colour_inc=0;
float colR, colG, colB;
float currentPosY;
int bubbleQuant = 4;
double speed;

//bubble locations
int[] xCord={5, 7, 10, 13, 16, 18, 20, 22, 24, 26};
int[] yCord={5, 14, 8, 14, 7, 16, 5, 12, 16, 4};
float[] rad={4, 3, 2, 4, 3, 3, 2, 3, 2, 4};
FCircle[] bub=new FCircle[10];

//color codes
int[] r=new int[10];
int[] g=new int[10];
int[] b=new int[10];
int c1, c2, c3;

ArrayList<FBody> isTouching;

/* Initialization of virtual tool */
PImage            colour;
PGraphics output;
PFont             F;

/* end elements definition *********************************************************************************************/

boolean done=false;
ArrayList <Splat> splats = new ArrayList <Splat> ();
boolean splatshown=false;
boolean selectCol = true;
boolean redraw = false;
boolean wasPulled = false;
boolean released = false;
boolean loadBalloon = false;
boolean Fisica = true;
/* end elements definition *********************************************************************************************/

int scene =0;
int sceneNum =3;

/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */
  file = new SoundFile(this, "splat.mp3");


  /* set font type and size */
  F                   = createFont("Arial", 16, true);

  /* screen size definition */
  size(1100, 700);

  /* device setup */

  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, "COM4", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();

  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);

  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);


  widgetOne.device_set_parameters();

  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();


  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4);  
  s.h_avatar.setStroke(0);
  s.h_avatar.setFill(255);
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

  //createSling();
  createControls();
  //createBubbles();

  //cp5.setColorActive(0xffff0000);


  wall = create_wall(posWall.x-0.2, posWall.y+rEE+.01, posWall.x+0.2, posWall.y+rEE+.01);

  /* world conditions setup */
  world.setGravity((0.0), (6000.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(0.4);
  world.setEdgesFriction(1.2);

  output = createGraphics(800, 800, JAVA2D);
  world.draw();



  /* setup framerate speed */
  frameRate(baseFrameRate);


  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw() {
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if (renderingForce == false) {
    background(255);
    updateTitle();
    image(output, 0, 0);
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/


/* Timer variables */
long currentMillis = millis();
long previousMillis = 0;
float interval = 50;

/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    getEndEffectorState();
    sceneActions();
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/
void playAudio() {
  if (done==false)
  {
    file.play();
    if (DEBUGAUDIO) {
      print("here");
    }
  }
}

class Splat {
   float x, y;
  int i;
  float radi;
  PGraphics splat;
  boolean done;
  boolean z;

  Splat(float x, float y, int i) {
    this.x = x;
    this.y = y;
    this.i = i;
    if (scene ==1) {
      rad = i;
    } else if (scene ==2) {
      radi = rad[i]*10;
    }
    splat = createGraphics(200, 200, JAVA2D);
    create();
  }

  void create() {
    splat.beginDraw();
    splat.smooth();
    if (scene ==1) {
      splat.colorMode(HSB, 360, 100, 100);
      splat.fill(s.h_avatar.getFillColor());
    } else if (scene ==2) {
      splat.colorMode(RGB, 255);
      splat.fill(r[i], g[i], b[i]);
    }
    splat.noStroke();
    for (float i=3; i<29; i+=.35) {
      float angle = random(0, TWO_PI);
      float splatX = (splat.width-50)/2 + 25 + cos(angle)*2*i;
      float splatY = (splat.height-50)/2 + 25 + sin(angle)*3*i;
      splat.ellipse(splatX, splatY, rad-i, rad-i+1.8);
    }
    splat.endDraw();
  }
  void display() {
    output.beginDraw();
    output.imageMode(CENTER);
    output.image(splat, x, y);
    output.endDraw();
  }
  void saveSplat() {
    splat.save("./saved/test.png");
  }
}

PVector device_to_graphics(PVector deviceFrame) {
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame) {
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}

PShape create_wall(float x1, float y1, float x2, float y2) {
  x1 = pixelsPerMeter * x1;
  y1 = pixelsPerMeter * y1;
  x2 = pixelsPerMeter * x2;
  y2 = pixelsPerMeter * y2;

  return createShape(LINE, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
}

void createSling() {

  anchor1              = new FBox(1, 1);
  anchor1.setFill(0);
  anchor1.setPosition(2, 12);
  anchor1.setStatic(true);
  world.add(anchor1);

  anchor2              = new FBox(1, 1);
  anchor2.setFill(0);
  anchor2.setPosition(22, 12);
  anchor2.setStatic(true);
  world.add(anchor2);

  balloon                   = new FCircle(1);
  balloon.setPosition(s.h_avatar.getX(), s.h_avatar.getY());
  balloon.setStatic(true);
  balloon.setSensor(true);
  balloon.setFill(0);
  balloon.setStroke(0);
  //world.add(balloon);

  joint1 = new FDistanceJoint(anchor1, s.h_avatar);
  world.add(joint1);

  joint2 = new FDistanceJoint(anchor2, s.h_avatar);
  world.add(joint2);
}

void removeSling() {
  world.remove(joint1);
  world.remove(joint2);
  world.remove(anchor1);
  world.remove(anchor2);
  delay(500);
  save("./saved/test.png");
  delay(100);
}

void createControls() {
  cp5 = new ControlP5(this);

  PFont p = createFont("Verdana", 17); 
  ControlFont font = new ControlFont(p);

  // change the original colors
  cp5.setColorForeground(color(0, 0, 0));
  cp5.setColorBackground(color(0, 0, 0));
  cp5.setFont(font);

  cp5.addButton("Next")
    .setLabel("Next")
    .setPosition(980, 100)
    .setSize(100, 50)
    .setColorBackground(color(255, 0, 0))

    ;
  cp5.addButton("Prev")
    .setLabel("Prev")
    .setPosition(980, 200)
    .setSize(100, 50)
    .setColorBackground(color(255, 128, 0))

    ;

  cp5.addButton("save")
    .setLabel("save")
    .setPosition(980, 650)
    .setSize(100, 50)
    .setColorBackground(color(255, 0, 255))

    ;
}

void controlEvent(CallbackEvent event) {
  if (event.getAction() == ControlP5.ACTION_CLICK) {
    switch(event.getController().getAddress()) {
    case "/Next":
      if (scene<sceneNum+1) {
        scene++;
        updateScene();
      } else {
        scene = sceneNum;
      }
      break;
    case "/Prev":
      if (scene >0) {
        scene--;
        updateScene();
      } else {
        scene =0;
      }
      break;
    case "/save":
      output.save("./saved/test.png");
      break;
    }
  }
}

void checkSplat() {

  isTouching = s.h_avatar.getTouching();
  if (DEBUG) {
    println(isTouching);
  }
  for (int i =0; i<10; i++) {
    if (isTouching.contains(bub[i])) { 
      currentMillis = millis();
      if (currentMillis - previousMillis > interval) {         
        splatshown = false;
        animateSplat(bub[i], i); 
        previousMillis = millis();
      }
    }
  }
}

void animateSplat(FCircle bubble, int i) {
  playAudio();
  if (splatshown == false) {
    splats.add(new Splat(bubble.getX()*40, bubble.getY()*40, i));
    if (DEBUGPOS) {
      println(bubble.getX());
      println(bubble.getY());
    }
    splatshown = true;
    world.remove(bubble);
  }
}

void keyPressed() {
  if (key == 'q') {
    selectCol = false;
  }
  if (key == 'w') {
    selectCol = true;
  }
  if (key == 'r') {
    loadBalloon = true;
    if (DEBUG) {
      println("balloon loaded");
    }
  }
}


boolean pulledBack() {
  currentPosY = s.h_avatar.getY()/150;
  if (currentPosY > .1) {
    return true;
  } else {
    return false;
  }
}
float x_vel, y_vel;
boolean isReleased() {

  if (wasPulled & !pulledBack()) {
    wasPulled = false;
    released = true;
    x_vel = s.h_avatar.getVelocityX();
    y_vel = s.h_avatar.getVelocityY();
    speed = Math.sqrt(Math.pow(x_vel, 2) + Math.pow(y_vel, 2));
    if (DEBUGSPEED) {
      println(speed);
    }
    return true;
  } else 
  {
    return false;
  }
}

void drawSplat(double speed)
{
  released = false;
  if (splatshown == false) {
    if (speed <70) {
      splats.add(new Splat(s.h_avatar.getX()*40, s.h_avatar.getY()*40, 10));
    } else if (speed <110) {
      splats.add(new Splat(s.h_avatar.getX()*40, s.h_avatar.getY()*40, 18));
    } else {
      splats.add(new Splat(s.h_avatar.getX()*40, s.h_avatar.getY()*40, 22));
    }
    splats.get(splats.size()-1).display();
    playAudio();
    if (DEBUGPOS) {
      println(s.h_avatar.getX());
      println(s.h_avatar.getY());
    }
    splatshown = true;
  }
}

boolean isMoving() {

  if (abs(s.h_avatar.getVelocityX())<.05 && abs(s.h_avatar.getVelocityY())<.05) {
    return false;
  } else {
    return true;
  }
}
void updateScene() {
  clearAll();
  if (scene ==1) {
    startSling();
    s.h_avatar.setFill(0, 255, 0);
    Fisica = false;
  } else if (scene ==2) {
    startPop();
    Fisica = true;
  } else if (scene ==3) {
    startSquish();
    Fisica = true;
  }
  updateTitle();
}

void updateTitle() {
  textAlign(CENTER);

  if (scene ==1) {
    text("Slingshot", width/4, 70);
  } else if (scene ==2) {
    text("Bubbles", width/4, 70);
  } else if (scene ==3) {
    text("Squish", width/4, 70);
  } else {
    text("choose a scene", width/4, 70);
  }
  textFont(F, 22);
}

void clearAll() {
  removeSling();
  removePop();
  removeSquish();
}
void startSling() {
  createSling();
  createPalette();
}

void startPop() {
  drawBub();
}

void startSquish() {
  drawBlob(squish1, 25, 20, 21, 70);
  drawBlob(squish2, 10, 20, 21, 70);
  drawCircle(sqCirc1, 20, edgeTopLeftX+worldWidth/1.3-3, edgeTopLeftY+2*worldHeight/6.0+11);
  drawCircle(sqCirc2, 22, edgeTopLeftX+worldWidth/1.3-16, edgeTopLeftY+2*worldHeight/6.0+12);
}

void createPalette() {
}

void removePop() {
  splats.clear();
  removeBub();
}

void removeSquish() {
  world.remove(squish1);
  world.remove(squish2);
  world.remove(sqCirc1);
  world.remove(sqCirc2);
}

void drawBub() {
  for (int i=0; i<10; i++) {
    randomize();
    r[i]=c1;
    g[i]=c2;
    b[i]=c3;
  }
  for (int i=0; i<10; i++)
  {
    world.remove(bub[i]);
    bub[i] = new FCircle(rad[i]);
    bub[i].setPosition(xCord[i], yCord[i]);
    bub[i].setStatic(true);
    bub[i].setFill(r[i], g[i], b[i]);
    bub[i].setNoStroke();
    world.add(bub[i]);
  }
}
void removeBub() {
  for (int i=0; i<10; i++)
  {
    world.remove(bub[i]);
  }
}

void randomize() {
  c1=int(random(255));
  c2=int(random(255));
  c3=int(random(255));
}

void drawBlob(FBlob f, int x, int y, int v, int z) {
  f                   = new FBlob();
  f.setAsCircle(x, y, v, z);
  f.setStroke(0);
  f.setStrokeWeight(2);
  //f.setFill(255);
  f.setStatic(true);
  f.setFriction(20);
  f.setDensity(100);
  f.setSensor(true);
  f.setFill(random(255), random(255), random(255));
  world.add(f);
}

void drawCircle(FCircle c, float size, float x, float y) {
  c                   = new FCircle(size);
  c.setPosition(x, y);
  c.setStatic(true);
  c.setSensor(true);
  c.setNoFill();
  c.setNoStroke();
  world.add(c);
}
void sceneActions() {
  if (scene == 1) {
    checkSling();
  } else if (scene ==2) {
    checkSplat();
  } else if (scene ==3) {
    checkSquish();
  }
}

void checkSling() {
  isReleased();
  if (DEBUGREL) {
    println(pulledBack());
  }
  if (pulledBack()) {
    wasPulled = true;
  }
  if (released && !isMoving()) {
    if (DEBUG) {
      println("drawing");
    }
    splatshown = false;
    drawSplat(speed);
  }
}

void checkSquish() {
  /* INSERT CODE HERE FOR SQUISH FORCE INTERACTION: */
}
void getEndEffectorState() {
  renderingForce = true;

  if (haplyBoard.data_available()) {
    /* GET END-EFFECTOR STATE (TASK SPACE) */
    widgetOne.device_read_data();
    angles.set(widgetOne.get_device_angles()); 
    posEE.set(widgetOne.get_device_position(angles.array()));
    if (!Fisica) {
      /* haptic wall force calculation */
      fWall.set(0, 0);

      penWall.set(0, (posWall.y - (posEE.y + rEE)));
      if (DEBUG) {
        println(penWall.y);
      }

      if (penWall.y < 0) {
        fWall = fWall.add(penWall.mult(-kWall));
      }

      fEE = (fWall.copy()).mult(-1);
      fEE.set(graphics_to_device(fEE));
      /* end haptic wall force calculation */
      posEE.set(posEE.copy().mult(175));
    } else {
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));
    }
  }
  s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7+6); 
  s.updateCouplingForce();
  if (Fisica) {
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
  }

  torques.set(widgetOne.set_device_torques(fEE.array()));
  widgetOne.device_write_torques();

  world.step(1.0f/1000.0f);
  renderingForce = false;
}
/* end helper functions section ****************************************************************************************/
