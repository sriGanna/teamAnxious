/**
 **********************************************************************************************************************
 * @file       app.pde
 * @author     Raquel R, Sri G, Unma D
 * @version    V1.0.0
 * @date       01-Apr-2021
 * @brief      Haptic Reflective Journal
 **********************************************************************************************************************
 * @attention
 *
 *
 ***********************************************************************************************************************/
 
 
/* library imports *****************************************************************************************************/
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
import processing.sound.*;
import controlP5.*;

SoundFile file;
SoundFile file2;
ControlP5 cp5;
/* end library imports *************************************************************************************************/



/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/



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
PVector           posWall                             = new PVector(0.01, 0.1);

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
float             worldHeight                         = 21.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;

/* define maze blocks */
FBox[] walls = new FBox[28];
PShape wall;

FBox[][]              spacedWalls=new FBox[4][13];
float wallW = 0.6;

float w1=4;
float w2=9.4;
float w3=13.2;

/*Define circles*/
FCircle           circle2;
FCircle           bubble;
FPoly             next, prev;

/* define start and stop button */
FCircle           c1, c2, c3, c4, c5, c;


/* Define Blob Variables*/
FBlob             f;

/*Bubble Pop Variables*/
FBox[] popWall = new FBox[3];
PImage            haplyAvatar, bubbleImg, tomato;
boolean done=false;
int prevMode = 0;
boolean modeChange = false;
boolean touched =false;

/* Timer variables */
long currentMillis = millis();
long previousMillis = 0;
float interval = 10;



/* define game start */


int               scene                               =0;
int sceneNum =5;

/* text font */
PFont             F;

boolean DEBUG = false;
boolean DEBUGSCENE = false;

/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1100, 800);

  /* set font type and size */
  F                   = createFont("Arial", 16, true);

  /* create pantagraph graphics */

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

  /* Insert Drawings for Mode 3 Here */
  deviceOrigin.add(worldPixelWidth/2, 0);
  //create_pantagraph();


  /*  prevnext buttons */
  createControls();

  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(255, 0, 0); 


  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);


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
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    hapticSimulationStep();
    //trackScene();
  }
}
/* end simulation section **********************************************************************************************/
/* helper functions section, place helper functions here ***************************************************************/


void playAudio() {
  if (done==false)
  {
    file.play();
    if (DEBUG) {
      print("here");
    }
  }
}



void damp(float dampingVal) {
  s.h_avatar.setDamping(dampingVal);
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
void hapticSimulationStep() {
  /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */

  renderingForce = true;

  if (haplyBoard.data_available()) {
    getEndEffectorState();
  }

  s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7);


  s.updateCouplingForce();
  if (scene != 2) {
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
  }
  torques.set(widgetOne.set_device_torques(fEE.array()));
  widgetOne.device_write_torques();


  world.step(1.0f/1000.0f);

  renderingForce = false;
}

void getEndEffectorState() {
  /* GET END-EFFECTOR STATE (TASK SPACE) */
  widgetOne.device_read_data();



  if (scene ==2) {
    angles.set(widgetOne.get_device_angles()); 
    posEE.set(widgetOne.get_device_position(angles.array()));

    /* haptic wall force calculation */
    fWall.set(0, 0);

    penWall.set(0, (posWall.y - (posEE.y + rEE)));

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



void createControls() {
  cp5 = new ControlP5(this);

  PFont p = createFont("Verdana", 17); 
  ControlFont font = new ControlFont(p);

  // change the original colors
  cp5.setColorForeground(color(0, 0, 0));
  cp5.setColorBackground(color(0, 0, 0));
  cp5.setFont(font);

  cp5.addButton("red")
    .setLabel("red")
    .setPosition(worldWidth*40, 100)
    .setSize(100, 50)
    .setColorBackground(color(255, 0, 0))

    ;
  cp5.addButton("orange")
    .setLabel("orange")
    .setPosition(980, 150)
    .setSize(100, 50)
    .setColorBackground(color(255, 128, 0))

    ;
  cp5.addButton("yellow")
    .setLabel("yellow")
    .setPosition(980, 200)
    .setSize(100, 50)
    .setColorBackground(color(255, 255, 0))

    ;
  cp5.addButton("green")
    .setLabel("green")
    .setPosition(980, 250)
    .setSize(100, 50)
    .setColorBackground(color(0, 255, 0))

    ;
  cp5.addButton("lBlue")
    .setLabel("light-blue")
    .setPosition(980, 300)
    .setSize(100, 50)
    .setColorBackground(color(0, 128, 255))

    ;
  cp5.addButton("blue")
    .setLabel("blue")
    .setPosition(980, 350)
    .setSize(100, 50)
    .setColorBackground(color(0, 0, 255))

    ;
  cp5.addButton("purple")
    .setLabel("purple")
    .setPosition(980, 400)
    .setSize(100, 50)
    .setColorBackground(color(255, 0, 255))

    ;
  cp5.addButton("save")
    .setLabel("save")
    .setPosition(980, 650)
    .setSize(100, 50)
    .setColorBackground(color(255, 0, 255))

    ;
}

void changeScene(String button) {
  if (button == "next") {
  } else if (button =="prev") {
  }
}

void trackScene() {
  if (scene == sceneNum) {
    world.remove(next);
  }
  if (scene==0) {
    world.remove(prev);
  }
  if (s.h_avatar.isTouchingBody(next) && !touched) {
    scene += 1;
    touched = true;
  } else if (s.h_avatar.isTouchingBody(prev)&& !touched) {
    scene -=1;
  } else {
    touched = false;
  }
  if (DEBUGSCENE) {
    println(scene);
  }
}

/* end helper functions section ****************************************************************************************/
