/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
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
import ddf.minim.*;

Minim minim;
AudioPlayer song;
SoundFile file;
SoundFile file2;
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
boolean           renderingForce                     = false;
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
PVector           posWall                             = new PVector(0.01, 0.10);

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
float             worldWidth                          = 32.0;  
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

/* Timer variables */
long currentMillis = millis();
long previousMillis = 0;
float interval = 10;



/* define game start */
boolean           gameStart                           =false;
boolean           reset                               =false;
int               mode                                =0;

/* text font */
PFont             F;

boolean DEBUG = true;

/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */
  file = new SoundFile(this, "pop1.wav");
  file2 = new SoundFile(this, "squish.mp3");
  /* screen size definition */
  size(1280, 820);

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

  /* create pantagraph graphics */
  wall = create_wall(posWall.x-0.2, posWall.y+rEE, posWall.x+0.2, posWall.y+rEE);
  wall.setStroke(color(0));
  /* End of Mode 2 Drawings */

  float wx1=7.18;
  float wx2=21.4;
  float wy1=3;
  float wy2=17.2;

  createWall(wx1, false, 0);
  resetW();
  createWall(wy1, true, 1);
  resetW();
  createWall(wx2, false, 2);
  resetW();
  createWall(wy2, true, 3);

  for (int i=0; i<4; i++)
  {
    for (int j=0; j<=12; j++)
    {
      if (i ==0 && j==2) {
        j++;
      }
      spacedWalls[i][j].setStatic(true);
      spacedWalls[i][j].setFill(0, 255, 255);
      spacedWalls[i][j].setNoStroke();
      spacedWalls[i][j].setForce(0, 0);
      world.add(spacedWalls[i][j]);
    }
  }  

  //circle
  circle2                   = new FCircle(1);
  circle2.setPosition(12, 7);
  circle2.setStatic(false);
  circle2.setDensity(4);
  circle2.setFill(255, 0, 0);
  circle2.setNoStroke();
  world.add(circle2);


  /* Insert Drawings for Mode 4 Here */
  c                   = new FCircle(20.0);
  c.setPosition(edgeTopLeftX+worldWidth/1.3-4, edgeTopLeftY+2*worldHeight/6.0-1);
  c.setStatic(true);
  c.setSensor(true);
  c.setNoFill();
  c.setNoStroke();
  world.add(c);

  f                   = new FBlob();
  f.setAsCircle(16, 7, 20, 70);
  f.setNoStroke();
  f.setStrokeWeight(2);
  f.setStatic(true);
  f.setFriction(20);
  f.setDensity(100);
  f.setSensor(true);
  f.setNoFill();
  //f.setFill(random(255), random(255), random(255));
  world.add(f);
  /* End of Mode 4 Drawings */

  /* Mode 5 Drawings */
  bubble                   = new FCircle(1.7);
  bubble.setPosition(8.5, 7);
  bubble.setStatic(true);
  bubble.setFill(255);
  bubble.setNoStroke();
  world.add(bubble);


  /* Mode 2 Button */

  c2                  = new FCircle(1.0);
  c2.setPosition(edgeTopLeftX+2, edgeTopLeftY+worldHeight/2.0-6);
  c2.setFill(200, 0, 0);
  c2.setStaticBody(true);
  c2.setSensor(true);
  world.add(c2);

  /* Mode 3 Button */
  c3                  = new FCircle(1.0);
  c3.setPosition(edgeTopLeftX+2, edgeTopLeftY+worldHeight/2.0-4);
  c3.setFill(0, 0, 200);
  c3.setStaticBody(true);
  c3.setSensor(true);
  world.add(c3);

  /* Mode 4 Button */
  c4                  = new FCircle(1.0);
  c4.setPosition(edgeTopLeftX+2, edgeTopLeftY+worldHeight/2.0-2);
  c4.setFill(100, 50, 150);
  c4.setStaticBody(true);
  c4.setSensor(true);
  world.add(c4);

  /* Mode 5 Button */
  c5                  = new FCircle(1.0);
  c5.setPosition(edgeTopLeftX+2, edgeTopLeftY+worldHeight/2.0);
  c5.setFill(50, 75, 20);
  c5.setStaticBody(true);
  c5.setSensor(true);
  world.add(c5);

  clearWalls();
  clearBlob();
  clearLine();
  clearBubble();

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
    fill(0, 0, 0);
    textAlign(CENTER);
    text("Touch a coloured circle to switch modes", width/4, 70);
    textFont(F, 22);
    if (mode ==2)
    {
      shape(wall);
    }
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    hapticSimulationStep();
    checkMode();
    runMode();
  }
}
/* end simulation section **********************************************************************************************/
/* helper functions section, place helper functions here ***************************************************************/

void createWall(float w, boolean isHorizontal, int wallNumber) {
  for (int j=0; j<=12; j++)
  {
    if (isHorizontal==true && j<4)
    {
      spacedWalls[wallNumber][j]=new FBox(wallW+0.2, 1);
      spacedWalls[wallNumber][j].setPosition(w1+4.2, w);      
      w1=w1+1.4;
    } else if (isHorizontal==true && j>=4 && j<8)
    {
      spacedWalls[wallNumber][j]=new FBox(wallW, 1);
      spacedWalls[wallNumber][j].setPosition(w2+4.2, w);     
      w2=w2+1;
    } else if (isHorizontal==true && j>=8 && j<=12)
    {
      spacedWalls[wallNumber][j]=new FBox(wallW-0.2, 1);
      spacedWalls[wallNumber][j].setPosition(w3+4.2, w);      
      w3=w3+0.8;
    } else if (isHorizontal==false && j<4)
    {
      spacedWalls[wallNumber][j]=new FBox(1, wallW+0.2);
      spacedWalls[wallNumber][j].setPosition(w, w1);  
      if (DEBUG) {
        println(w, w1);
      }
      w1=w1+1.4;
    } else if (isHorizontal==false && j>=4 && j<8)
    {
      spacedWalls[wallNumber][j]=new FBox(1, wallW);
      spacedWalls[wallNumber][j].setPosition(w, w2); 
      if (DEBUG) {
        //println(w,w2);
      }
      w2=w2+1;
    } else if (isHorizontal==false && j>=8 && j<=12)
    {
      spacedWalls[wallNumber][j]=new FBox(1, wallW-0.2);
      spacedWalls[wallNumber][j].setPosition(w, w3); 
      w3=w3+0.8;
      if (DEBUG) {
        //println(w,w3);
      }
    }
  }
}

void resetW() {
  w1=4;
  w2=9.4;
  w3=13.2;
  if (DEBUG) {
    //print("here" + w1+" "+w2+" "+w3);
  }
}
void playAudio() {
  if (done==false)
  {
    file.play();
    if (DEBUG) {
      print("here");
    }
  }
}
void checkMode() {
  if (s.h_avatar.isTouchingBody(c2)) {
    mode =2;
  } else if (s.h_avatar.isTouchingBody(c3)) {
    mode =3;
  } else if (s.h_avatar.isTouchingBody(c4)) {  
    mode = 4;
  } else if (s.h_avatar.isTouchingBody(c5)) {
    mode = 5;
  }
  if (mode != prevMode) {
    modeChange = true;
  } else {
    modeChange = false;
  }
  prevMode = mode;
}

void runMode() {
  if (modeChange) {
    switch(mode) {
    case 2:
      beginLine();
      clearWalls();
      clearBlob();
      clearBubble();
      if (DEBUG) {
        print("mode 2 running");
      }
      break;
    case 3:
      beginWalls();
      clearBubble();
      clearLine();
      clearBlob();
      if (DEBUG) {
        print("mode 3 running");
      }
      break;
    case 4:
      beginBlob();
      clearWalls();
      clearBubble();
      clearLine();
      if (DEBUG) {
        print("mode 4 running");
      }
      break;
    case 5:
      beginBubble();
      clearWalls();
      clearBlob();
      clearLine();
      if (DEBUG) {
        print("mode 5 running");
      }
      break;
    default: 
      //clearWalls();
      clearBlob();
      clearLine();
      clearBubble();
      break;
    }
  }
}

void beginLine() {
  shape(wall);
}
void clearLine() {
}

void beginBlob() {
  f.setStroke(0);
  f.setFill(255, 0, 0);
}
void clearBlob() {
  f.setNoFill();
  f.setNoStroke();
}

void beginWalls() {
  if (DEBUG) {
    print("beggining wall");
  }
  for (int i=0; i<4; i++)
  {
    for (int j=0; j<=12; j++)
    {
      //if (i ==0 && j==2) {
      //  j++;
      //}

      spacedWalls[i][j].setFill(0, 50, 0);
      spacedWalls[i][j].setSensor(false);
    }
  }
  circle2.setFill(255, 0, 0);
  circle2.setSensor(false);
}
void clearWalls() {
  if (DEBUG) {
    print("clearing wall");
  }

  for (int i=0; i<4; i++)
  {
    for (int j=0; j<=12; j++)
    {
      spacedWalls[i][j].setNoFill();
      spacedWalls[i][j].setSensor(true);
    }
  }
  circle2.setNoFill();
}

void beginBubble() {
  bubble.setFill(0, 0, 245);
  bubble.setPosition(10, 8);
  bubble.setSensor(false);
  world.add(bubble);
  done = false;
}
void clearBubble() {
  world.remove(bubble);
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
  if (mode != 2) {
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
  }
  torques.set(widgetOne.set_device_torques(fEE.array()));
  widgetOne.device_write_torques();
  setAnimations();

  world.step(1.0f/1000.0f);

  renderingForce = false;
}

void getEndEffectorState() {
  /* GET END-EFFECTOR STATE (TASK SPACE) */
  widgetOne.device_read_data();



  if (mode ==2) {
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

void setAnimations() {
  if (mode == 4) {
    if ((s.h_avatar.isTouchingBody(c) && !file2.isPlaying())) {
      s.h_avatar.setDamping(800);
      file2.play();
    } else {
      s.h_avatar.setDamping(0);
    }
  }
  if (mode ==5) {
    if (s.h_avatar.isTouchingBody(bubble) && !file.isPlaying()) {
      //file.play();

      currentMillis = millis();
      if (currentMillis - previousMillis > interval) {
        playAudio();
        done=true;
        clearBubble();
        if (DEBUG)
          print("inside");
      }
    } else {
      previousMillis = millis();
      //circle2.setFill(0,0,0);
    }
  }
}


/* end helper functions section ****************************************************************************************/
