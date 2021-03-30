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
import ddf.minim.*;
import java.util.*;
import controlP5.*;

Minim minim;
AudioPlayer song;
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


/* Initialization of elements */
FCircle           circle1, bbody;
FPoly             b1;
FPoly             b2;
FLine             l1;
FLine             l2;
FLine             l3;
FBlob           blob;
FBox            anchor1, anchor2;
FDistanceJoint    joint1, joint2;
FBox          c1, c2, c3, c4, c5, c6, c7;
FCircle select, balloon;

PShape wall;
FCircle[] bubbles = new FCircle[28];
float colour_inc=0;
float colR, colG, colB;
float currentPosY;
int bubbleQuant = 4;
double speed;
ArrayList<FBody> isTouching;

/* Initialization of virtual tool */
PImage            colour;
PGraphics output;

/* end elements definition *********************************************************************************************/

boolean done=false;
ArrayList <Splat> splats = new ArrayList <Splat> ();
boolean splatshown=false;
boolean selectCol = true;
boolean redraw = false;
boolean wasPulled = false;
boolean released = false;
boolean loadBalloon = false;
/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */
  file = new SoundFile(this, "splat.mp3");
  //file.play();

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

  createSling();
  createPalette();
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

    for (Splat abs : splats) {
      abs.display();
    }
    imageMode(CENTER);
    image(output, x, y);
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/


//void contactResult(FContactResult result) {
//  // Draw an ellipse where the contact took place and as big as the normal impulse of the contact
//  ellipse(result.getX(), result.getY(), result.getNormalImpulse(), result.getNormalImpulse());

//  // Trigger your sound here
//  // ...
//  playAudio();
//  done=true;
//}


/* Timer variables */
long currentMillis = millis();
long previousMillis = 0;
float interval = 50;
/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */

    renderingForce = true;
    //file.play();

    if (haplyBoard.data_available()) {
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));

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
    }
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7+6); 


    s.updateCouplingForce();
    //fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    //fEE.div(100000); //dynes to newtons

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    //keyPressed();
    //if (selectCol) {
    //  selectColour();
    //} else {
    //  hideSelect();
    //}

    //checkSplat();
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

    world.step(1.0f/1000.0f);
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/
void playAudio() {
  if (done==false)
  {
    file.play();
    //print("here");
  }
}

void addLine(FLine l) {
  l.setStatic(true);
  l.setFill(0, 255, 0);
  l.setStroke(0, 0, 0);
  l.setStrokeWeight(3);
  world.add(l);
}

void addPoly(FPoly p) {
  p.setStatic(true);
  p.setFill(82, 50, 148);
  p.setNoStroke();
  world.add(p);
}

class Splat {
  float x, y;
  float rad;
  PGraphics splat;

  Splat(float x, float y, float rad) {
    this.x = x;
    this.y = y;
    this.rad = rad;
    splat = createGraphics(200, 200, JAVA2D);
    create();
  }

  void create() {
    splat.beginDraw();
    splat.smooth();
    splat.colorMode(HSB, 360, 100, 100);
    splat.fill(s.h_avatar.getFillColor());
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
  void saveSplat(){
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

void createPalette() {
  cp5 = new ControlP5(this);

  PFont p = createFont("Verdana", 17); 
  ControlFont font = new ControlFont(p);

  // change the original colors
  cp5.setColorForeground(color(0, 0, 0));
  cp5.setColorBackground(color(0, 0, 0));
  cp5.setFont(font);

  cp5.addButton("red")
    .setLabel("red")
    .setPosition(980, 100)
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

void controlEvent(CallbackEvent event) {
  if (event.getAction() == ControlP5.ACTION_CLICK) {
    switch(event.getController().getAddress()) {
    case "/red":
      colR = 255;
      colG = 0;
      colB = 0;
      s.h_avatar.setFill(colR, colG, colB);
      break;
    case "/orange":
      colR = 255;
      colG = 128;
      colB = 0;
      s.h_avatar.setFill(colR, colG, colB);
      break;
    case "/yellow":
      colR = 255;
      colG = 255;
      colB = 0;
      s.h_avatar.setFill(colR, colG, colB);
      break;
    case "/green":
      colR = 0;
      colG = 255;
      colB = 0;
      s.h_avatar.setFill(colR, colG, colB);
      break;
    case "/lBlue":
      colR = 0;
      colG = 128;
      colB = 255;
      s.h_avatar.setFill(colR, colG, colB);
      break;
    case "/blue":
      colR = 0;
      colG = 0;
      colB = 255;
      s.h_avatar.setFill(colR, colG, colB);
      break;
    case "/purple":
      colR = 255;
      colG = 0;
      colB = 255;
      s.h_avatar.setFill(colR, colG, colB);
      break;
    case "/save":
      //removeSling();
      //createSling();
      output.save("./saved/test.png");
      break;
    }
  }
}
void selectColour_old() {
  if (s.h_avatar.isTouchingBody(c1)) { ///red
    colour_inc++;
    if (colour_inc >3600) {
      colour_inc=0;
    }
    c1.setFill(colour_inc/20+100, 0, 0);
    s.h_avatar.setFill(colour_inc/20+100, 0, 0);
    colR = colour_inc/20+100;
    colG = 0;
    colB = 0;
  } else if (s.h_avatar.isTouchingBody(c2)) { //orange 
    colour_inc++;
    if (colour_inc >3600) {
      colour_inc=0;
    }
    c2.setFill(255, colour_inc/20+75, 0);
    s.h_avatar.setFill(255, colour_inc/20+75, 0);
    colR = 255;
    colG = colour_inc/20+75;
    colB = 0;
  } else if (s.h_avatar.isTouchingBody(c3)) { //yellow
    colour_inc++;
    if (colour_inc >3600) {
      colour_inc=0;
    }
    c3.setFill(colour_inc/20+125, colour_inc/20+125, 0);
    colR = colour_inc/20+125;
    colG = colour_inc/20+125;
    colB = 0;
    s.h_avatar.setFill(colour_inc/20+125, colour_inc/20+125, 0);
  } else if (s.h_avatar.isTouchingBody(c4)) { //green
    colour_inc++;
    if (colour_inc >3600) {
      colour_inc=0;
    }
    c4.setFill(0, colour_inc/20+75, 0);
    colR = 0;
    colG = colour_inc/20+75;
    colB = 0;
    s.h_avatar.setFill(0, colour_inc/20+75, 0);
  } else if (s.h_avatar.isTouchingBody(c5)) { //light blue
    colour_inc++;
    if (colour_inc >3600) {
      colour_inc=0;
    }
    c5.setFill(0, colour_inc/20+125, colour_inc/20+125);
    colR = 0;
    colG = colour_inc/20+125;
    colB = colour_inc/20+125;
    s.h_avatar.setFill(0, colour_inc/20+125, colour_inc/20+125);
  } else if (s.h_avatar.isTouchingBody(c6)) { //blue 
    colour_inc++;
    if (colour_inc >3600) {
      colour_inc=0;
    }
    c6.setFill(0, 0, colour_inc/20+100);
    colR = 0;
    colG = 0;
    colB = colour_inc/20+100;
    s.h_avatar.setFill(0, 0, colour_inc/20+100);
  } else if (s.h_avatar.isTouchingBody(c7)) { //purple 
    colour_inc++;
    if (colour_inc >3600) {
      colour_inc=0;
    }
    c7.setFill(255, 0, colour_inc/20+100);
    colR = 255;
    colG = 0;
    colB = colour_inc/20+100;
    s.h_avatar.setFill(255, 0, colour_inc/20+100);
  } else {
    colour_inc = 0;
  }
}

void checkSplat() {

  isTouching = s.h_avatar.getTouching();
  if (DEBUG) {
    println(isTouching);
  }
  for (int i =0; i<bubbleQuant; i++) {
    if (isTouching.contains(bubbles[i])) {
      splatshown = false;
      animateSplat(bubbles[i]);
    }
  }
}

void animateSplat(FCircle bubble) {
  playAudio();
  if (splatshown == false) {
    splats.add(new Splat(bubble.getX()*40, bubble.getY()*40, 17));
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

/* end helper functions section ****************************************************************************************/
