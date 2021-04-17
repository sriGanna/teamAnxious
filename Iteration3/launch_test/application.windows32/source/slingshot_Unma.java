import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.serial.*; 
import static java.util.concurrent.TimeUnit.*; 
import java.util.concurrent.*; 
import processing.sound.*; 
import ddf.minim.*; 
import java.util.*; 
import controlP5.*; 

import co.haply.hphysics.*; 
import org.jbox2d.collision.*; 
import org.jbox2d.collision.shapes.*; 
import org.jbox2d.common.*; 
import org.jbox2d.dynamics.*; 
import org.jbox2d.dynamics.contacts.*; 
import org.jbox2d.dynamics.controllers.*; 
import org.jbox2d.dynamics.joints.*; 
import org.jbox2d.pooling.*; 
import org.jbox2d.pooling.arrays.*; 
import org.jbox2d.pooling.stacks.*; 
import org.jbox2d.util.blob.*; 
import org.jbox2d.util.nonconvex.*; 
import org.jbox2d.util.sph.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class slingshot_Unma extends PApplet {

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

public final int NUM_PALETTES = 10;
public final float PALETTE_SPACER = 1.5f; //space between palette elements
public final float BUTTON_SPACER = 1.25f*1.5f; //space between GUI elements

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
float             pixelsPerMeter                      = 4000.0f;
float             radsPerDegree                       = 0.01745f;

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0f;

/* end effector radius in meters */
float             rEE                                 = 0.006f;

/* virtual wall parameter  */
float             kWall                               = 2000;
PVector           fWall                               = new PVector(0, 0);
PVector           penWall                             = new PVector(0, 0);
PVector           posWall                             = new PVector(0.01f, 0.1f);

/* pantagraph link parameters in meters */
float             l                                   = 0.07f;
float             L                                   = 0.09f;


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
float             worldWidth                          = 27.5f;  
float             worldHeight                         = 17.5f; 

float             edgeTopLeftX                        = 0.0f; 
float             edgeTopLeftY                        = 0.0f; 
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
int colR, colG, colB;
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


FBox[] colorSwatch = new FBox[6];
ArrayList<ColorPalette> palettes;
int paletteIndex;

/* setup section *******************************************************************************************************/
public void setup() {
  /* put setup code here, run once: */
  file = new SoundFile(this, "splat.mp3");
  //file.play();

  /* screen size definition */
  

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
  haplyBoard          = new Board(this, Serial.list()[0], 0);
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
  createPalettes();
  paletteIndex = 0;
  float x = createColorPicker(palettes.get(paletteIndex)) - BUTTON_SPACER;
  float y = edgeBottomRightY - 1.5f;
  //createBubbles();

  wall = create_wall(posWall.x-0.2f, posWall.y+rEE+.01f, posWall.x+0.2f, posWall.y+rEE+.01f);

  /* world conditions setup */
  world.setGravity((0.0f), (6000.0f)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(0.4f);
  world.setEdgesFriction(1.2f);

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
public void draw() {
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if (renderingForce == false) {
    background(255);
    image(output, 0, 0);
    //checkChangeColor();
    world.draw();
    checkChangeColor();
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
public void playAudio() {
  if (done==false)
  {
    file.play();
    //print("here");
  }
}

public void addLine(FLine l) {
  l.setStatic(true);
  l.setFill(0, 255, 0);
  l.setStroke(0, 0, 0);
  l.setStrokeWeight(3);
  world.add(l);
}

public void addPoly(FPoly p) {
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

  public void create() {
    splat.beginDraw();
    splat.smooth();
    splat.colorMode(HSB, 360, 100, 100);
    splat.fill(s.h_avatar.getFillColor());
    splat.noStroke();
    for (float i=3; i<29; i+=.35f) {
      float angle = random(0, TWO_PI);
      float splatX = (splat.width-50)/2 + 25 + cos(angle)*2*i;
      float splatY = (splat.height-50)/2 + 25 + sin(angle)*3*i;
      splat.ellipse(splatX, splatY, rad-i, rad-i+1.8f);
    }
    splat.endDraw();
  }
  public void display() {
    output.beginDraw();
    output.imageMode(CENTER);
    output.image(splat, x, y);
    output.endDraw();
  }
  public void saveSplat() {
    splat.save("./saved/test.png");
  }
}

public PVector device_to_graphics(PVector deviceFrame) {
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


public PVector graphics_to_device(PVector graphicsFrame) {
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}

public PShape create_wall(float x1, float y1, float x2, float y2) {
  x1 = pixelsPerMeter * x1;
  y1 = pixelsPerMeter * y1;
  x2 = pixelsPerMeter * x2;
  y2 = pixelsPerMeter * y2;

  return createShape(LINE, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
}

public void createSling() {

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


  joint1 = new FDistanceJoint(anchor1, s.h_avatar);
  world.add(joint1);

  joint2 = new FDistanceJoint(anchor2, s.h_avatar);
  world.add(joint2);
}

public void removeSling() {
  world.remove(joint1);
  world.remove(joint2);
  world.remove(anchor1);
  world.remove(anchor2);
  for(int i=0;i<palettes.size();i++)
  {
    for(int j=0;j<6;i++)
    {
      world.remove(colorSwatch[j]);
    }
  }
  delay(500);
  save("./saved/test.png");
  delay(100);
}

public void createPalette() {
  cp5 = new ControlP5(this);

  PFont p = createFont("Verdana", 17); 
  ControlFont font = new ControlFont(p);

  // change the original colors
  cp5.setColorForeground(color(0, 0, 0));
  cp5.setColorBackground(color(0, 0, 0));
  cp5.setFont(font);

  cp5.addButton("save")
    .setLabel("save")
    .setPosition(960, 610)
    .setSize(100, 50)
    .setColorBackground(color(65, 60, 88))

    ;
  cp5.addButton("prev")
    .setLabel("prev")
    .setPosition(960, 120)
    .setSize(100, 30)
    .setColorBackground(color(47,0,79))

    ;
  cp5.addButton("next")
    .setLabel("next")
    .setPosition(960, 160)
    .setSize(100, 30)
    .setColorBackground(color(47,0,79))

    ;
}

public void controlEvent(CallbackEvent event) {
  if (event.getAction() == ControlP5.ACTION_CLICK) {
    switch(event.getController().getAddress()) {
    case "/prev":
      paletteIndex = (paletteIndex - 1 ) % (NUM_PALETTES);
      if (paletteIndex < 0) {
        paletteIndex = NUM_PALETTES - 1;
      }
      updateColorPicker(palettes.get(paletteIndex));
      break;
    case "/next":
      paletteIndex = (paletteIndex + 1) % (NUM_PALETTES);
      updateColorPicker(palettes.get(paletteIndex));
      break;
    case "/save":
      output.save("./saved/test.png");
      break;
    }
  }
}


public void checkSplat() {

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

public void animateSplat(FCircle bubble) {
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

public void keyPressed() {
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


public boolean pulledBack() {
  currentPosY = s.h_avatar.getY()/150;
  if (currentPosY > .1f) {
    return true;
  } else {
    return false;
  }
}
float x_vel, y_vel;
public boolean isReleased() {

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

public void drawSplat(double speed)
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

public boolean isMoving() {

  if (abs(s.h_avatar.getVelocityX())<.05f && abs(s.h_avatar.getVelocityY())<.05f) {
    return false;
  } else {
    return true;
  }
}
  
//palettes
public void createPalettes() {
  palettes = new ArrayList<ColorPalette>();
  for (int i=0; i< NUM_PALETTES; i++) {
    palettes.add(createPalette(i)); //add all defined palettes
  }
}

public ColorPalette createPalette(int index) {
  ColorSwatch[] palette = new ColorSwatch[6];
  switch(index) {
    case(9): //pastel og
    palette[5] = new ColorSwatch(255, 166, 158, 5); //pink
    palette[4] = new ColorSwatch(250, 243, 221, 4); //yellow
    palette[3] = new ColorSwatch(184, 242, 230, 3); //green
    palette[2] = new ColorSwatch(205, 168, 230, 2); //purple
    palette[1] = new ColorSwatch(153, 196, 224, 1); //blue
    palette[0] = new ColorSwatch(94, 100, 114, 0); //grey
    break;
    case(8): //pastel but make it fun
    palette[5] = new ColorSwatch(155, 140, 237, 5); //purple
    palette[4] = new ColorSwatch(235, 226, 134, 4); //yellow
    palette[3] = new ColorSwatch(232, 104, 147, 3); //pink
    palette[2] = new ColorSwatch(255, 149, 138, 2); //orange
    palette[1] = new ColorSwatch(126, 222, 204, 1); //green
    palette[0] = new ColorSwatch(103, 182, 219, 0); //blue
    break;
    case(7): //depressed cherry blossoms
    palette[5] = new ColorSwatch(234, 191, 203, 5); //pink
    palette[4] = new ColorSwatch(193, 145, 161, 4); //brown pink
    palette[3] = new ColorSwatch(95, 010, 135, 3); //purple
    palette[2] = new ColorSwatch(47, 0, 79, 2); //angry purple
    palette[1] = new ColorSwatch(164, 80, 139, 1); //deep pink
    palette[0] = new ColorSwatch(26, 020, 035, 0); //dead purple
    break;
    case(6): //a field of tulips that you'll never see because you never get out of your house
    palette[5] = new ColorSwatch(211, 063, 073, 5); //red
    palette[4] = new ColorSwatch(221, 255, 247, 4); //blue
    palette[3] = new ColorSwatch(147, 184, 073, 3); //green
    palette[2] = new ColorSwatch(234, 214, 055, 2); //yellow
    palette[1] = new ColorSwatch(65, 60, 80, 1); //blue
    palette[0] = new ColorSwatch(38, 39, 48, 0); //black
    break;
    case(5): //party balloons at the party you never go to
    palette[5] = new ColorSwatch(0, 071, 119, 5); //blue
    palette[4] = new ColorSwatch(163, 000, 000, 4); //red
    palette[3] = new ColorSwatch(255, 119, 000, 3); //orange
    palette[2] = new ColorSwatch(239, 210, 141, 2); //beige
    palette[1] = new ColorSwatch(0, 175, 181, 1); //blue
    palette[0] = new ColorSwatch(255, 249, 79, 0); //yellow
    break;
    case(4): //daisies but in high saturation
    palette[5] = new ColorSwatch(251, 97, 7, 5); //orange
    palette[4] = new ColorSwatch(243, 222, 44, 4); //yellow
    palette[3] = new ColorSwatch(124, 181, 24, 3); //green slime
    palette[2] = new ColorSwatch(31, 39, 07, 2); //black like my heart
    palette[1] = new ColorSwatch(92, 128, 1, 1); //moss green
    palette[0] = new ColorSwatch(251, 176, 45, 0); //if orange and yellow had a kid
    break;
    case(3): //tie dye shirt gone wrong
    palette[5] = new ColorSwatch(255, 102, 102, 5); //pink
    palette[4] = new ColorSwatch(204, 255, 102, 4); //green or yellow
    palette[3] = new ColorSwatch(93, 046, 140, 3); //purple
    palette[2] = new ColorSwatch(46, 196, 182, 2); //teal not cyan
    palette[1] = new ColorSwatch(241, 232, 184, 1); //beige again
    palette[0] = new ColorSwatch(004, 004, 003, 0); //the void
    break;
    case(2): //daisies but they're all dead
    palette[5] = new ColorSwatch(241, 247, 237, 5); //eggshell
    palette[4] = new ColorSwatch(36, 062, 054, 4); //moss
    palette[3] = new ColorSwatch(124, 169, 130, 3); //dead green
    palette[2] = new ColorSwatch(224, 238, 198, 2); //light green
    palette[1] = new ColorSwatch(194, 168, 062, 1); //yellow
    palette[0] = new ColorSwatch(56, 29, 42, 0); //brown
    break;
    case(1): //that one namib desert photo
    palette[5] = new ColorSwatch(255, 210, 117, 5); //sand
    palette[4] = new ColorSwatch(232, 174, 104, 4); //dark sand
    palette[3] = new ColorSwatch(165, 127, 96, 3); //even darker sand
    palette[2] = new ColorSwatch(227, 165, 135, 2); //flesh
    palette[1] = new ColorSwatch(219, 90, 60, 1); //sand but tanned
    palette[0] = new ColorSwatch(66, 066, 066, 0); //demonic black
    break;
    case(0): //every monday blues in 2020
    palette[5] = new ColorSwatch(3, 026, 107, 5); //blue
    palette[4] = new ColorSwatch(2, 19, 79, 4); //also blue
    palette[3] = new ColorSwatch(105, 108, 194, 3); //still blue
    palette[2] = new ColorSwatch(182, 235, 252, 2); //more blue
    palette[1] = new ColorSwatch(5, 178, 220, 1); //it's all blue
    palette[0] = new ColorSwatch(82, 126, 183, 0); //red. lol nope
    break;
  default:  //pastel rainbow
    palette[5] = new ColorSwatch(155, 140, 237, 5); //purple
    palette[4] = new ColorSwatch(235, 226, 134, 4); //yellow
    palette[3] = new ColorSwatch(232, 104, 147, 3); //pink
    palette[2] = new ColorSwatch(255, 149, 138, 2); //orange
    palette[1] = new ColorSwatch(126, 222, 204, 1); //green
    palette[0] = new ColorSwatch(103, 182, 219, 0); //blue
    break;
  }

  return new ColorPalette(palette);
}


//check color
public void checkChangeColor() {
  ColorPalette palette = palettes.get(paletteIndex);
  for (int i=0; i<palette.getLength(); i++) {
    if (colorSwatch[i].isTouchingBody(s.h_avatar)) {
        setDrawingColor(palette.getSwatch(i).getColor());
    }
  }
}


public void setDrawingColor(int r, int g, int b) {
  colR = r;
  colG = g;
  colB = b;
  s.h_avatar.setFill(colR, colG, colB);
}

public void setDrawingColor(int[] rgb) {
  setDrawingColor(rgb[0], rgb[1], rgb[2]);
}

public void updateColorPicker(ColorPalette palette) {
  ColorSwatch swatch;
  for (int i=0; i<palette.getLength(); i++) {
    swatch = palette.getSwatch(i);
    colorSwatch[i].setFillColor(color(swatch.getRed(), swatch.getGreen(), swatch.getBlue()));
    world.draw();
  }
}

public float createColorPicker(ColorPalette palette) {
  float x = 25.2f;
  float y = 4;
  ColorSwatch swatch;
  for (Integer i=0; i< 6; i++) {
    y = y + PALETTE_SPACER;
    colorSwatch[i] = new FBox(2, 1);
    colorSwatch[i].setPosition(x, y);
    colorSwatch[i].setStatic(true);
    colorSwatch[i].setSensor(true);
    colorSwatch[i].setName(i.toString());

    swatch = palette.getSwatch(i);
    //print(swatch);
    colorSwatch[i].setFillColor(color(swatch.getRed(), swatch.getGreen(), swatch.getBlue()));
    //print("here "+swatch.getRed());
    world.add(colorSwatch[i]);

    //world.draw();
  }
  
  return x;
}  

/* end helper functions section ****************************************************************************************/
  public void settings() {  size(1100, 700); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "slingshot_Unma" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
