import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.serial.*; 
import static java.util.concurrent.TimeUnit.*; 
import java.util.concurrent.*; 
import processing.sound.*; 
import ddf.minim.*; 
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

public class paintball_explode_multi extends PApplet {

/**********************************************************************************************************************
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







AudioPlayer song;
SoundFile file;

ControlP5 cp5;

/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/


boolean DEBUG = false;
boolean DEBUGPOS = true;

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
float             pixelsPerCentimeter                 = 40.0f;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                = new PVector(0, 0); 

/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 30.0f;  
float             worldHeight                         = 20.0f; 

float             edgeTopLeftX                        = 0.0f; 
float             edgeTopLeftY                        = 0.0f; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

//bubble locations
int[] xCord={5, 7, 10, 13, 16, 18, 20, 22, 24, 26};
int[] yCord={5, 14, 8, 14, 7, 16, 5, 12, 16, 4};
float[] rad={4, 3, 2, 4, 3, 3, 2, 3, 2, 4};
FCircle[] bub=new FCircle[10];
FCircle[] burst=new FCircle[10];
//color codes
int[] r=new int[10];
int[] g=new int[10];
int[] b=new int[10];

ArrayList<FBody> isTouching;
/* Initialization of elements */
FCircle           circle1;

/* Timer variables */
long currentMillis = millis();
long previousMillis = millis();
float interval = 1500;
int time = -1;

/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar, pac2, bubble;
PGraphics output;

/* end elements definition *********************************************************************************************/

int c1, c2, c3;

boolean done=false;
boolean splatshown=false;
boolean reset=false;

Splat abc;
ArrayList <Splat> splats = new ArrayList <Splat> ();
/* setup section *******************************************************************************************************/
public void setup() {
  /* put setup code here, run once: */
  file = new SoundFile(this, "pop1.wav");
  //file.play();

  /* screen size definition */
  

  /* device setup */
  cp5 = new ControlP5(this);

  PFont p = createFont("Verdana", 17); 
  ControlFont font = new ControlFont(p);

  // change the original colors
  cp5.setColorForeground(0xffaa0000);
  cp5.setColorBackground(color(0, 0, 0));
  cp5.setFont(font);
  cp5.setColorActive(0xffff0000);
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

  //randomize();

  //reset button
  cp5.addButton("Reset")
    .setPosition(500, 710)
    .setSize(150, 50)
    ;



  for (int i=0; i<10; i++)
  {
    randomize();
    r[i]=c1;
    g[i]=c2;
    b[i]=c3;
  }

  //creating field of bubbles
  for (int i =0; i<10; i++)
  {
    bub[i] = new FCircle(rad[i]);
    bub[i].setPosition(xCord[i], yCord[i]);
    bub[i].setStatic(true);
    bub[i].setFill(r[i], g[i], b[i]);
    bub[i].setNoStroke();
    world.add(bub[i]);
    burst[i] = new FCircle(rad[i]+1);
    burst[i].setPosition(xCord[i], yCord[i]);
    burst[i].setStatic(true);
    burst[i].setNoFill();
    burst[i].setNoStroke();
  }


  //bubble = loadImage("../img/bubble.png"); 
  //bubble.resize((int)(hAPI_Fisica.worldToScreen(2)), (int)(hAPI_Fisica.worldToScreen(2)));
  //bbody.attachImage(bubble); 

  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4);  
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 


  /* If you are developing on a Mac users must update the path below 
   * from "../img/Haply_avatar.png" to "./img/Haply_avatar.png" 
   */
  haplyAvatar = loadImage("../img/tack.png"); 
  haplyAvatar.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1)));
  s.h_avatar.attachImage(haplyAvatar); 

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


    //for (Splat s : splats) {
    //  if (splatshown==true) {
    //    s.display();
    //  }
    //  //abc.display();
    //}    
    image(output, 0, 0);

    world.draw();
  }
}
/* end draw section ****************************************************************************************************/

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
      posEE.set(posEE.copy().mult(200));
    }

    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 


    s.updateCouplingForce();
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();

    world.step(1.0f/1000.0f);


    checkSplat();

    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/

public void Reset() {
  done=false;
  splatshown=false;
  splats.clear();
  //world.clear();
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
    burst[i] = new FCircle(rad[i]+3);
    burst[i].setPosition(xCord[i], yCord[i]);
    burst[i].setStatic(true);
    burst[i].setNoFill();
    burst[i].setNoStroke();
  } 
  reset=false;
}

/* helper functions section, place helper functions here ***************************************************************/
public void playAudio() {
  if (done==false)
  {
    file.play();
  }
}

public void randomize() {
  c1=PApplet.parseInt(random(255));
  c2=PApplet.parseInt(random(255));
  c3=PApplet.parseInt(random(255));
}

class Splat {
  float x, y;
  int i;
  float radi;
  PGraphics splat;
  boolean done;
  boolean z;

  Splat(float x, float y, int i) {
    this.x = x*40;
    this.y = y*40;
    this.i = i;
    radi = rad[i]*10;
    splat = createGraphics(600, 600, JAVA2D);
    create();
  }

  public void create() {
    splat.beginDraw();
    splat.smooth();
    splat.colorMode(RGB, 255);
    splat.fill(r[i], g[i], b[i]);
    splat.noStroke();
    for (float i=3; i<29; i+=.35f) {
      float angle = random(0, TWO_PI);
      float splatX = (splat.width-50)/2 + 25 + cos(angle)*2*i;
      float splatY = (splat.height-50)/2 + 25 + sin(angle)*3*i;
      splat.ellipse(splatX, splatY, radi-i, radi-i+1.8f);
    }
    splat.endDraw();
  }
  public void display() {
    output.beginDraw();
    output.imageMode(CENTER);
    output.image(splat, x, y);
    output.endDraw();
  }
}

public void checkSplat() {

  isTouching = s.h_avatar.getTouching();
  if (DEBUG) {
    println(isTouching);
  }
  for (int i =0; i<10; i++) {
    if (isTouching.contains(bub[i])) { 
      currentMillis = millis();
      if (currentMillis - previousMillis > interval) {         
        splatshown = false;
        animateSplat(bub[i], burst[i], i); 
        previousMillis = millis();
      }
    }
    if (splatshown && timer_passed(100)&&burst[i] != null) {
      world.remove(burst[i]);
    }
  }
}

public void animateSplat(FCircle bubble, FCircle burstCirc, int i) {
  playAudio();
  if (splatshown == false) {
    splats.add(new Splat(bubble.getX(), bubble.getY(), i));
    if (DEBUGPOS) {
      println(bubble.getX());
      println(bubble.getY());
    }
    splats.get(splats.size()-1).display();
    splatshown = true;
    if(bubble != null){
    world.add(burstCirc);
    println("added burst");
    }
    world.remove(bubble);
    
  }
}

public void timer_reset() {
  time = millis();
}

public boolean timer_passed(int mseconds) {
  return ( millis() - time > mseconds );
}



/* end helper functions section ****************************************************************************************/
  public void settings() {  size(1200, 800); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "paintball_explode_multi" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
