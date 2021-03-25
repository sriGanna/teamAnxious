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

Minim minim;
AudioPlayer song;
SoundFile file;
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
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                = new PVector(0, 0); 

/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 20.0;  
float             worldHeight                         = 15.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;


/* Initialization of elements */
FCircle           circle1, bbody;
FPoly             b1;
FPoly             b2;
FLine             l1;
FLine             l2;
FLine             l3;
FBlob           blob;

/* Timer variables */
long currentMillis = millis();
long previousMillis = 0;
float interval = 10;

/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar, pac2, bubble;

/* end elements definition *********************************************************************************************/

boolean done=false;
ArrayList <Splat> splats = new ArrayList <Splat> ();
/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */
  file = new SoundFile(this, "pop1.wav");
  //file.play();

  /* screen size definition */
  size(800, 600);

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

  //reset circle
  circle1                   = new FCircle(3);
  circle1.setPosition(15, 5);
  circle1.setStatic(true);
  circle1.setFill(255, 255, 255);
  circle1.setNoStroke();
  world.add(circle1);

  //balloon  
  //l1 = new FLine(10,8.5,9.8,8.8);
  //addLine(l1);

  //l2 = new FLine(9.8,8.8,10.1,9);
  //addLine(l2);

  //l3 = new FLine(10.1,9,9.9,9.3);
  //addLine(l3);

  b1                   = new FPoly(); 
  b1.vertex(-0.7f, -1.0f);
  b1.vertex( 0.7f, -1.0f);
  b1.vertex( 0, 0);
  b1.setPosition(10, 8.5); 
  addPoly(b1);


  b2                   = new FPoly(); 
  b2.vertex(-0.3f, 1.0f);
  b2.vertex( 0.3f, 1.0f);
  b2.vertex( 0, 0);
  b2.setPosition(10, 7.5); 
  addPoly(b2); 

  bbody                   = new FCircle(1.7);
  bbody.setPosition(10, 7);
  bbody.setStatic(true);
  bbody.setFill(97, 59, 175);
  bbody.setNoStroke();
  world.add(bbody);

  //blob                   = new FBlob();
  ////blob.setAsCircle(10,10,2,16);
  //blob.vertex(10, 10);
  //blob.vertex(1, 9);
  //blob.vertex(12, 8);

  //blob.setFill(0, 255, 0);
  //world.add(blob);


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
  world.setGravity((0.0), (6000.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(0.4);
  world.setEdgesFriction(1.2);


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

    for (Splat s : splats) {
      s.display();
    }


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


    if (s.h_avatar.isTouchingBody(bbody) || s.h_avatar.isTouchingBody(b1) || s.h_avatar.isTouchingBody(b2) && !file.isPlaying()) {
      //file.play();

      currentMillis = millis();
      if (currentMillis - previousMillis > interval) {
        playAudio();
        done=true;
        //print("inside");
        //delay(10);
        bbody.setNoFill();
        b1.setNoFill();
        b2.setNoFill();
        bbody.setSensor(true);
        if(done){
        splats.add(new Splat(400, 300));
        done = false;
        }
      }
    } else {
      previousMillis = millis();
      //bbody.setFill(0,0,0);
    }

    if (s.h_avatar.isTouchingBody(circle1)) {
      b1.setPosition(10, 8.5); 
      b1.setFill(82, 50, 148);
      b2.setPosition(10, 7.5); 
      b2.setFill(82, 50, 148);
      bbody.setFill(97, 59, 175);
      bbody.setPosition(10, 7);
      done=false;
      bbody.setSensor(false);
    }
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

  Splat(float x, float y) {
    this.x = x;
    this.y = y;
    rad = 17;
    splat = createGraphics(200, 200, JAVA2D);
    create();
  }

  void create() {
    splat.beginDraw();
    splat.smooth();
    splat.colorMode(HSB, 360, 100, 100);
    splat.fill(random(360), 100, 100);
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
    imageMode(CENTER);
    image(splat, x, y);
  }
}
/* end helper functions section ****************************************************************************************/
