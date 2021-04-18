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
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
import processing.sound.*;
import ddf.minim.*;
import controlP5.*;

AudioPlayer song;
SoundFile file;

ControlP5 cp5;

/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/


boolean DEBUG = false;
boolean DEBUGPOS = true;

public final int NUM_PALETTES = 10;
public final float PALETTE_SPACER = 1.5; //space between palette elements
public final float BUTTON_SPACER = 1.25*1.5; //space between GUI elements

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
float             worldWidth                          = 30;  
float             worldHeight                         = 20; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

//bubble locations
int[] xCord={5, 7, 10, 13, 15, 18, 19, 22, 24, 23};
int[] yCord={5, 14, 8, 14, 7, 16, 5, 12, 16, 4};
float[] rad={4, 3, 2, 4, 3, 3, 2, 3, 2, 4};
FCircle[] bub=new FCircle[10];
FCircle[] burst=new FCircle[10];
//color codes
int[] r=new int[10];
int[] g=new int[10];
int[] b=new int[10];

int colR, colG, colB;
FBox menu;

int change=0;

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


FBox[] colorSwatch = new FBox[6];
ArrayList<ColorPalette> palettes;
ColorPalette selected=null;
int shade=0;
int paletteIndex;


Splat abc;
ArrayList <Splat> splats = new ArrayList <Splat> ();
/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */
  file = new SoundFile(this, "pop1.wav");
  //file.play();

  /* screen size definition */
  size(1200, 800);

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

  createPalette();
  createPalettes();
  createMenu();
  paletteIndex = 0;
  float x = createColorPicker(palettes.get(paletteIndex)) - BUTTON_SPACER;
  float y = edgeBottomRightY - 1.5;

  //reset button
  //cp5.addButton("Reset")
  //  .setPosition(500, 710)
  //  .setSize(150, 50)
  //  ;



  //for (int i=0; i<10; i++)
  //{
  //  randomize();
  //  r[i]=c1;
  //  g[i]=c2;
  //  b[i]=c3;
  //}

  //initial=palettes.get(0);   
  selected=palettes.get(0);
  
  for (int i=0; i<10; i++) {
    shade=int(random(6));
    setDrawingColor(selected.getSwatch(shade).getColor());
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
  world.setGravity((0.0), (6000.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(0.4);
  world.setEdgesFriction(1.2);

  output = createGraphics(800, 800, JAVA2D);
  
  background(255);
  
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


    //for (Splat s : splats) {
    //  if (splatshown==true) {
    //    s.display();
    //  }
    //  //abc.display();
    //}    
    image(output, 0, 0);

    world.draw();
    checkChangeColor();
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
  //print("reset");
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
void playAudio() {
  if (done==false)
  {
    file.play();
  }
}

void randomize() {
  //print("in randomize");
  selected=palettes.get(paletteIndex);
  shade=int(random(6));
  setDrawingColor(selected.getSwatch(shade).getColor());
  //c1=int(random(255));
  //c2=int(random(255));
  //c3=int(random(255));
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

  void create() {
    splat.beginDraw();
    splat.smooth();
    splat.colorMode(RGB, 255);
    splat.fill(r[i], g[i], b[i]);
    splat.noStroke();
    for (float i=3; i<29; i+=.35) {
      float angle = random(0, TWO_PI);
      float splatX = (splat.width-50)/2 + 25 + cos(angle)*2*i;
      float splatY = (splat.height-50)/2 + 25 + sin(angle)*3*i;
      splat.ellipse(splatX, splatY, radi-i, radi-i+1.8);
    }
    splat.endDraw();
  }
  void display() {
    output.beginDraw();
    output.imageMode(CENTER);
    output.image(splat, x, y);
    output.endDraw();
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
        animateSplat(bub[i], burst[i], i); 
        previousMillis = millis();
      }
    }
    if (splatshown && timer_passed(100)&&burst[i] != null) {
      world.remove(burst[i]);
    }
  }
}

void animateSplat(FCircle bubble, FCircle burstCirc, int i) {
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

void timer_reset() {
  time = millis();
}

boolean timer_passed(int mseconds) {
  return ( millis() - time > mseconds );
}


//palettes
void createPalettes() {
  palettes = new ArrayList<ColorPalette>();
  for (int i=0; i< NUM_PALETTES; i++) {
    palettes.add(createPalette(i)); //add all defined palettes
  }
}

ColorPalette createPalette(int index) {
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
void checkChangeColor() {
  //ColorPalette palette = palettes.get(paletteIndex);
  //for (int i=0; i<palette.getLength(); i++) {
  //  //if (colorSwatch[i].isTouchingBody(s.h_avatar)) {
  //  //    setDrawingColor(palette.getSwatch(i).getColor());
  //  //}
  //}
  if(change==1) {
    print("here");
    Reset();
    change=0;
  }
}


void setDrawingColor(int r, int g, int b) {
  c1 = r;
  c2 = g;
  c3 = b;
  //s.h_avatar.setFill(colR, colG, colB);
}

void setDrawingColor(int[] rgb) {
  setDrawingColor(rgb[0], rgb[1], rgb[2]);
}

void updateColorPicker(ColorPalette palette) {
  ColorSwatch swatch;
  for (int i=0; i<palette.getLength(); i++) {
    swatch = palette.getSwatch(i);
    colorSwatch[i].setFillColor(color(swatch.getRed(), swatch.getGreen(), swatch.getBlue()));
    world.draw();
  }
}

float createColorPicker(ColorPalette palette) {
  float x = 25.2+3;
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

void createMenu(){
  
  menu              = new FBox(4, 20);
  menu.setFill(100,100,100);
  menu.setPosition(28,10);
  menu.setStatic(true);
  world.add(menu);
  
}

void controlEvent(CallbackEvent event) {
  if (event.getAction() == ControlP5.ACTION_CLICK) {
    switch(event.getController().getAddress()) {
    case "/prev":
      paletteIndex = (paletteIndex - 1 ) % (NUM_PALETTES);
      if (paletteIndex < 0) {
        paletteIndex = NUM_PALETTES - 1;
      }
      updateColorPicker(palettes.get(paletteIndex));
      change=1;
      print("prev");
      break;
    case "/next":
      paletteIndex = (paletteIndex + 1) % (NUM_PALETTES);
      updateColorPicker(palettes.get(paletteIndex));
      change=1;
      print("next");
      break;
    case "/save":
      output.save("./saved/test.png");
      break;
    case "/Return":
      printPath("launch_test.pde");
      launch(sketchPath("")+"myfile.bat");
      delay(500);
      exit();
      break;
    }
  }
}


void createPalette() {
  cp5 = new ControlP5(this);

  PFont p = createFont("Verdana", 17); 
  ControlFont font = new ControlFont(p);

  // change the original colors
  cp5.setColorForeground(color(0, 0, 0));
  cp5.setColorBackground(color(0, 0, 0));
  cp5.setFont(font);

  cp5.addButton("save")
    .setLabel("save")
    .setPosition(1075, 610)
    .setSize(100, 50)
    .setColorBackground(color(65, 60, 88))

    ;
   cp5.addButton("Return")
    .setLabel("Return")
    .setPosition(1075, 670)
    .setSize(100, 50)
    .setColorBackground(color(65, 60, 88))

    ;
  cp5.addButton("prev")
    .setLabel("prev")
    .setPosition(1075, 120)
    .setSize(100, 30)
    .setColorBackground(color(47,0,79))

    ;
  cp5.addButton("next")
    .setLabel("next")
    .setPosition(1075, 160)
    .setSize(100, 30)
    .setColorBackground(color(47,0,79))

    ;
}


void printPath(String app) {
  PrintWriter output=null;
  output = createWriter("myfile.bat");
  output.print("cd ");
  // output.println(myPath);
  String myPath = sketchPath("");
  String newPath = myPath.substring(0, myPath.lastIndexOf('\\'));
  newPath = newPath.substring(0, newPath.lastIndexOf('\\'));
  newPath = newPath.substring(0, newPath.lastIndexOf('\\')); // uncomment when exporting!!
  output.print(newPath);
  output.println("\\launch_test\\application.windows64\\");
  output.println("launch_test.exe");
  //output.println(app);
  output.flush();
  output.close();
  output=null;
}


/* end helper functions section ****************************************************************************************/
