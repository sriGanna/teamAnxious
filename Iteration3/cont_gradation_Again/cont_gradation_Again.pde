//OLD

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
import controlP5.*;
/* end library imports *************************************************************************************************/

ControlP5 cp5;

/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/

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

/* text font */
PFont             f;

/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

float             kWall                               = 200;
PVector           fWall                               = new PVector(0, 0);
PVector           penWall                             = new PVector(0, 0);
PVector           posWall                             = new PVector(0.01, 0.10);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           posEELast                           = new PVector(0, 0);
PVector           fEE                                = new PVector(0, 0); 

float             rEE                                 = 0.006;
/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 35.0;  
float             worldHeight                         = 25.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;


/* Initialization of wall */
FBox              wall, wallPortal12, wallPortal11, wallHidePortal1, wallPortal5, wall4, wallPortal4, wallPortal3, wallHidePortal5, wallPortal2, wall9, wallPortal120; 
FCircle           circle1, circle2;
FBlob             blob1;

int colR, colG, colB;
FBox menu;
FBox[][] walls=new FBox[35][25];

int defaultcp=0;

/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar, pac2;
//PGraphics outputSplat;
//int graphW = 1200;
//int graphH  = 800;
/* end elements definition *********************************************************************************************/

float threshold = 20;


/* Timer variables */
long currentMillis = millis();
long previousMillis = 0;
float interval = 5000;

//float x=random(35);
//float y=random(22.5);


FBox[] colorSwatch = new FBox[6];
ArrayList<ColorPalette> palettes;
ColorPalette selected=null;
int shade=0;
int paletteIndex;
boolean removed = false;

PGraphics splat;

/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */

  /* screen size definition */
  size(1200, 800);

  /* device setup */
  f                   = createFont("Arial", 30, true);

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

  createPalette();
  createPalettes();
  createMenu();
  paletteIndex = 0;
  float x = createColorPicker(palettes.get(paletteIndex)) - BUTTON_SPACER;
  float y = edgeBottomRightY - 1.5;

  //moved, one way 
  //wallPortal3                  = new FBox(2,3);
  //wallPortal3.setPosition(18, 5);
  //wallPortal3.setFill(255,0,0);
  ////wallPortal4.setDensity(800);
  //wallPortal3.setNoStroke();
  //wallPortal3.setSensor(true);
  //wallPortal3.setStatic(true);
  //world.add(wallPortal3); 


  for (int i=0; i<35; i++) {
    for (int j=0; j<25; j++) {
      walls[i][j] = new FBox(1, 1);
      walls[i][j].setPosition(i, j);
      //if (i>0 && i<=15 && j>0 && j<=13) {
      //  walls[i][j].setFill(0,0,0);
      //}
      //if (i>15 && i<35 && j>0 && j<=13) {
      //  walls[i][j].setFill(255,0,0);
      //}
      //if (i>15 && i<35 && j>13 && j<25) {
      //  walls[i][j].setFill(0,255,0);
      //}
      //if (i>0 && i<=15 && j>13 && j<25) {
      //  walls[i][j].setFill(0,0,255);
      //}
      //walls[i][j].setFill(random(255), random(255), random(255));
      walls[i][j].setNoStroke();
      walls[i][j].setNoFill();
      walls[i][j].setSensor(true);
      walls[i][j].setStatic(true);
      world.add(walls[i][j]);
    }
  }




  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setNoStroke();
  //s.h_avatar.setFill(255, 255, 255);
  s.h_avatar.setSensor(false);
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

  /* world conditions setup */
  world.setGravity((0.0), (0.0)); //1000 cm/(s^2)
  //world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);

  //outputSplat = createGraphics(graphW, graphH, JAVA2D);

  background(255);
  //outputSplat.beginDraw();
  //outputSplat.endDraw();

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
    //background(255);
    //world.setFill(color(0,0,0));
    //image(outputSplat, 0, 0);
    
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


    ////angry
    //if (s.h_avatar.isTouchingBody(wallPortal3)){   
    //  s.h_avatar.adjustPosition(18,9);  
    //}
    //else {
    ////s.h_avatar.setVelocity(0,70);
    //s.h_avatar.setDamping(700);

    //for(int i=0;i<20;i++) {
    //   s.h_avatar.setVelocity(x,y);
    //   x=random(35);
    //   y=random(22.5);
    //   print(x,y);
    //}
    //}

    //retry1
    //ArrayList a = s.h_avatar.getTouching();

    //s.h_avatar.setDamping(700);
    //if (s.h_avatar.isTouchingBody(wallPortal3)) {   
    //s.h_avatar.setVelocity(50, 0); 
    //delay(1000);
    //s.h_avatar.setVelocity(0,50); 
    //delay(1000);
    //s.h_avatar.setVelocity(50,0); 
    //delay(1000);
    //s.h_avatar.adjustPosition(18,21); 
    //s.h_avatar.adjustPosition(18,25); 
    //s.h_avatar.adjustPosition(18,29); 
    //s.h_avatar.adjustPosition(18,34); 
    //s.h_avatar.adjustPosition(18,40);
    //}

    //try2working
    //print(s.h_avatar.getTouching());
    for (int i=0; i<35; i++) {
      for (int j=0; j<25; j++) {
        if (s.h_avatar.isTouchingBody(walls[i][j])) {
          s.h_avatar.setDamping(600);
          if (i>0 && i<=20 && j>0 && j<=9) {
            s.h_avatar.setVelocity(50, 0);
            //fillcolour
            selected=palettes.get(paletteIndex);
            shade=int(random(6));
            setDrawingColor(selected.getSwatch(shade).getColor());
            //s.h_avatar.setFill(random(255),random(255),random(255));
          } else if (i>15 && i<35 && j>0 && j<=20) {
            s.h_avatar.setVelocity(0, 50);
            selected=palettes.get(paletteIndex);
            shade=int(random(6));
            setDrawingColor(selected.getSwatch(shade).getColor());
            //s.h_avatar.setFill(random(255),random(255),random(255));
          } else if (i>11 && i<35 && j>13 && j<25) {
            s.h_avatar.setVelocity(-50, 0);
            selected=palettes.get(paletteIndex);
            shade=int(random(6));
            setDrawingColor(selected.getSwatch(shade).getColor());
            //s.h_avatar.setFill(random(255),random(255),random(255));
          } else if (i>0 && i<=15 && j>8 && j<25) {
            s.h_avatar.setVelocity(0, -50);
            selected=palettes.get(paletteIndex);
            shade=int(random(6));
            setDrawingColor(selected.getSwatch(shade).getColor());
            //s.h_avatar.setFill(random(255),random(255),random(255));
          }
          //if(i>0 && i<35 && j>0 && j<25)
          //{
          //  //int randomX=int(random(-9,10));
          //  //int randomY=int(random(-9,10));
          //  //int c=0;
          //  int c=int(random(0,4));
          //  if(c==0) {
          //    s.h_avatar.setVelocity(50,0);
          //  }
          //  else if(c==1) {
          //    s.h_avatar.setVelocity(0,50);
          //  }
          //  else if(c==2) {
          //    s.h_avatar.setVelocity(-50,0);
          //  }
          //  else if(c==3) {
          //    s.h_avatar.setVelocity(0,-50);
          //  }
          //  //print(randomX, randomY);

          //}
          //else {
          //  randomX=0;
          //  randomY=0;
          //}
        }
      }
    }


    if (removed) {
      save("./saved/art-"+year() + nf(month(), 2) + nf(day(), 2) + "-" + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2) + ".png");   
      addpalette();

      removed = false;
    }
    //if(!removed && noPalette()){
    //  addpalette();
    //}
    ////try3
    //currentMillis = millis();
    //if (currentMillis - previousMillis > interval) {
    //      s.h_avatar.setVelocity(50,0);
    //      previousMillis = currentMillis; 
    //}

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();

    world.step(1.0f/1000.0f);

    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/
PVector device_to_graphics(PVector deviceFrame) {
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}

PVector graphics_to_device(PVector graphicsFrame) {
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
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
  ColorPalette palette = palettes.get(paletteIndex);
  for (int i=0; i<palette.getLength(); i++) {
    //if (colorSwatch[i].isTouchingBody(s.h_avatar)) {
    //    setDrawingColor(palette.getSwatch(i).getColor());
    //}
  }
}


void setDrawingColor(int r, int g, int b) {
  colR = r;
  colG = g;
  colB = b;
  s.h_avatar.setFill(colR, colG, colB);
  //print("here");
  //splat = createGraphics(graphW, graphH, JAVA2D);
  //splat.beginDraw();
  //splat.colorMode(RGB, 255);
  //splat.fill(colR,colG,colB);
  //splat.noStroke();
  //splat.ellipse(s.h_avatar.getX()*40, s.h_avatar.getY()*40, 10,10);
  //splat.endDraw();
  //outputSplat.beginDraw();
  //outputSplat.image(splat, s.h_avatar.getX(), s.h_avatar.getY());
  //outputSplat.endDraw();
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

void createMenu() {

  menu              = new FBox(4, 20);
  menu.setFill(100, 100, 100);
  menu.setPosition(28, 10);
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
      break;
    case "/next":
      paletteIndex = (paletteIndex + 1) % (NUM_PALETTES);
      updateColorPicker(palettes.get(paletteIndex));
      break;
    case "/save":
      //outputSplat.endDraw();
      removepalette();
      delay(500);
      //output.save("./saved/test"+year()+month()+day()+"-"+hour()+minute()+second()+".png");
      //save("./saved/art-"+year() + nf(month(), 2) + nf(day(), 2) + "-" + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2) + ".png");   
      removed = true;
      //outputSplat.beginDraw();
      //addpalette();
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
    .setColorBackground(color(47, 0, 79))

    ;
  cp5.addButton("next")
    .setLabel("next")
    .setPosition(1075, 160)
    .setSize(100, 30)
    .setColorBackground(color(47, 0, 79))

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

void removepalette() {  
  for (int j=0; j<10; j++) {
    disappear(palettes.get(j));
  }
  //cp5.getController("prev").hide();
  //cp5.getController("next").hide();
  //cp5.getController("save").hide();
  //cp5.getController("Return").hide();
  ////cp5.getController("prev").hide();
  //world.remove(menu);
}

void disappear(ColorPalette palette)
{
  for (int i=0; i<6; i++) {
    world.remove(colorSwatch[i]);
  }
}

void addpalette() {
  for (int j=0; j<10; j++) {
    appear(palettes.get(j));
  }
  cp5.getController("prev").show();
  cp5.getController("next").show();
  cp5.getController("save").show();
  cp5.getController("Return").show();
  //cp5.getController("prev").hide();
  world.add(menu);
}

void appear(ColorPalette palette)
{
  for (int i=0; i<6; i++) {
    world.add(colorSwatch[i]);
  }
}

/* end helper functions section ****************************************************************************************/
