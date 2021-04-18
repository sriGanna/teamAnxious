
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;

public class Splat {
  public float x, y;
  public float rad;
  public PGraphics splat;

  public Splat(float x, float y, float rad) {
    this.x = x;
    this.y = y;
    this.rad = rad;
    splat = createGraphics(200, 200, JAVA2D); //change size!! 
    create();
  }

  public void create() {
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
  public void display() {
    outputSp
  void saveSplat() {
    splat.save("./saved/test.png");
  }lat.beginDraw();
    outputSplat.imageMode(CENTER);
    outputSplat.image(splat, x, y);
    outputSplat.endDraw();
  }
}
