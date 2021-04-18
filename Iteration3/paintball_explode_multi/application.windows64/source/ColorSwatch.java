public class ColorSwatch{
  private int r;
  private int g;
  private int b;
  private int paletteIndex;
  
  public ColorSwatch(int r, int g, int b, int index){
    this.r = r;
    this.g = g;
    this.b = b;
    this.paletteIndex = index;
  }
  
  public ColorSwatch(int r, int g, int b){
    this(r, g, b, 0);
  }
  
  public ColorSwatch(int[] c){
    this(c[0], c[1], c[2], 0);
  }
  
  //default to black
  public ColorSwatch(){
   this(0, 0, 0, 0); 
  }
  
  public void setPaletteIndex(int index){
    paletteIndex = index;
  }
  
  public int getPaletteIndex(){
    return paletteIndex;
  }
  
  public int getRed(){
    return this.r;
  }
  
  public int getGreen(){
    return this.g;
  }
  
  public int getBlue(){
    return this.b;
  }
  
  public int[] getColor(){
    int[] ret = {this.r, this.g, this.b};
    return ret;
  }
  
  public void setColor(int[] c){
    this.r = c[0];
    this.g = c[1];
    this.b = c[2];
  }
  
  public void setColor(int r, int g, int b){
    this.r = r;
    this.g = g;
    this.b = b;
  }
  
}
