public class ColorPalette{
  public ColorSwatch[] palette = new ColorSwatch[6];
  
  public ColorPalette(ColorSwatch[] p){
    for(int i=0; i<p.length; i++){
      this.palette[i] = p[i];
    }
  }
  
  public ColorPalette(ColorSwatch s0, ColorSwatch s1, ColorSwatch s2, ColorSwatch s3, ColorSwatch s4, ColorSwatch s5){
    this(new ColorSwatch[] {s0, s1, s2, s3, s4, s5});
  }
  
  //create empty (black) color palette
  public ColorPalette(){
    this(new ColorSwatch[] {});
  }
  
  public int getLength(){
    return palette.length;
  }
  
  //returns black if swatch doesn't exist
  public ColorSwatch getSwatch(int index){
    if(this.palette[index] != null){
      return this.palette[index];
    }
    return new ColorSwatch(0, 0, 0);
  }
  
  //returns previous color or black if no previous color  
  public ColorSwatch setSwatch(int index, ColorSwatch swatch){
    ColorSwatch ret = this.getSwatch(index);
    this.palette[index] = swatch;
    return ret;
  }
  
  //returns previous color or black if no previous color
  public ColorSwatch addSwatch(ColorSwatch swatch){
    ColorSwatch ret = this.getSwatch(this.palette.length);
    this.palette[this.palette.length] = swatch;
    return ret;
  }
  
  //removes swatch at index and replaces with black, returns original color or black if no original color
  public ColorSwatch removeSwatch(int index){
    ColorSwatch ret = this.getSwatch(index);
    this.palette[index] = new ColorSwatch(0, 0, 0);
    return ret;
  }
  
}
