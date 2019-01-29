import processing.serial.*;

Serial myPort;
int portIndex = 0;
String inString;
float c_Angle_X, c_Angle_Y, k_Angle_X, k_Angle_Y;
int lf = 10;      // ASCII linefeed
String[] list;

void draw_rect() {
  scale(5);
  beginShape(QUADS);
  
//Z+ (to the drawing area)
  fill(#00ff00);
  vertex(-30, -5, 20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  //Z-
  fill(#0000ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, 5, -20);
  vertex(-30, 5, -20);
  
  //X-
  fill(#ff0000);
  vertex(-30, -5, -20);
  vertex(-30, -5, 20);
  vertex(-30, 5, 20);
  vertex(-30, 5, -20);
  
  //X+
  fill(#ffff00);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(30, 5, -20);
  
  //Y-
  fill(#ff00ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(-30, -5, 20);
  
  //Y+
  fill(#00ffff);
  vertex(-30, 5, -20);
  vertex(30, 5, -20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  

  endShape();
  
  
}

void setup(){
  
  //Canvas setup  
    size(1400, 800, P3D);
    background(255);
    fill(200, 102, 0);
    stroke(0);
    strokeWeight(1);
    
  //Serial connection setup
    printArray(Serial.list());
    print(" Connecting to ", Serial.list()[portIndex]);
    myPort = new Serial(this, Serial.list()[portIndex], 115200);
    myPort.bufferUntil(lf);
   }
    
void draw(){
    background(255);
    
    pushMatrix();
    translate(width/3, height/2);
    if (millis() > 10000){
      list = split(inString, ',');
      c_Angle_X = float(list[0]);
      c_Angle_Y = float(list[1]);
    }
    rotateX(radians(c_Angle_X));
    rotateY(radians(c_Angle_Y));
    draw_rect();
    popMatrix();
    
    pushMatrix();
    translate(2*width/3, height/2);
    if (millis() > 10000){
      list = split(inString, ',');
      k_Angle_X = float(list[2]);
      k_Angle_Y = float(list[3]);
    }
    rotateX(radians(k_Angle_X));
    rotateY(radians(k_Angle_Y));
    draw_rect();
    popMatrix();
    
    if (millis() > 10000) {
    textSize(20);
    text("Comp_filteredX   = " + float(list[0]),15,20);
    text("Comp_filteredY   = " + float(list[1]),15,40);
    text("Kalman_filteredX = " + float(list[2]),30,20);
    text("Kalman_filteredY = " + float(list[3]),30,40);
    }
   }
  
void serialEvent(Serial p){
    inString = p.readString();
   }
   
