float x_origin;
float y_origin;
float scaling = 300;
Point p_origin = new Point();


int tstep = 0;
boolean recording = false;

public class Point{
  float x;
  float y;
 
};

Point transform(Point p){
  Point p_transformed = new Point();
  p_transformed.x = p_origin.x + scaling*p.x;
  p_transformed.y = p_origin.y - scaling*p.y;
  return p_transformed;
}

void draw_pendulum(float theta){

  translate(width/2, height/2);
  rotate(theta-PI/2);
  
  strokeWeight(3);
  line(0,0,1*scaling,0);
  
  strokeWeight(1);
  circle(1*scaling,0,0.2*scaling);
}

void setup(){
  size(800, 800);
  x_origin = width/2;
  y_origin = height/2;
  p_origin.x = x_origin;
  p_origin.y = y_origin;
  
  draw_pendulum(PI);
}

void draw(){
}
