class Obstacle{
  PVector loc;
  float size;
 
  Obstacle(float x, float y, float size_){
    loc = new PVector(x,y);
    size = size_;
  }
  
  void display(){
    fill(255,0,0);
    ellipse(loc.x,loc.y,size,size);
  }
}
