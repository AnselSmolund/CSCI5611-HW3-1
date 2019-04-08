class Position{
  float f_score;
  float g_score;
  float h_score;
  PVector loc;
  Position cameFrom;
  ArrayList<Position> neighbors;
  Position(float x, float y){
    neighbors = new ArrayList<Position>();
    cameFrom = start;
    f_score = 0;
    g_score = 0;
    h_score = 0;
    loc = new PVector(x,y);
  }
  
  boolean inObstacle(Obstacle o){
    float v1 = (loc.x - o.loc.x) * (loc.x - o.loc.x);
    float v2 = (loc.y - o.loc.y) * (loc.y - o.loc.y);
    float d = sqrt(v1 + v2);  
  
    if(((d - moverSize) <= (o.size/2))) {
      return true;
    }else{
      
      return false;
    }

  }
  
  boolean hasClearPath(Position p, ArrayList<Obstacle> obs_list){
    
    float a = loc.x - p.loc.x;
    float b = loc.y - p.loc.y;
    float x = sqrt(a*a + b*b);
    
    for(int i = 0; i < obs_list.size(); i++){
      Obstacle o = obs_list.get(i);
      float n = (abs((o.loc.x - loc.x) * (p.loc.y - loc.y) - 
      (o.loc.y - loc.y) * (p.loc.x - loc.x)) / x - moverSize);
      if(n < o.size/2){
        return false;
      }
    }
    return true;
   }    
  

    
  void display(color c, float w){
    stroke(0);
    fill(c);
    ellipse(loc.x,loc.y,w,w);
  }
    
}
