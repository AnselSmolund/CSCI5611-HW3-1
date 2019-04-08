class Edge{
  Position p1;
  Position p2;
  Edge(Position p1_, Position p2_){
    p1 = p1_;
    p2 = p2_;
  }
  
  void display(float alph){
    strokeWeight(1);
   // stroke(0,0,0,alph);
    line(p1.loc.x,p1.loc.y,p2.loc.x,p2.loc.y);
  }
}   
