Mover m;
float moverSize = 10;
ArrayList<Mover> ms;
ArrayList<Obstacle> obstacles;
Obstacle obs;
Obstacle obs2;
ArrayList<Float> order;
boolean stillWorking;
Position start;

PVector goal;
void setup(){
  size(800,800);
  ms = new ArrayList<Mover>();
  obstacles = new ArrayList<Obstacle>(); 
  order = new ArrayList<Float>();
  stillWorking = true;
  for(int i = 0; i < 100; i++){
    PVector start = new PVector(random(moverSize,width-moverSize),random(0,100));
    PVector goal = new PVector(random(moverSize,width-moverSize),random(600,height-moverSize));

    //if(i%2 == 0){
    //  start.x = 10 + (i * moverSize);// + random(-50,50);
    //  start.y = 100;// + random(-50,50);
    //  goal.x = 10 + (i * moverSize);// + random(-50,50);
    //  goal.y = 700;// + random(-50,50);
    //}
    //if(i%2 == 1){
    //  start.x = 10 + (i * moverSize);//+ random(-50,50);
    //  start.y = 700;//+ random(-50,50);
    //  goal.x = 10 + (i * moverSize);//+ random(-50,50);
    //  goal.y = 100;//+ random(-50,50);
    //}
    ////if(i%4 == 2){
    ////  start.x = 700;//+ random(-50,50);
    ////  start.y = 700;//+ random(-50,50);
    ////  goal.x = 100;//+ random(-50,50);
    ////  goal.y = 100;// + random(-50,50);
    ////}
    ////if(i%4 == 3){
    ////  start.x = 100;// + random(-50,50);
    ////  start.y = 700;// + random(-50,50);
    ////  goal.x = 700;// + random(-50,50);
    ////  goal.y = 100;// + random(-50,50);
    ////}
    Mover m = new Mover(start,5,goal, (i * 40) + 20);
    boolean in = false;
    for(int j = 0; j < ms.size(); j++){
      if(inside(m.goal,ms.get(j).goal,moverSize/2)){
        in = true;
      }
    }
    if(!in){
      ms.add(m);
    }

  }

  
  Obstacle o1 = new Obstacle(100,height/2,150);
  obstacles.add(o1);
  Obstacle o2 = new Obstacle(300,height/2,150);
  obstacles.add(o2);
  Obstacle o3 = new Obstacle(500,height/2,150);
  obstacles.add(o3);
  Obstacle o4 = new Obstacle(700,height/2,150);
  obstacles.add(o4);
  
  goal = new PVector(700,700);
  
  for(Mover m : ms){

    m.buildRoadMap();
    

  }
  for(Mover m : ms){
    m.findShortestPath();

  }
 // obs2 = new Obstaclght-180,400);
  
}

void draw(){
  background(127);
  for(Mover m : ms){
    for(int i = 0; i < m.path.size()-1;i++){
      stroke(m.col);
      line(m.path.get(i).loc.x,m.path.get(i).loc.y,m.path.get(i+1).loc.x,m.path.get(i+1).loc.y);
      stroke(0);
    } 
   // m.checkBoundaries();
    m.update();
    m.display();
  }
  for(Obstacle o : obstacles){
    o.display();
  }
  
  int count = 0;
  for(Mover m : ms){
    if(!m.finished){
      count++;
    }
  }
  if(count == 0){
    stillWorking = false;
  }
  
  if(!stillWorking){
    for(int i = 0; i < order.size(); i++){
      fill(order.get(i));
      ellipse(i * moverSize + 10,height-300,moverSize,moverSize);
    }
  }
    
}

float costEstimate(Position p1, Position p2){
  float v1 = (p2.loc.x - p1.loc.x) * (p2.loc.x - p1.loc.x);
  float v2 = (p2.loc.y - p1.loc.y) * (p2.loc.y - p1.loc.y);
  
  return (float)Math.sqrt(v1 + v2);
}

void mousePressed(){
  
  obstacles.add(new Obstacle(mouseX,mouseY,100));
  for(Mover m : ms){
    m.reconfigureRoute();
  }
}
void keyPressed(){
  setup();
}

boolean inside(PVector a, PVector b, float size){
  float v1 = (a.x - b.x) * (a.x - b.x);
  float v2 = (a.y - b.y) * (a.y - b.y);
  float d = sqrt(v1 + v2);  

  if(((d - size) <= (size))) {
    return true;
  }
  else{
    return false;
  }
}
  
