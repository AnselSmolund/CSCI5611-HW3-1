class Mover{
  PVector loc;
  PVector vel;
  PVector ahead;
  PVector ahead2;
  PVector goal;
  PVector desired;
  PVector steering;
  PVector avoidance;
  float seperation_radius;
  int mass;
  color col;
  ArrayList<Position>path;
  ArrayList<Position>world;
  ArrayList<Position> openSet;
  ArrayList<Position> exploredSet;
  ArrayList<Edge> edges;
  Position end;
  Position start;
  int count;
  int moverPos;
  float moverPerc;
  int currentNode;
  float MAX_FORCE = 5.4;
  float MAX_VELOCITY = 2;
  boolean finished = false;
  float MAX_AVOID_AHEAD = 20;
  float AVOID_FORCE = 600;
  PVector finishNode;
  Mover(PVector loc_,int mass_, PVector goal_, float sep){
    //MAX_FORCE = random(0,10);
    loc = loc_; 
    vel = new PVector(0,0);
    desired = new PVector(0,0);
    steering = new PVector(0,0);
    ahead = new PVector(0,0);
    avoidance = new PVector(0,0);
    mass = mass_;
    goal = goal_;
    finishNode = goal_.copy();
    
    end = new Position(goal_.x,goal_.y);
    start = new Position(loc_.x,loc_.y);

    col = color(random(255),random(255),random(255));
    seperation_radius = sep;
    path = new ArrayList<Position>();
    world = new ArrayList<Position>();
    world.add(start);
    world.add(end);
    count = 0;
    edges = new ArrayList<Edge>();
    openSet = new ArrayList<Position>();
    openSet.add(start);
    exploredSet = new ArrayList<Position>();
    vel.limit(MAX_VELOCITY);
 
    
    moverPos = 0;
    moverPerc = 0;
    currentNode = 0;

  }
  void buildRoadMap(){
    int total = 0;

  
    while(total <= 200){
      Position p = new Position(random(0,width-moverSize),random(0,height-moverSize));  
      for(Obstacle o : obstacles){     
        if(!p.inObstacle(o)){
          world.add(p);
          total++;
        }  
      }
    }

    for(int i = 0; i < world.size(); i++){  
      Position p = world.get(i);
      for(int j = 0; j < world.size(); j++){
        if(p.hasClearPath(world.get(j), obstacles) && i != j){ 
  
          edges.add(new Edge(p,world.get(j)));
          p.neighbors.add(world.get(j));
        }     
      }
    }
    
  }
  void reconfigureRoute(){
    start = new Position(loc.x,loc.y);
 
    openSet.clear();
    exploredSet.clear();
    path.clear();
    world.clear();
    world.add(start);
    world.add(end);
    buildRoadMap();
    openSet.add(start);
    moverPerc = 0;
    moverPos = 0;
    currentNode = 0;
    count = 0;
    findShortestPath();
  }
  void findShortestPath(){
    Position currentPos = start;
    //for(int i = 0; i < world.size(); i++){
      
    //  Position p = world.get(i);
    //  for(int j = 0; j < world.size(); j++){
    //    if(p.hasClearPath(world.get(j), obs) && i != j){       
    //      edges.add(new Edge(p,world.get(j)));
    //      p.neighbors.add(world.get(j));
    //    }     
    //  }
    //}

    while(openSet.size()>0){
      int curIndex = 0;
      for(int i = 0; i < openSet.size();i++){
        if(openSet.get(i).f_score < openSet.get(curIndex).f_score){
          curIndex = i;
        }
    }
    currentPos = openSet.get(curIndex);
    currentPos.display(color(255,255,0),10);
    
    if(currentPos == end){
     
    
      path = reconstructPath();     
    }
    
    openSet.remove(curIndex);
    exploredSet.add(currentPos);
    
    for(int i = 0; i < currentPos.neighbors.size(); i++){

      Position curr = currentPos.neighbors.get(i);
      if(exploredSet.contains(curr)){
        continue;
      }
      float tentative_gScore = currentPos.g_score + costEstimate(currentPos, curr);
      if(!openSet.contains(curr)){
        openSet.add(curr);
      }else if(tentative_gScore >= curr.g_score){
        continue;
      }
      
      curr.cameFrom = currentPos;
      curr.g_score = tentative_gScore;
      curr.f_score = curr.g_score + costEstimate(curr,end);

      }
    } 
  } 
  ArrayList reconstructPath(){
    ArrayList<Position> path = new ArrayList<Position>();
    
    Position tmp = end;
    
    path.add(tmp);
    while(tmp != start){
      path.add(tmp.cameFrom); 
      tmp = tmp.cameFrom;
    }
    currentNode = path.size() - 1;
    return path;
  }
  PVector seperate(ArrayList<Mover> movers){
    PVector force = new PVector(0,0);
    int count = 0;
    for(Mover m : movers){
      float d = PVector.dist(loc,m.loc);
      if((d>0) && (d < moverSize)){
        PVector diff = PVector.sub(m.loc,loc);
      //  diff.normalize();
       // diff.div(d);
        force.add(diff);
        count++;
      }
    }
    if(count > 0){
      force.div(count);
      force.mult(-1);
    }
    
    force.normalize();
    force.mult(10);
    //if(force.mag() > 0){
    //  force.normalize();
    //  force.mult(MAX_VELOCITY);
    //  force.sub(vel);
    //  force.limit(MAX_FORCE);
    //}
    return force;
  }
      
  void update(){
   // steering = arrive(goal);

    if(round(loc.y) == round(end.loc.y) && !finished){

      order.add(map(MAX_FORCE,0,10,0,255));
      finished = true;
    }
    if(path.size() > 0){

      if(currentNode == 0){
        steering = arrive(goal);
      }else{       
        steering = arrive(path.get(currentNode).loc);
      }
      if(PVector.dist(loc,path.get(currentNode).loc) < 50){     
        currentNode--;
        if(currentNode < 0){
          currentNode = 0;
        }
      }

    }    

  
    steering.add(seperate(ms));
    steering.add(checkCollision(obstacles));
    

    steering.limit(MAX_FORCE);
    //steering.mult(.1);
    steering.mult(0.5);
    vel = PVector.add(vel,steering);
    vel.limit(MAX_VELOCITY);
    loc = PVector.add(loc,vel);
  }

  
  boolean lineIntersectsCircle(PVector position, PVector ahead, Obstacle obs){
    PVector tempv = vel.copy();
    tempv.normalize();
    tempv.mult(MAX_AVOID_AHEAD * 0.5 * vel.mag() / MAX_VELOCITY);
    ahead2 = position.copy().add(tempv);
    if(PVector.dist(obs.loc,ahead) <= obs.size/2 || PVector.dist(obs.loc,ahead2) <=obs.size/2 || PVector.dist(obs.loc,loc) - moverSize/2 <= obs.size/2){
      return true;
    }else{
      return false;
    }
  } 
  PVector seek(PVector target){
    PVector force = new PVector(0,0);
    desired = target.sub(loc);
    desired.normalize();
    desired.mult(MAX_VELOCITY);
    force = desired.sub(vel);
    return force;
  }
  
  PVector arrive(PVector t){
    PVector target = t.copy();
    PVector desired = PVector.sub(target,loc);
    float d = desired.mag();
    desired.normalize();
    
    if(d < 100){
      float m = map(d,0,100,0,3);
      desired.mult(m);
    }
    else{
      desired.mult(3);
    }
    PVector steer = PVector.sub(desired,vel);
    steer.limit(1.1);
    return steer;
  }
  
  void display(){
    fill(col);
    ellipse(loc.x,loc.y,moverSize,moverSize);
    ellipse(goal.x,goal.y,moverSize,moverSize);
  }
  

  void checkBoundaries(){
    if(loc.x + moverSize/2 > width){
      loc.x = width - moverSize/2;
    }
    if(loc.x - moverSize/2 < 0){
      loc.x = moverSize/2;
    }

  }
  PVector checkCollision(ArrayList<Obstacle> obs){
    PVector tempv = new PVector();
    tempv.x = vel.x;
    tempv.y = vel.y;
    tempv.mult(MAX_AVOID_AHEAD * vel.mag() / MAX_VELOCITY);
    ahead = PVector.add(loc,tempv);
    //line(loc.x,loc.y,ahead.x,ahead.y);

    
    //Obstacle closest = null;
    //if(obstacles.size() > 1){
    //  for(int i = 0; i < obstacles.size();i++){
    //    Obstacle o = obstacles.get(i);   
    //    if(lineIntersectsCircle(loc,ahead,o) && (closest == null || PVector.dist(loc,o.loc) < PVector.dist(loc,closest.loc))){
    //      closest = o;
    //    }
    //  }
    //}else{
    //  closest = obstacles.get(0);
    //}
    Obstacle danger = null;
    for(int i = 0; i < obs.size(); i++){
      Obstacle o = obs.get(i);
      if(lineIntersectsCircle(loc,ahead,o)){
        danger = o;
      }
    }
      
    if(danger!=null){
      avoidance.x = ahead.x - danger.loc.x;
      avoidance.y = ahead.y - danger.loc.y;
      avoidance.normalize();
      avoidance.mult(AVOID_FORCE);     
    }
    else{
        avoidance.mult(0);
     }
    
    return avoidance;

    
  }

  
  boolean inCircle(PVector circle, float size){
    float v1 = (loc.x - circle.x) * (loc.x - circle.x);
    float v2 = (loc.y - circle.y) * (loc.y - circle.y);
    float d = sqrt(v1 + v2);  
  
    if(((d - size) <= (size))) {
      return true;
    }
    else{
      return false;
    }
  }
}
