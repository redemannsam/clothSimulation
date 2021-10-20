//Cloth Simulation
public void settings() {
  size(400, 500, "processing.opengl.PGraphics3D");
}
Camera camera;
//Create Window
String windowTitle = "Swinging Rope";
void setup() {
  surface.setTitle(windowTitle);
  initScene();
  camera = new Camera();
  img = loadImage("cloth.jpg"); 
}

//Simulation Parameters
PImage img;

float updateRate=20;
float sphereRad =50;
float floor = 500;
Vec3 gravity = new Vec3(0,400, 0);
float radius = 2;
Vec3 stringTop = new Vec3(60, -50, 0);
float restLen = 4;
float mass = 0.1; //TRY-IT: How does changing mass affect resting length of the rope?
float k = 1000; //TRY-IT: How does changing k affect resting length of the rope?
float kv = 50; //TRY-IT: How big can you make kv?

//Initial positions and velocities of masses
static int maxNodes = 100;

Vec3 pos[][] = new Vec3[maxNodes][maxNodes];
Vec3 vel[][] = new Vec3[maxNodes][maxNodes];
Vec3 acc[][] = new Vec3[maxNodes][maxNodes];

int numNodes = 20;
int numRopes = 25;
void initScene(){
  for(int j=0; j<numRopes; j++){
    for (int i = 0; i < numNodes; i++){
      pos[j][i] = new Vec3(0,0,0);
      pos[j][i].x = stringTop.x-restLen*i;
      pos[j][i].y = stringTop.y; //Make each node a little lower
      pos[j][i].z = -sphereRad+restLen*j;
      vel[j][i] = new Vec3(0,0, 0);
    }
  }
}

void update(float dt){
  
  
  
  //Reset accelerations each timestep (momenum only applies to velocity)
  for(int j=0; j<numRopes; j++){
    acc[j][0] = new Vec3(0,0, 0);
    acc[j][0].add(gravity);
  //Compute (damped) Hooke's law for each spring
    for (int i = 0; i < numNodes-1; i++){
      Vec3 diff = pos[j][i+1].minus(pos[j][i]);
      float stringF = -k*(diff.length() - restLen);
      //println(stringF,diff.length(),restLen);
      
      Vec3 stringDir = diff.normalized();
      float projVbot = dot(vel[j][i], stringDir);
      float projVtop = dot(vel[j][i+1], stringDir);
      float dampF = -kv*(projVtop - projVbot);
      //Vec3 friction = vel[j][i].times(0.3);
      
      //println(stringF,dampF, friction);
      Vec3 force = stringDir.times(stringF+dampF);
      acc[j][i].add(force.times(-1.0/mass));
      acc[j][i+1] = new Vec3(0,0, 0);
      acc[j][i+1].add(gravity);
      acc[j][i+1].add(force.times(1.0/mass));
      
    }
  }
  
  for(int j=0; j<numRopes-1; j++){
  //Compute (damped) Hooke's law for each spring
    for (int i = 0; i < numNodes; i++){
      Vec3 diff = pos[j+1][i].minus(pos[j][i]);
      float stringF = -k*(diff.length() - restLen);
      //println(stringF,diff.length(),restLen);
      
      Vec3 stringDir = diff.normalized();
      float projVbot = dot(vel[j][i], stringDir);
      float projVtop = dot(vel[j+1][i], stringDir);
      float dampF = -kv*(projVtop - projVbot);
      //Vec3 friction = vel[j][i].times(0.3);
      
      //println(stringF,dampF, friction);
      Vec3 force = stringDir.times(stringF+dampF);
      acc[j][i].add(force.times(-1.0/mass));
      acc[j+1][i].add(force.times(1.0/mass));
    }
  }
  for(int j=0; j<numRopes; j++){
  //apply friction
    for (int i = 0; i < numNodes; i++){
      acc[j][i].subtract(vel[j][i].times(1));
    }
  }
  //Midpoint integration
  for(int j=0; j< numRopes; j++){
    for (int i = 1; i < numNodes; i++){
      vel[j][i].add(acc[j][i].times(dt*0.5));
      pos[j][i].add(vel[j][i].times(dt));
      vel[j][i].add(acc[j][i].times(dt*0.5));
      
    }
  }
  
  //Collision detection and response
  for(int j=0; j< numRopes; j++){
    for (int i = 0; i < numNodes; i++){
      if (pos[j][i].y+radius > floor){
        vel[j][i].y *= -.9;
        pos[j][i].y = floor - radius;
      }
      if(pos[j][i].length()<sphereRad+radius){
        pos[j][i]=pos[j][i].normalized().times(sphereRad+radius);
        Vec3 a=projAB(vel[j][i],pos[j][i]);
        vel[j][i] = vel[j][i].minus(a);
      }
    }
  }
  
}

//Draw the scene: one sphere per mass, one line connecting each pair
boolean paused = true;
void draw() {
  noStroke();
  camera.Update(1.0/frameRate);
  background(255);
  directionalLight(255, 255, 255, 0, 1, 0);
  //directionalLight(255, 255, 255, 0, -1, 0);
  //directionalLight(255, 255, 255, 0, 0, -1);
  directionalLight(255, 255, 255, -1, 0, 0);
  
  
  fill( 0, 0, 255 );
  
  sphere( sphereRad );
  
  if (!paused){
    for(int i=0; i<updateRate; i++){
      update(1/(2*updateRate*frameRate));
    }
  }
  fill(0,0,0);
  for(int j=0; j< numRopes-1; j++){
    for (int i = 0; i < numNodes-1; i++){
      beginShape();
      
      pushMatrix();
      translate(pos[j][i].x,pos[j][i].y, pos[j][i].z);
      texture(img);
      // vertex( x, y, z, u, v) where u and v are the texture coordinates in pixels
      vertex(0, 0,0, 0, 0);
      float scalar=1.01;
      vertex((pos[j+1][i].x - pos[j][i].x)*scalar, (pos[j+1][i].y- pos[j][i].y)*scalar, (pos[j+1][i].z-pos[j][i].z)*scalar, img.width, 0);
      vertex((pos[j+1][i+1].x - pos[j][i].x)*scalar,(pos[j+1][i+1].y - pos[j][i].y)*scalar, (pos[j+1][i+1].z - pos[j][i].z)*scalar, img.width, img.height);
      vertex((pos[j][i+1].x - pos[j][i].x)*scalar,(pos[j][i+1].y - pos[j][i].y)*scalar, (pos[j][i+1].z - pos[j][i].z)*scalar, 0, img.height);
      endShape();
      popMatrix();
      //pushMatrix();
      
      //line(pos[j][i].x,pos[j][i].y,pos[j][i+1].x,pos[j][i+1].y);
      //translate(pos[j][i+1].x,pos[j][i+1].y, pos[j][i+1].z);
      //sphere(radius);
      //popMatrix();
    }
  }
  
  if (paused)
    surface.setTitle(windowTitle + " [PAUSED]");
  else
    surface.setTitle(windowTitle + " "+ nf(frameRate,0,2) + "FPS");
}

void keyPressed(){
  camera.HandleKeyPressed();
  if (key == ' ')
    paused = !paused;
        
}

void keyReleased()
{
  camera.HandleKeyReleased();
}


///////////////////
// Vec2D Library
///////////////////

public class Vec3 {
  public float x, y, z;
  
  public Vec3(float x, float y, float z){
    this.x = x;
    this.y = y;
    this.z =z;
  }
  
  public String toString(){
    return "(" + x+ ", " + y +", "+z+")";
  }
  
  public float length(){
    return sqrt(x*x+y*y+z*z);
  }
  
  public float lengthSqr(){
    return x*x+y*y+z*z;
  }
  
  public Vec3 plus(Vec3 rhs){
    return new Vec3(x+rhs.x, y+rhs.y, z+rhs.z);
  }
  
  public void add(Vec3 rhs){
    x += rhs.x;
    y += rhs.y;
    z +=rhs.z;
  }
  
  public Vec3 minus(Vec3 rhs){
    return new Vec3(x-rhs.x, y-rhs.y, z-rhs.z);
  }
  
  public void subtract(Vec3 rhs){
    x -= rhs.x;
    y -= rhs.y;
    z -=rhs.z;
  }
  
  public Vec3 times(float rhs){
    return new Vec3(x*rhs, y*rhs, z*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
    z *= rhs;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y +z*z);
    x /= magnitude;
    y /= magnitude;
    z /=magnitude;
  }
  
  public Vec3 normalized(){
    float magnitude = sqrt(x*x + y*y + z*z);
    return new Vec3(x/magnitude, y/magnitude, z/magnitude);
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y+ z*z);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
      z *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y + z*z);
    x *= newL/magnitude;
    y *= newL/magnitude;
    z *= newL/magnitude;
  }
  
  public float distanceTo(Vec3 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    float dz = rhs.z -z;
    return sqrt(dx*dx + dy*dy +dz*dz);
  }
  
}

Vec3 interpolate(Vec3 a, Vec3 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec3 a, Vec3 b){
  return a.x*b.x + a.y*b.y +a.z*b.z;
}

Vec3 projAB(Vec3 a, Vec3 b){
  return b.times((a.x*b.x + a.y*b.y +a.z*b.z)/(b.length()*b.length()));
}
