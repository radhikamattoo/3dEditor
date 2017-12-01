
// OpenGL Helpers to reduce the clutter
#include "Helpers.h"

// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>

// Linear Algebra Library
#include <Eigen/Core>

#include <fstream>

// For A x = b solver
#include <Eigen/Dense>

#include <Eigen/Geometry>

// IO Stream
#include <iostream>

#include <vector>

// Timer
#include <chrono>

#include <cmath>

#include <math.h>

#define PI 3.14159265

using namespace std;
using namespace Eigen;

// Vertices
VertexBufferObject VBO;
MatrixXf V(3,36);

// Colors
VertexBufferObject VBO_C;
MatrixXf C(3,36);

// Normals
VertexBufferObject VBO_N_F;
VertexBufferObject VBO_N_V;
MatrixXf N_faces(3,36); //per triangle normal
MatrixXf N_vertices(3,36); //per vertex normal
vector<int> rendering;

// Create an object type
enum ObjectType { Unit, Bunny, Bumpy };
vector<ObjectType> types;

// Create a rendering type
enum RenderType { Fill, Wireframe, Flat, Phong };
vector<RenderType> renders;

// Orthographic or perspective projection?
bool ortho = false;

// Number of objects existing in the scene
int numObjects = 0;

// Keeping track of mouse position
double currentX, currentY, previousX, previousY;

// Is an object selected?
bool selected = false;
int selected_index = -1;
bool selectedPress = false;

// Light position
Vector3f lightPos(2.0, 2.0, 1.0);

// Amount to divide/multiply vertices by in orthographic projection
int ORTHO_FACTOR = 70;

//----------------------------------
// VERTEX/TRANSFORMATION/INDEX DATA
//----------------------------------
Eigen::MatrixXf colors(3, 36); // dynamically resized per object
Eigen::Matrix4f orthographic(4,4);
Eigen::Matrix4f perspective(4,4);
Eigen::Matrix4f projection(4,4);
Eigen::Matrix4f view(4,4); // control camera position
Eigen::MatrixXf translation(4,4); // dynamically resized per object
Eigen::MatrixXf rotation(4,4); // dynamically resized per object
Eigen::MatrixXf scaling(4,4); // dynamically resized per object
Eigen::MatrixXf model(4,4); // dynamically resized per object
Eigen::MatrixXf MVP(4,4); // dynamically resized per object

//----------------------------------
// OFF DATA
//----------------------------------
pair<MatrixXd, MatrixXd> bunny;
pair<MatrixXd, MatrixXd> bumpy;

//----------------------------------
// VIEW MATRIX PARAMETERS
//----------------------------------
float focal_length = 1.0;
Vector3f eye(-2.0, -2.0, focal_length); //camera position/ eye position  //e
Vector3f look_at(0.0, 0.0, 0.0); //target point, where we want to look //g
Vector3f up_vec(0.0, 1.0, 0.0); //up vector //t

//----------------------------------
// PERSPECTIVE PROJECTION PARAMETERS
//----------------------------------
// FOV angle is hardcoded to 60 degrees
float theta = (PI/180) * 60;

// near and far are hardcoded
float n = -0.1;
float f = -100.;
// right and left
float r;
float l;
// top and bottom
float t;
float b;
float aspect;


vector<float> split_line(string line, bool F)
{
  string extracted;
  vector<float> data;
  int z = 0;

  // # of faces is the first character in every line, start at 2 to skip
  if(F){
    z = 2;
  }
  for(int i = z; i <= line.length(); i++){

    char val = line[i];

    if(val == ' ' || i == line.length()){ // Finished building int
      // Convert to int and push to data vector
      data.push_back(atof(extracted.c_str()));
      extracted = "";
    }else{ // Still building int
      extracted.push_back(val);
    }
  }
  return data;
}
// Iterates through given OFF file and fills V & F matrices
pair<MatrixXd, MatrixXd> read_off_data(string filename, bool enlarge)
{
  // Load file
  string line;
  ifstream stream(filename.c_str());
  getline(stream, line); //first line is OFF

  // Get data from 2nd line
  getline(stream, line);
  vector<float> data = split_line(line, false);

  // Extract metadata into vars
  int vertices = data[0];
  int faces = data[1];
  MatrixXd V = MatrixXd::Zero(vertices, 3);
  MatrixXd F = MatrixXd::Zero(faces, 3);
  vector<float> line_data;

  // Fill V & F matrices from file
  for(int v = 0; v < vertices; v++){
    getline(stream, line);
    line_data = split_line(line, false);

    for(int j = 0; j < 3; j++){
      if(enlarge){
        V(v,j) = (line_data[j]*7);
      }else{
        V(v,j) = (line_data[j]/7);
      }
      if(!enlarge){ //cube
        // V(v,0) += 0.1;
        // V(v,1) += 0.1;
      }else{ //bunny
        V(v,0) += 0.05;
        V(v,1) -= 0.35;
      }
    }

  }

  for(int f = 0; f < faces; f++){
    getline(stream, line);
    line_data = split_line(line, true);
    for(int j = 0; j < 3; j++){
      F(f,j) = line_data[j];
    }
  }

  // Construct pair and return
  pair<MatrixXd, MatrixXd> matrices(V, F);
  return matrices;

}
bool isInVector(vector<int> summed, int idx){
  for(int i = 0; i < summed.size(); i++){
    if(idx == summed[i]) return true;
  }
  return false;
}
// Iterates through the N buffer and adds in the normals
void addNormals(ObjectType type, int start)
{
  switch(type){
    case Unit:{
      // go through faces
      for(int i = start; i < start + 36; i+=3)
      {
        Vector3f edge1 = V.col(i + 1) - V.col(i);
        Vector3f edge2 = V.col(i + 2) - V.col(i);
        Vector3f normal = (edge1.cross(edge2)).normalized();
        N_faces.col(i) << normal;
        N_faces.col(i + 1) << normal;
        N_faces.col(i + 1) << normal;
        N_vertices.col(i) += normal;
        N_vertices.col(i + 1) += normal;
        N_vertices.col(i + 2) += normal;
      }

      for(int i = start; i < start + 36; i++){
        vector<int> summed;
        Vector3f current = V.col(i);
        Vector3f sum =  N_vertices.col(i);
        summed.push_back(i);
        for(int j = start; j < start + 36; j++){
          if(isInVector(summed, j)) continue;
          if(i == j) continue;
          Vector3f other = V.col(j);
          if(other[0] == current[0] && other[1] == current[1] && other[2] == current[2]){
            sum += N_vertices.col(j);
            summed.push_back(j);
          }
        }
        sum = sum.normalized();
        for(int k = 0; k < summed.size(); k++){
          int idx = summed[k];
          N_vertices.col(idx) = sum;
        }
      }

      VBO_N_V.update(N_vertices);
      VBO_N_F.update(N_faces);
      break;
    }

    case Bunny:{
      // go through faces
      for(int i = start; i < start + 3000; i+=3)
      {
        Vector3f edge1 = V.col(i + 1) - V.col(i);
        Vector3f edge2 = V.col(i + 2) - V.col(i);
        Vector3f normal = (edge1.cross(edge2)).normalized();
        N_faces.col(i) << normal;
        N_faces.col(i + 1) << normal;
        N_faces.col(i + 1) << normal;
        N_vertices.col(i) += normal;
        N_vertices.col(i + 1) += normal;
        N_vertices.col(i + 2) += normal;
      }

      for(int i = start; i < start + 3000; i++){
        vector<int> summed;
        Vector3f current = V.col(i);
        Vector3f sum =  N_vertices.col(i);
        summed.push_back(i);
        for(int j = start; j < start + 3000; j++){
          if(isInVector(summed, j)) continue;
          if(i == j) continue;
          Vector3f other = V.col(j);
          if(other[0] == current[0] && other[1] == current[1] && other[2] == current[2]){
            sum += N_vertices.col(j);
            summed.push_back(j);
          }
        }
        sum = sum.normalized();
        for(int k = 0; k < summed.size(); k++){
          int idx = summed[k];
          N_vertices.col(idx) = sum;
        }
      }
      cout << N_vertices << endl;

      // cout << "N_Vertices is: \n" << N_vertices.block(0, start, 3, 3) << endl;

      VBO_N_V.update(N_vertices);
      VBO_N_F.update(N_faces);
      break;
    }

    case Bumpy:
      // go through faces
      for(int i = start; i < start + 3000; i+=3)
      {
        Vector3f edge1 = V.col(i + 1) - V.col(i);
        Vector3f edge2 = V.col(i + 2) - V.col(i);
        Vector3f normal = (edge1.cross(edge2)).normalized();
        N_faces.col(i) << normal;
        N_faces.col(i + 1) << normal;
        N_faces.col(i + 1) << normal;
        N_vertices.col(i) += normal;
        N_vertices.col(i + 1) += normal;
        N_vertices.col(i + 2) += normal;
      }

      for(int i = start; i < start + 3000; i++){
        vector<int> summed;
        Vector3f current = V.col(i);
        Vector3f sum =  N_vertices.col(i);
        summed.push_back(i);
        for(int j = start; j < start + 3000; j++){
          if(isInVector(summed, j)) continue;
          if(i == j) continue;
          Vector3f other = V.col(j);
          if(other[0] == current[0] && other[1] == current[1] && other[2] == current[2]){
            sum += N_vertices.col(j);
            summed.push_back(j);
          }
        }
        sum = sum.normalized();
        for(int k = 0; k < summed.size(); k++){
          int idx = summed[k];
          N_vertices.col(idx) = sum;
        }
      }
      cout << "N_Vertices is: \n" << N_vertices.block(0, start, 3, 3) << endl;
      VBO_N_V.update(N_vertices);
      VBO_N_F.update(N_faces);
      break;
  }
}
void initialize(GLFWwindow* window)
{
  VBO.init();
  VBO_C.init();
  VBO_N_V.init();
  VBO_N_F.init();

  //------------------------------------------
  // VERTEX DATA
  //------------------------------------------
   V <<
  -0.0, -0.0,  0.0,
   0.0, -0.0,  0.0,
   0.0, -0.0, -0.0, //bottom triangle
  -0.0, -0.0, -0.0,
  -0.0, -0.0,  0.0,
   0.0, -0.0, -0.0, //upper triangle
  -0.0, -0.0, -0.0,
   0.0, -0.0, -0.0,
   0.0,  0.0, -0.0, //bottom triangle
  -0.0,  0.0, -0.0,
  -0.0, -0.0, -0.0,
   0.0,  0.0, -0.0, //upper triangle
   -0.0, -0.0,  0.0,
   -0.0,  0.0,  0.0,
   -0.0, -0.0, -0.0, //bottom triangle
   -0.0,  0.0,  0.0,
   -0.0, -0.0, -0.0,
   -0.0,  0.0, -0.0, //upper triangle
   0.0, -0.0,  0.0,
   0.0,  0.0,  0.0,
   0.0, -0.0, -0.0, //bottom triangle
   0.0, -0.0, -0.0,
   0.0,  0.0,  0.0,
   0.0,  0.0, -0.0, //upper triangle
   0.0,  0.0,  0.0,
   0.0,  0.0, -0.0,
   -0.0, 0.0, 0.0, //bottom triangle
   -0.0, 0.0, -0.0,
   0.0,  0.0, -0.0,
   -0.0, 0.0, 0.0, //upper triangle
   -0.0, -0.0,  0.0,
    0.0, -0.0,  0.0,
    0.0,  0.0,  0.0, //bottom triangle
   -0.0, -0.0,  0.0,
    0.0,  0.0,  0.0,
   -0.0,  0.0,  0.0; //upper triangle
   VBO.update(V);

  //------------------------------------------
  // PROJECTION MATRIX
  //------------------------------------------
  // Get the size of the window
  int width, height;
  glfwGetWindowSize(window, &width, &height);
  aspect = width/height;

  t = tan(theta/2) * abs(n);
  b = -t;

  r = aspect * t;
  l = -r;

  // Apply projection matrix to corner points
  orthographic <<
  2/(r - l), 0., 0., -((r+l)/(r-l)),
  0., 2/(t - b), 0., -((t+b)/(t-b)),
  0., 0., 2/(abs(f)-abs(n)), -(n+f)/(abs(f)-abs(n)),
  0., 0., 0.,   1.;

  // orthographic <<
  // 2/(r - l), 0., 0., -((r+l)/(r-l)),
  // 0., 2/(t - b), 0., -((t+b)/(t-b)),
  // 0., 0., 2/(n-f), -(n+f)/(n-f),
  // 0., 0., 0.,   1.;


  // perspective maps a frustrum to a unit cube
  // take  vertex from each end of the frustrum and map them to the unit cube
  perspective <<
  2*abs(n)/(r-l), 0., (r+l)/(r-l), 0.,
  0., (2 * abs(n))/(t-b), (t+b)/(t-b), 0.,
  0., 0.,   (abs(f) + abs(n))/(abs(n) - abs(f)), (2 * abs(f) * abs(n))/(abs(n) - abs(f)),
  0., 0., -1., 0;

  if(ortho){
    projection = orthographic;
  }else{
    projection = perspective;
  }

  //------------------------------------------
  // VIEW/CAMERA MATRIX
  //------------------------------------------
  Vector3f w = (eye - look_at).normalized();
  Vector3f u = (up_vec.cross(w).normalized());
  Vector3f v = w.cross(u);

  Matrix4f look;
  look <<
  u[0], u[1], u[2], 0.,
  v[0], v[1], v[2], 0.,
  w[0], w[1], w[2], 0.,
  0.,   0.,    0.,  0.5;

  Matrix4f at;
  at <<
  0.5, 0.0, 0.0, -eye[0],
  0.0, 0.5, 0.0, -eye[1],
  0.0, 0.0, 0.5, -eye[2],
  0.0, 0.0, 0.0, 0.5;
  view = look * at;

  //------------------------------------------
  // MODEL MATRIX
  //------------------------------------------
  translation <<
  1., 0., 0., 0.,
  0., 1., 0., 0.,
  0., 0., 1., 0.,
  0., 0., 0., 1.;
  rotation <<
  1., 0., 0., 0.,
  0., 1., 0., 0.,
  0., 0., 1., 0.,
  0., 0., 0., 1.;
  scaling <<
  1., 0., 0., 0.,
  0., 1., 0., 0.,
  0., 0., 1., 0.,
  0., 0., 0., 1.;

  model = translation * rotation * scaling;

  //------------------------------------------
  // MVP MATRIX
  //------------------------------------------
  MVP = projection * view * model;

  //------------------------------------------
  // COLORS
  //------------------------------------------
  C <<
  0.0,  1.0,  0.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0,
  1.0,  1.0,  1.0;

  VBO_C.update(C);

  //------------------------------------------
  // NORMALS
  //------------------------------------------
  for(int i = 0; i < 36; i++){
    N_faces.col(i) << 0.0, 0.0, 0.0;
    N_vertices.col(i) << 0.0, 0.0, 0.0;

  }
  ObjectType type = Unit;
  addNormals(type, 0);

}
void addUnitCube()
{
  ObjectType t = Unit;
  types.push_back(t);
  RenderType r = Fill;
  renders.push_back(r);

  // Hold onto start index
  int start = 0;
  if(numObjects > 1){
    start = V.cols();
    V.conservativeResize(3, V.cols() + 36);
    N_faces.conservativeResize(3, V.cols() + 36);
    N_vertices.conservativeResize(3, V.cols() + 36);
    C.conservativeResize(3, V.cols() + 36);
    model.conservativeResize(4, 4 * numObjects);
    MVP.conservativeResize(4, 4 * numObjects);
    translation.conservativeResize(4, 4 * numObjects);
    rotation.conservativeResize(4, 4 * numObjects);
    scaling.conservativeResize(4, 4 * numObjects);

    translation.block(0, 4 * (numObjects-1), 4, 4) <<
    1., 0., 0., 0.,
    0., 1., 0., 0.,
    0., 0., 1., 0.,
    0., 0., 0., 1.;
    rotation.block(0, 4 * (numObjects-1), 4, 4) <<
    1., 0., 0., 0.,
    0., 1., 0., 0.,
    0., 0., 1., 0.,
    0., 0., 0., 1.;
    scaling.block(0, 4 * (numObjects-1), 4, 4) <<
    1., 0., 0., 0.,
    0., 1., 0., 0.,
    0., 0., 1., 0.,
    0., 0., 0., 1.;

    model.block(0, 4 * (numObjects-1), 4, 4) = translation.block(0, 4 * (numObjects-1), 4, 4) * rotation.block(0, 4 * (numObjects-1), 4, 4) * scaling.block(0, 4 * (numObjects-1), 4, 4);
    MVP.block(0, 4 * (numObjects-1), 4, 4) = projection * view *   model.block(0, 4 * (numObjects-1), 4, 4);
  }

  // Update sizes of all matrices

  // BOTTOM
  V.col(start) << 0.5f, -0.5f, 0.5f;
  V.col(start + 1) <<   0.5, -0.5, -0.5;
  V.col(start + 2) <<   -0.5,-0.5, -0.5;
  V.col(start + 3) << -0.5,-0.5, -0.5;
  V.col(start + 4) <<  -0.5, -0.5, 0.5;
  V.col(start + 5) <<  0.5, -0.5, 0.5;

  // BACK
  V.col(start + 6) << 0.5,  0.5, -0.5;
  V.col(start + 7) <<  -0.5, -0.5, -0.5;
  V.col(start + 8) << 0.5, -0.5, -0.5;
  V.col(start + 9) << 0.5,  0.5, -0.5;
  V.col(start + 10) << -0.5, -0.5, -0.5;
  V.col(start + 11) << -0.5,  0.5, -0.5;

  // LEFT
  V.col(start + 12) << -0.5,  0.5, -0.5;
  V.col(start + 13) << -0.5,  0.5,  0.5;
  V.col(start + 14) << -0.5, -0.5, -0.5;
  V.col(start + 15) << -0.5, -0.5, -0.5;
  V.col(start + 16) <<  -0.5,  0.5,  0.5;
  V.col(start + 17) <<  -0.5, -0.5,  0.5;

  // RIGHT
  V.col(start + 18) << 0.5, -0.5,  0.5;
  V.col(start + 19) << 0.5,  0.5,  0.5;
  V.col(start + 20) << 0.5, -0.5, -0.5;
  V.col(start + 21) << 0.5, -0.5, -0.5;
  V.col(start + 22) << 0.5,  0.5,  0.5;
  V.col(start + 23) << 0.5,  0.5, -0.5;

  // TOP
  V.col(start + 24) << 0.5,  0.5,  0.5;
  V.col(start + 25) << 0.5,  0.5, -0.5;
  V.col(start + 26) << -0.5, 0.5, 0.5;
  V.col(start + 27) << -0.5, 0.5, -0.5;
  V.col(start + 28) << 0.5,  0.5, -0.5;
  V.col(start + 29) << -0.5, 0.5, 0.5;

  // FRONT
  V.col(start + 30) <<  -0.5, -0.5,  0.5;
  V.col(start + 31) <<   0.5, -0.5,  0.5;
  V.col(start + 32) <<   0.5,  0.5,  0.5;
  V.col(start + 33) <<  -0.5,  0.5,  0.5;
  V.col(start + 34) <<   -0.5, -0.5,  0.5;
  V.col(start + 35) <<  0.5,  0.5,  0.5;

  // initialize C & N
  for(int i = start; i < start + 36; i++){
    N_vertices.col(i) << 0.0, 0.0, 0.0;
    C.col(i) << 1.0, 1.0, 0.0;
  }
  // Calculate normals for triangle vertices & faces
  addNormals(t, start);

  if(ortho){
    V.block(0, start, 3, 36) /= ORTHO_FACTOR;
  }
  VBO_C.update(C);
  VBO.update(V);


}
void addBunny()
{
  ObjectType t = Bunny;
  types.push_back(t);
  RenderType r = Fill;
  renders.push_back(r);

  // Create data matrices from OFF files
  MatrixXd V_bunny = bunny.first;
  MatrixXd F_bunny = bunny.second;

  // Hold onto start index
  int start = 0;
  if(numObjects > 1){
    start = V.cols();
    V.conservativeResize(3, V.cols() + 3000);
    N_faces.conservativeResize(3, N_faces.cols() + 3000);
    N_vertices.conservativeResize(3, N_vertices.cols() + 3000);
    C.conservativeResize(3, V.cols() + 3000);
    // Transformation matrices
    model.conservativeResize(4, 4 * numObjects);
    MVP.conservativeResize(4, 4 * numObjects);
    translation.conservativeResize(4, 4 * numObjects);
    rotation.conservativeResize(4, 4 * numObjects);
    scaling.conservativeResize(4, 4 * numObjects);

    translation.block(0, 4 * (numObjects-1), 4, 4) <<
    1., 0., 0., 0.,
    0., 1., 0., 0.,
    0., 0., 1., 0.,
    0., 0., 0., 1.;
    rotation.block(0, 4 * (numObjects-1), 4, 4) <<
    1., 0., 0., 0.,
    0., 1., 0., 0.,
    0., 0., 1., 0.,
    0., 0., 0., 1.;
    scaling.block(0, 4 * (numObjects-1), 4, 4) <<
    1., 0., 0., 0.,
    0., 1., 0., 0.,
    0., 0., 1., 0.,
    0., 0., 0., 1.;

    model.block(0, 4 * (numObjects-1), 4, 4) = translation.block(0, 4 * (numObjects-1), 4, 4) * rotation.block(0, 4 * (numObjects-1), 4, 4) * scaling.block(0, 4 * (numObjects-1), 4, 4);
    MVP.block(0, 4 * (numObjects-1), 4, 4) = projection * view *   model.block(0, 4 * (numObjects-1), 4, 4);
  }else{

    V.conservativeResize(3, 3000);
    N_faces.conservativeResize(3, 3000);
    N_vertices.conservativeResize(3, 3000);
    C.conservativeResize(3, 3000);

  }

  for(int i = 0; i < F_bunny.rows(); i++)
  {
    for(int j = 0; j < F_bunny.cols(); j++)
    {

      vector<float> vertices;
      int idx = F_bunny(i,j);
      // take the row from idx and push the 3 points for 1 vertex
      for(int x = 0; x < 3; x++){
        vertices.push_back(V_bunny(idx,x));
      }
      V.col(start + (i*3) + j) << vertices[0], vertices[1], vertices[2];
      C.col(start + (i*3) + j) << 0.0, 0.0, 1.0; // green
      N_vertices.col(start + (i*3) + j) << 0.0, 0.0, 0.0;
      N_faces.col(start + (i*3) + j) << 0.0, 0.0, 0.0;

    }
  }
  // cout << "C is now: \n" << C.block(0, 0, 3, 6) << endl;
  addNormals(t, start);

  if(ortho){
    V.block(0, start, 3, 3000) /= ORTHO_FACTOR;
  }
  VBO_C.update(C);
  VBO.update(V);

}
void addBumpy()
{
  ObjectType t = Bumpy;
  types.push_back(t);
  RenderType r = Fill;
  renders.push_back(r);

  // Create data matrices from OFF files
  MatrixXd V_bumpy = bumpy.first;
  MatrixXd F_bumpy = bumpy.second;
  // Hold onto start index
  int start = 0;
  if(numObjects > 1){
    start = V.cols();
    V.conservativeResize(3, V.cols() + 3000);
    N_faces.conservativeResize(3, N_faces.cols() + 3000);
    N_vertices.conservativeResize(3, N_vertices.cols() + 3000);
    C.conservativeResize(3, V.cols() + 3000);
    model.conservativeResize(4, 4 * numObjects);
    MVP.conservativeResize(4, 4 * numObjects);
    translation.conservativeResize(4, 4 * numObjects);
    rotation.conservativeResize(4, 4 * numObjects);
    scaling.conservativeResize(4, 4 * numObjects);

    translation.block(0, 4 * (numObjects-1), 4, 4) <<
    1., 0., 0., 0.,
    0., 1., 0., 0.,
    0., 0., 1., 0.,
    0., 0., 0., 1.;
    rotation.block(0, 4 * (numObjects-1), 4, 4) <<
    1., 0., 0., 0.,
    0., 1., 0., 0.,
    0., 0., 1., 0.,
    0., 0., 0., 1.;
    scaling.block(0, 4 * (numObjects-1), 4, 4) <<
    1., 0., 0., 0.,
    0., 1., 0., 0.,
    0., 0., 1., 0.,
    0., 0., 0., 1.;

    model.block(0, 4 * (numObjects-1), 4, 4) = translation.block(0, 4 * (numObjects-1), 4, 4) * rotation.block(0, 4 * (numObjects-1), 4, 4) * scaling.block(0, 4 * (numObjects-1), 4, 4);
    MVP.block(0, 4 * (numObjects-1), 4, 4) = projection * view *   model.block(0, 4 * (numObjects-1), 4, 4);
  }else{
    V.conservativeResize(3, 3000);
    N_faces.conservativeResize(3, 3000);
    N_vertices.conservativeResize(3, 3000);
    C.conservativeResize(3, 3000);
  }
  // cout << "New shape of V: " << V.rows() << "," << V.cols() << endl;
  // Iterate through columns of V and get 3 3D points to build 1 triangle
  // cout << "F_bumpy shape: " << F_bumpy.rows() << "," << F_bumpy.cols() << endl;
  for(int i = 0; i < F_bumpy.rows(); i++)
  {
    for(int j = 0; j < F_bumpy.cols(); j++)
    {
      vector<float> vertices;
      int idx = F_bumpy(i,j);
      // take the row from idx and push the 3 points for 1 vertex
      for(int x = 0; x < 3; x++){
        vertices.push_back(V_bumpy(idx,x));
      }
      V.col(start + (i*3) + j) << vertices[0], vertices[1], vertices[2];
      C.col(start + (i*3) + j) << 0.0, 1.0, 0.0;
      N_vertices.col(start + (i*3) + j) << 0.0, 0.0, 0.0;
      N_faces.col(start + (i*3) + j) << 0.0, 0.0, 0.0;

    }
  }

  addNormals(t, start);

  if(ortho){
    V.block(0, start, 3, 3000) /= ORTHO_FACTOR;
  }

  VBO_C.update(C);
  VBO.update(V);
}
// Translates triangle based on mouse movement
void translateTriangle()
{
  // Compare previousX and currentX, etc. and figure out translation
  float x_difference;
  float y_difference;
  // cout << "Previous (x,y): " << previousX << "," << previousY << endl;
  // cout << "Current (x,y): " << currentX << "," << currentY << endl;

  // cout << "Previous positions: " << previousX << " , " << previousY << endl;
  // cout << "Current positions: " << currentX << " , " << currentY << endl;
  if(currentX > previousX) // mouse moved right
  {
    x_difference = currentX - previousX;
    translation(0,selected_index + 3) += x_difference;
  }else{ // mouse moved left
    x_difference = previousX - currentX;
    translation(0, selected_index + 3) -= x_difference;
  }
  if(currentY > previousY) // mouse moved up
  {
    y_difference = currentY - previousY;
    translation(1,selected_index + 3) += y_difference;
  }else{ //mouse moved down
    y_difference = previousY - currentY;
    translation(1,selected_index + 3) -= y_difference;
  }

  model.block(0, selected_index, 4,4) = translation.block(0, selected_index, 4, 4) * rotation.block(0, selected_index, 4, 4) * scaling.block(0, selected_index, 4, 4);
  MVP.block(0, selected_index, 4, 4) = projection * view * model.block(0, selected_index, 4,4);
}
void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos)
{
  // Get the size of the window
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // Convert screen position to world coordinates
  Eigen::Vector4f p_screen(xpos,height-1-ypos,0,1);
  Eigen::Vector4f p_canonical((p_screen[0]/width)*2-1,(p_screen[1]/height)*2-1,0,1);
  Eigen::Vector4f p_world = view.inverse() * projection.inverse() * p_canonical;

  double xworld = p_world[0];
  double yworld = p_world[1];

  // Keep track of mouse positions
  if(!previousX && !previousY)
  {
    previousX = xworld;
    previousY = yworld;
  }else{
    previousX = currentX;
    previousY = currentY;

    currentX = xworld;
    currentY = yworld;
  }

}
// Solve for x, given Ax = b
vector<float> solver(Vector3f &a_coord, Vector3f &b_coord, Vector3f &c_coord, Vector3f &ray_direction, Vector3f &ray_origin)
{
  // Construct matrices/vectors to solve for
  Matrix3f A_;
  Vector3f b_;

  Vector3f a_minus_b = a_coord - b_coord;
  Vector3f a_minus_c = a_coord - c_coord;
  Vector3f a_minus_e = a_coord - ray_origin;

  A_ << a_minus_b[0], a_minus_c[0], ray_direction[0],   a_minus_b[1], a_minus_c[1], ray_direction[1], a_minus_b[2], a_minus_c[2], ray_direction[2];
  b_ << a_minus_e[0], a_minus_e[1], a_minus_e[2];

  // cout << "Here is the matrix A:\n" << A_ << endl;
  // cout << "Here is the vector b:\n" << b_ << endl;

  Vector3f sol = A_.colPivHouseholderQr().solve(b_);
  if(!(A_*sol).isApprox(b_, 0.003)){
    sol[0] = -1;
    sol[1] = -1;
    sol[2] = -1;
  }
  vector<float> solutions;

  // cout << "Here is the solution: " << sol << endl;
  solutions.push_back(sol[0]);
  solutions.push_back(sol[1]);
  solutions.push_back(sol[2]);
  return solutions;
}
// Translates indices from F matrix into 3D coordinates from V
vector<Vector3d> get_triangle_coordinates(MatrixXd &V, MatrixXd &F, unsigned row)
{
  Vector3f coordinates;
  for(unsigned y=0; y< F.cols();y++)
  {
    // Get F indices from row
    coordinates[y] = F(row,y);
  }
  // Now have indices to index into V with
  float a_component = coordinates[0];
  float b_component = coordinates[1];
  float c_component = coordinates[2];

  // cout << a_component << endl;
  // cout << b_component << endl;
  // cout <<"C index:" << c_component << endl;

  // Get triangle coordinates
  Vector3d a_coord = RowVector3d(V(a_component,0),V(a_component, 1),V(a_component, 2));
  Vector3d b_coord = RowVector3d(V(b_component,0),V(b_component, 1),V(b_component, 2));
  Vector3d c_coord = RowVector3d(V(c_component,0),V(c_component, 1),V(c_component, 2));

  // cout << "a_coord: " <<  a_coord << endl;
  // cout << "b_coord: " << b_coord << endl;
  // cout << "c_coord: " << c_coord << endl;

  vector<Vector3d> ret;
  ret.push_back(a_coord);
  ret.push_back(b_coord);
  ret.push_back(c_coord);
  return ret;
}

bool does_intersect(float t, float u, float v)
{
  if(t > 0 && u >= 0 && v >=0 && (u+v) <= 1){ return true; }
  return false;
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
  // Get the position of the mouse in the window
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Get the size of the window
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // Create data matrices from OFF files
    MatrixXd V_bumpy = bumpy.first;
    MatrixXd F_bumpy = bumpy.second;
    MatrixXd V_bunny = bunny.first;
    MatrixXd F_bunny = bunny.second;

    double aspect = double(width)/double(height);
    // cout << "Width: " << width << endl; //640
    // cout << "Height: " << height << endl; //480

    if(action == GLFW_PRESS){
      // Check if an object was clicked on and select it
      // TODO: ORTHOGRAPHIC PROJECTION NOT WORKING

      // Construct ray and convert to world coordinates
      Vector3f ray_direction(0., 0., -1.);

      Eigen::Vector3f ray_screen(xpos,height-1-ypos,0); //screen coordinates
      // cout << "Ray screen: \n" << ray_screen << endl;

      Eigen::Vector4f ray_canonical((ray_screen[0]/width)*2-1,(ray_screen[1]/height)*2-1,0,1); //canonical view volume
      // cout << "Ray canonical: \n" << ray_canonical << endl;

      // projection is either the orthographic or perpsective projection matrix
      Vector4f ray_projection = projection.inverse() * ray_canonical; //camera space
      // cout << "Ray projection: \n" << ray_projection << endl;

      Vector4f ray_world = view.inverse() * ray_projection; //world space
      // cout << "Ray world: \n" << ray_world << endl;

      Vector3f ray_origin(ray_world[0], ray_world[1], ray_world[2]);
      // cout << "Ray origin: \n" << ray_origin << endl;

      // selected = false;
      int start = 0;
      for(int t_idx = 0; t_idx < types.size(); t_idx++){
        ObjectType type = types[t_idx];
        switch(type){
          case Unit:
            for(int s = start; s < start + 36; s+=3){
              Vector3f coord1 = V.col(s);
              Vector3f coord2 = V.col(s + 1);
              Vector3f coord3 = V.col(s + 2);

              Vector4f new_coord1(coord1[0], coord1[1], coord1[2], 1.);
              Vector4f new_coord2(coord2[0], coord2[1], coord2[2], 1.);
              Vector4f new_coord3(coord3[0], coord3[1], coord3[2], 1.);

              new_coord1 = model.block(0, (t * 4), 4, 4) * new_coord1;
              new_coord2 = model.block(0, (t * 4), 4, 4) * new_coord2;
              new_coord3 = model.block(0, (t * 4), 4, 4) * new_coord3;

              coord1 << new_coord1[0], new_coord1[1], new_coord1[2];
              coord2 << new_coord2[0], new_coord2[1], new_coord2[2];
              coord3 << new_coord3[0], new_coord3[1], new_coord3[2];
              vector<float> solutions = solver(coord1, coord2, coord3, ray_direction, ray_origin);
              float u = solutions[0];
              float v = solutions[1];
              float t = solutions[2];
              if(does_intersect(t, u, v)){
                selectedPress = true;
                selected_index = t_idx * 4;
                cout << "UNIT CUBE Intersect!" << endl;
                break;
              }
            }
            start += 36;
            break;
          case Bunny:
            for(int s = start; s < start + 3000; s+=3){
              Vector3f coord1 = V.col(s);
              Vector3f coord2 = V.col(s + 1);
              Vector3f coord3 = V.col(s + 2);

              vector<float> solutions = solver(coord1, coord2, coord3, ray_direction, ray_origin);
              float u = solutions[0];
              float v = solutions[1];
              float t = solutions[2];
              if(does_intersect(t, u, v)){
                selectedPress = true;
                selected_index = t_idx * 4;
                cout << "BUNNY Intersect!" << endl;
                break;
              }
            }
            start += 3000;
            break;
          case Bumpy:
            for(int s = start; s < start + 3000; s+=3){
              Vector3f coord1 = V.col(s);
              Vector3f coord2 = V.col(s + 1);
              Vector3f coord3 = V.col(s + 2);

              vector<float> solutions = solver(coord1, coord2, coord3, ray_direction, ray_origin);
              float u = solutions[0];
              float v = solutions[1];
              float t = solutions[2];
              if(does_intersect(t, u, v)){
                selectedPress = true;
                selected_index = t_idx * 4;
                cout << "BUMPY CUBE Intersect!" << endl;
                break;
              }
            }
            start += 3000;
            break;
        }
        if(selected) break;
      } // ObjectType for loop

    }else if(action == GLFW_RELEASE && selectedPress){
      selected = true;
      selectedPress = false;
    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // Update the position of the first vertex if the keys 1,2, or 3 are pressed
    if(action == GLFW_RELEASE){
      switch (key)
      {
        // ADDING OBJECTS
        case  GLFW_KEY_1:
            cout << "Adding unit cube to the origin" << endl;
            numObjects++;
            addUnitCube();
            break;
        case GLFW_KEY_2:
            cout << "Adding bumpy cube to the origin" << endl;
            numObjects++;
            addBumpy();
            break;
        case  GLFW_KEY_3:
            cout << "Adding bunny to the origin" << endl;
            numObjects++;
            addBunny();
            break;
        // RENDERING SETTINGS
        case  GLFW_KEY_4:
            cout << "Wireframe" << endl;
            break;
        case  GLFW_KEY_5:
            cout << "Flat Shading" << endl;
            break;
        case  GLFW_KEY_6:
            cout << "Phong Shading" << endl;
            break;
        // CAMERA TRANSLATION
        case  GLFW_KEY_7:
            cout << "Moving camera LEFT" << endl;
            break;
        case  GLFW_KEY_8:
            cout << "Moving camera RIGHT" << endl;
            break;
        case  GLFW_KEY_9:
            cout << "Moving camera UP" << endl;
            break;
        case  GLFW_KEY_0:
            cout << "Moving camera DOWN" << endl;
            break;
        case GLFW_KEY_MINUS:
          cout << "Moving camera IN" << endl;
          break;
        case GLFW_KEY_EQUAL:
          cout << "Moving camera OUT" << endl;
          break;
        // OBJECT ROTATION
        case  GLFW_KEY_F:
            cout << "Rotating 10 degrees along Z-axis " << endl;
            break;
        case  GLFW_KEY_G:
            cout << "Rotating -10 degrees along Z-axis" << endl;
            break;
        case  GLFW_KEY_H:
            cout << "Rotating 10 degrees along X-axis" << endl;
            break;
        case  GLFW_KEY_J:
            cout << "Rotating -10 degrees along X-axis" << endl;
            break;
        case  GLFW_KEY_K:
            cout << "Rotating 10 degrees along Y-axis" << endl;
            break;
        case  GLFW_KEY_L:
            cout << "Rotating -10 degrees along Y-axis" << endl;
            break;
        // OBJECT TRANSLATION
        case  GLFW_KEY_W:
            cout << "Moving object RIGHT " << endl;
            break;
        case  GLFW_KEY_E:
            cout << "Movign object LEFT" << endl;
            break;
        case  GLFW_KEY_R:
            cout << "Moving object UP" << endl;
            break;
        case  GLFW_KEY_T:
            cout << "Moving object DOWN" << endl;
            break;
        case  GLFW_KEY_Y:
            cout << "Moving object IN" << endl;
            break;
        case  GLFW_KEY_U:
            cout << "Moving object OUT" << endl;
            break;
        // OBJECT SCALING
        case  GLFW_KEY_S:
            cout << "Scaling UP" << endl;
            break;
        case  GLFW_KEY_D:
            cout << "Scaling DOWN" << endl;
            break;
        // PROJECTION
        case  GLFW_KEY_O:
            cout << "Orthographic Projection" << endl;
            ortho = true;
            break;
        case  GLFW_KEY_P:
            cout << "Perspective Projection" << endl;
            ortho = false;
            break;
        default:
            break;
      }
    }
}

int main(void)
{
    GLFWwindow* window;

    // Initialize the library
    if (!glfwInit())
        return -1;

    // Activate supersampling
    glfwWindowHint(GLFW_SAMPLES, 8);

    // Ensure that we get at least a 3.2 context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);

    // On apple we have to load a core profile with forward compatibility
  #ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  #endif

    // Create a windowed mode window and its OpenGL context
    window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    #ifndef __APPLE__
      glewExperimental = true;
      GLenum err = glewInit();
      if(GLEW_OK != err)
      {
        /* Problem: glewInit failed, something is seriously wrong. */
       fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
      }
      glGetError(); // pull and savely ignonre unhandled errors like GL_INVALID_ENUM
      fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
    #endif

    int major, minor, rev;
    major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
    minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
    rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
    printf("OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
    printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
    printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

    VertexArrayObject VAO;
    VAO.init();
    VAO.bind();

    // cout << "INIT" << endl;
    // Initialize everything needed
    initialize(window);

    //------------------------------------------
    // OFF DATA
    //------------------------------------------
    bunny = read_off_data("../data/bunny.off", true);
    bumpy = read_off_data("../data/bumpy_cube.off", false);
    // V = bunny/bumpy.first - holds 3D coordinates
    // F = bunny/bumpy.second - holds indices of triangles
    // indices from F matrix into 3D coordinates from V

    // Initialize the OpenGL Program
    // A program controls the OpenGL pipeline and it must contains
    // at least a vertex shader and a fragment shader to be valid
    Program program;
    const GLchar* vertex_shader =
            "#version 150 core\n"
                    "in vec3 position;" //vertex position
                    "in vec3 color;" //vertex color
                    "in vec3 vertex_normal;"
                    "in vec3 face_normal;"
                    // MVP
                    // "uniform boolean flat;"
                    "uniform mat4 model;"
                    "uniform mat4 view;"
                    "uniform mat4 projection;"

                    "out vec3 Normal;"
                    "out vec3 FragPos;"
                    "out vec3 objectColor;"
                    "void main()"
                    "{"
                    "    gl_Position = projection * view * model * vec4(position, 1.0);"
                    "    FragPos = vec3(model * vec4(position, 1.0f));"
                    // "    if(flat){"
                    "       Normal = mat3(transpose(inverse(model))) * face_normal;"
                    // "    }else{"
                    // "       Normal = mat3(transpose(inverse(model))) * vertex_normal;"
                    // "    }"
                    "    objectColor = color;"
                    "}";
    const GLchar* fragment_shader =
            "#version 150 core\n"
                    "in vec3 Normal;"
                    "in vec3 FragPos;"
                    "in vec3 objectColor;"
                    "out vec4 outColor;"
                    "uniform vec3 lightPos;"
                    "uniform vec3 viewPos;"
                    // "uniform bool flat;"
                    "void main()"
                    "{"
                    "    vec3 lightColor = vec3(1.0, 1.0, 1.0);"
                        // Ambient
                  "      float ambientStrength = 0.01f;"
                  "      vec3 ambient = ambientStrength * lightColor;"

                        // Diffuse
                  "      vec3 norm = normalize(Normal);"
                  "      vec3 lightDir = normalize(lightPos - FragPos);"
                  "      float diff = max(dot(norm, lightDir), 0.0);"
                  "      vec3 diffuse = diff * lightColor;"
                  "      vec3 result = (ambient + diffuse) * objectColor;"

                        // Specular
                  // "      float specularStrength = 0.5f;"
                  // "      vec3 viewDir = normalize(viewPos - FragPos);"
                  // "      vec3 reflectDir = reflect(-lightDir, norm);  "
                  // "      float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);"
                  // "      vec3 specular = specularStrength * spec * lightColor;  "
                  // "      vec3 result = (ambient + diffuse + specular) * objectColor;"
                  "      outColor = vec4(result, 1.0);"
                    "}";

    // Compile the two shaders and upload the binary to the GPU
    // Note that we have to explicitly specify that the output "slot" called outColor
    // is the one that we want in the fragment buffer (and thus on screen)
    program.init(vertex_shader,fragment_shader,"outColor");
    program.bind();

    // The vertex shader wants the position of the vertices as an input.
    // The following line connects the VBO we defined above with the position "slot"
    // in the vertex shader
    program.bindVertexAttribArray("position",VBO);
    program.bindVertexAttribArray("color",VBO_C);
    program.bindVertexAttribArray("vertex_normal",VBO_N_V);
    program.bindVertexAttribArray("face_normal",VBO_N_F);

    glUniform3f(program.uniform("lightPos"), 1.0, 1.0, 2.0);
    glUniform3f(program.uniform("viewPos"), eye[0], eye[1], eye[2]);
    // glUniform1i(program.uniform("flat"), GL_TRUE);

    // Save the current time --- it will be used to dynamically change the triangle color
    auto t_start = std::chrono::high_resolution_clock::now();

    // Register the keyboard callback
    glfwSetKeyCallback(window, key_callback);

    // Register the mouse click callback
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    // Register the mouse movement callback
    glfwSetCursorPosCallback(window, cursor_pos_callback);

    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // Loop until the user closes the window
    while (!glfwWindowShouldClose(window))
    {
      // Bind your program
      program.bind();
      // Clear the framebuffer
      glEnable(GL_DEPTH_TEST);
      glDepthFunc(GL_LESS);
      glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

      glUniformMatrix4fv(program.uniform("view"), 1, GL_FALSE, view.data());
      glUniformMatrix4fv(program.uniform("projection"), 1, GL_FALSE, projection.data());
      if(numObjects > 0){
        int start = 0;
        for(int i = 0; i < types.size(); i++){
          ObjectType t = types[i];

          switch(t){
            case Unit:{
              glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, model.block(0, (i * 4), 4, 4).data());
              // glDrawArrays(GL_TRIANGLES, start, 36);
              // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
              // glEnable(GL_POLYGON_OFFSET_LINE);
              // glPolygonOffset(-2.0f, -2.0f);
              // glLineWidth(1.0f);
              glDrawArrays(GL_TRIANGLES, start, 36);

              MatrixXf holder = MatrixXf::Zero(3,36);
              for(int i = start; i < start + 36; i++){
                holder.col(i) = C.col(i);
                C.col(i) << 0.0, 0.0, 0.0;
              }
              VBO_C.update(C);

              glDrawArrays(GL_LINE_LOOP, start, 36);

              for(int i = start; i < start + 36; i++){
                C.col(i) = holder.col(i);
              }
              VBO_C.update(C);
              start += 36;
              break;
            }
            case Bunny:{
              glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, model.block(0, (i * 4), 4, 4).data());
              glDrawArrays(GL_TRIANGLES, start, 3000);

              MatrixXf holder = MatrixXf::Zero(3,3000);
              for(int i = start; i < start + 3000; i++){
                holder.col(i) = C.col(i);
                C.col(i) << 0.0, 0.0, 0.0;
              }
              VBO_C.update(C);

              for(int i = start; i < start + 3000; i+=3){
                glDrawArrays(GL_LINE_LOOP, i, 3);
              }

              for(int i = start; i < start + 3000; i++){
                C.col(i) = holder.col(i);
              }
              VBO_C.update(C);
              start += 3000;
              break;
            }
            case Bumpy:{
              glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, model.block(0, (i * 4), 4, 4).data());

              glDrawArrays(GL_TRIANGLES, start, 3000);

              MatrixXf holder = MatrixXf::Zero(3,3000);
              for(int i = start; i < start + 3000; i++){
                holder.col(i) = C.col(i);
                C.col(i) << 0.0, 0.0, 0.0;
              }
              VBO_C.update(C);

              for(int i = start; i < start + 3000; i+=3){
                glDrawArrays(GL_LINE_LOOP, i, 3);
              }

              for(int i = start; i < start + 3000; i++){
                C.col(i) = holder.col(i);
              }
              VBO_C.update(C);
              start += 3000;
              break;
            }
          }
        }
      }

      // Swap front and back buffers
      glfwSwapBuffers(window);

      // Poll for and process events
      glfwPollEvents();

    }
    // Deallocate opengl memory
    program.free();

    // Deallocate glfw internals
    glfwTerminate();
    return 0;
}
