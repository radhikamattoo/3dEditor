
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
MatrixXf C_holder(3,3000);
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

// Is an object selected?
int selected_index = -1;
int selected_vertex_index = -1;
int previousIdx = -1;
ObjectType select_type;
ObjectType previousType;

// Amount to divide/multiply vertices by in orthographic projection
int ORTHO_FACTOR = 70;


Vector4f initBunnyOrtho;
Vector4f initBunnyPer;
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
pair<MatrixXf, MatrixXf> bunny;
pair<MatrixXf, MatrixXf> bumpy;
MatrixXf bunny_vertex_holder(3, 3000);
MatrixXf bumpy_vertex_holder(3, 3000);
MatrixXf unit_vertex_holder(3, 36);

// HOLDS NORMAL DATA TO ADD TO N_faces/N_vertices WHEN ADDING OBJECTS
MatrixXf unit_faces = MatrixXf::Zero(3,36);
MatrixXf unit_vertices = MatrixXf::Zero(3,36);

MatrixXf bunny_faces = MatrixXf::Zero(3,3000);
MatrixXf bunny_vertices = MatrixXf::Zero(3,3000);

MatrixXf bumpy_faces = MatrixXf::Zero(3,3000);
MatrixXf bumpy_vertices = MatrixXf::Zero(3,3000);

//----------------------------------
// VIEW MATRIX PARAMETERS
//----------------------------------
float focal_length = 1.5;
Vector3f eye(0.0, 0.0, focal_length); //camera position/ eye position  //e
Vector3f look_at(0.0, 0.0, 0.0); //target point, where we want to look //g
Vector3f up_vec(0.0, 1.0, 0.0); //up vector //t
// Light position
Vector3f lightPos;

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
pair<MatrixXf, MatrixXf> read_off_data(string filename, bool enlarge)
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
  MatrixXf V = MatrixXf::Zero(vertices, 3);
  MatrixXf F = MatrixXf::Zero(faces, 3);
  vector<float> line_data;

  // Fill V & F matrices from file
  for(int v = 0; v < vertices; v++){
    getline(stream, line);
    line_data = split_line(line, false);

    for(int j = 0; j < 3; j++){
      if(enlarge){
        V(v,j) = (line_data[j]*6);
      }else{
        V(v,j) = (line_data[j]/9);
      }
      if(!enlarge){ //cube
        // V(v,0) += 0.1;
        // V(v,1) += 0.1;
      }else{ //bunny
        // V(v,0) += 0.05;
        // V(v,1) -= 0.35;
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
  pair<MatrixXf, MatrixXf> matrices(V, F);
  return matrices;

}

bool isInVector(vector<int> summed, int idx){
  for(int i = 0; i < summed.size(); i++){
    if(idx == summed[i]) return true;
  }
  return false;
}

void changeRendering(RenderType type){
  int idx = selected_index/4;
  if(selected_index > -1){
    switch(type){
      case Fill:{
        renders[idx] = type;
      }
      case Wireframe:{
        renders[idx] = type;
        break;
      }
      case Flat:{
        renders[idx] = type;
        break;
      }
      case Phong:{
        renders[idx] = type;
        break;
      }
    }
  }
}
// Iterates through the N buffer and adds in the normals
// ONLY CALLED ONCE PER OBJECT TYPE AT INITIALIZATION
void addNormals(ObjectType type)
{
  switch(type){
    case Unit:{
      // go through faces
      for(int i = 0; i < 36; i+=3)
      {
        Vector3f edge1 = unit_vertex_holder.col(i + 1) - unit_vertex_holder.col(i);
        Vector3f edge2 = unit_vertex_holder.col(i + 2) - unit_vertex_holder.col(i);
        Vector3f normal = (edge1.cross(edge2)).normalized();

        unit_faces.col(i) << normal;
        unit_faces.col(i + 1) << normal;
        unit_faces.col(i + 2) << normal;
        unit_vertices.col(i) += normal;
        unit_vertices.col(i + 1) += normal;
        unit_vertices.col(i + 2) += normal;
      }
      for(int i = 0; i <  36; i++){
        vector<int> summed;
        Vector3f current = unit_vertex_holder.col(i);
        Vector3f sum =  unit_vertices.col(i);
        summed.push_back(i);
        for(int j = 0; j < 36; j++){
          if(isInVector(summed, j)) continue;
          if(i == j) continue;
          Vector3f other = unit_vertex_holder.col(j);
          if(other[0] == current[0] && other[1] == current[1] && other[2] == current[2]){
            sum += unit_vertices.col(j);
            summed.push_back(j);
          }
        }
        sum = sum.normalized();
        for(int k = 0; k < summed.size(); k++){
          int idx = summed[k];
          unit_vertices.col(idx) = sum;
        }
      }
      break;
    }

    case Bunny:{

      for(int i = 0; i < 3000; i+=3)
      {
        Vector3f edge1 = bunny_vertex_holder.col(i + 1) - bunny_vertex_holder.col(i);
        Vector3f edge2 = bunny_vertex_holder.col(i + 2) - bunny_vertex_holder.col(i);
        Vector3f normal = (edge1.cross(edge2)).normalized();
        bunny_faces.col(i) << normal;
        bunny_faces.col(i + 1) << normal;
        bunny_faces.col(i + 2) << normal;
        bunny_vertices.col(i) += normal;
        bunny_vertices.col(i + 1) += normal;
        bunny_vertices.col(i + 2) += normal;
      }
      for(int i = 0; i < 3000; i++){
        vector<int> summed;
        Vector3f current = bunny_vertex_holder.col(i);
        Vector3f sum =  bunny_vertices.col(i);
        summed.push_back(i);
        for(int j = 0; j < 3000; j++){
          if(isInVector(summed, j)) continue;
          if(i == j) continue;
          Vector3f other = bunny_vertex_holder.col(j);
          if(other[0] == current[0] && other[1] == current[1] && other[2] == current[2]){
            sum += bunny_vertices.col(j);
            summed.push_back(j);
          }
        }
        sum = sum.normalized();
        for(int k = 0; k < summed.size(); k++){
          int idx = summed[k];
          bunny_vertices.col(idx) = sum;
        }
      }
      break;
    }

    case Bumpy:{
      // go through faces
      for(int i = 0; i < 3000; i+=3)
      {
        Vector3f edge1 = bumpy_vertex_holder.col(i + 1) - bumpy_vertex_holder.col(i);
        Vector3f edge2 = bumpy_vertex_holder.col(i + 2) - bumpy_vertex_holder.col(i);
        Vector3f normal = (edge1.cross(edge2)).normalized();
        bumpy_faces.col(i) << normal;
        bumpy_faces.col(i + 1) << normal;
        bumpy_faces.col(i + 2) << normal;
        bumpy_vertices.col(i) += normal;
        bumpy_vertices.col(i + 1) += normal;
        bumpy_vertices.col(i + 2) += normal;
      }

      for(int i = 0; i < 3000; i++){
        vector<int> summed;
        Vector3f current = bumpy_vertex_holder.col(i);
        Vector3f sum =  bumpy_vertices.col(i);
        summed.push_back(i);
        for(int j = 0; j <  3000; j++){
          if(isInVector(summed, j)) continue;
          if(i == j) continue;
          Vector3f other = bumpy_vertex_holder.col(j);
          if(other[0] == current[0] && other[1] == current[1] && other[2] == current[2]){
            sum += bumpy_vertices.col(j);
            summed.push_back(j);
          }
        }
        sum = sum.normalized();
        for(int k = 0; k < summed.size(); k++){
          int idx = summed[k];
          bumpy_vertices.col(idx) = sum;
        }
      }
      break;
    }
  }
}

void initialize(GLFWwindow* window)
{
  VBO.init();
  VBO_C.init();
  VBO_N_V.init();
  VBO_N_F.init();
  if(ortho){
     lightPos << 1.0, 0.0, -1.0;
  }else{
    lightPos << 1.0, 0.0, 1.0;
  }

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

   // BOTTOM
   unit_vertex_holder.col(0) << 0.5f, -0.5f, 0.5f;
   unit_vertex_holder.col( 1) <<   0.5, -0.5, -0.5;
   unit_vertex_holder.col( 2) <<   -0.5,-0.5, -0.5;
   unit_vertex_holder.col( 3) << -0.5,-0.5, -0.5;
   unit_vertex_holder.col( 4) <<  -0.5, -0.5, 0.5;
   unit_vertex_holder.col( 5) <<  0.5, -0.5, 0.5;

   // BACK
   unit_vertex_holder.col( 6) << 0.5,  0.5, -0.5;
   unit_vertex_holder.col( 7) <<  -0.5, -0.5, -0.5;
   unit_vertex_holder.col( 8) << 0.5, -0.5, -0.5;
   unit_vertex_holder.col( 9) << 0.5,  0.5, -0.5;
   unit_vertex_holder.col( 10) << -0.5, -0.5, -0.5;
   unit_vertex_holder.col( 11) << -0.5,  0.5, -0.5;

   // LEFT
   unit_vertex_holder.col( 12) << -0.5,  0.5, -0.5;
   unit_vertex_holder.col( 13) << -0.5,  0.5,  0.5;
   unit_vertex_holder.col( 14) << -0.5, -0.5, -0.5;
   unit_vertex_holder.col( 15) << -0.5, -0.5, -0.5;
   unit_vertex_holder.col( 16) <<  -0.5,  0.5,  0.5;
   unit_vertex_holder.col( 17) <<  -0.5, -0.5,  0.5;

   // RIGHT
   unit_vertex_holder.col( 18) << 0.5, -0.5,  0.5;
   unit_vertex_holder.col( 19) << 0.5,  0.5,  0.5;
   unit_vertex_holder.col( 20) << 0.5, -0.5, -0.5;
   unit_vertex_holder.col( 21) << 0.5, -0.5, -0.5;
   unit_vertex_holder.col( 22) << 0.5,  0.5,  0.5;
   unit_vertex_holder.col( 23) << 0.5,  0.5, -0.5;

   // TOP
   unit_vertex_holder.col( 24) << 0.5,  0.5,  0.5;
   unit_vertex_holder.col( 25) << 0.5,  0.5, -0.5;
   unit_vertex_holder.col( 26) << -0.5, 0.5, 0.5;
   unit_vertex_holder.col( 27) << -0.5, 0.5, -0.5;
   unit_vertex_holder.col( 28) << 0.5,  0.5, -0.5;
   unit_vertex_holder.col( 29) << -0.5, 0.5, 0.5;

   // FRONT
   unit_vertex_holder.col( 30) <<  -0.5, -0.5,  0.5;
   unit_vertex_holder.col( 31) <<   0.5, -0.5,  0.5;
   unit_vertex_holder.col( 32) <<   0.5,  0.5,  0.5;
   unit_vertex_holder.col( 33) <<  -0.5,  0.5,  0.5;
   unit_vertex_holder.col( 34) <<   -0.5, -0.5,  0.5;
   unit_vertex_holder.col( 35) <<  0.5,  0.5,  0.5;

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
  0., 0.,  (abs(f) + abs(n))/(abs(n) - abs(f)), (2 * abs(f) * abs(n))/(abs(n) - abs(f)),
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

  if (ortho){
    model <<
        1./70,     0.,     0.,     0.,
        0.,     1./70,     0.,     0.,
        0.,     0.,     1./70,     0.,
        0.,     0.,     0.,     1.;
  }else {
    model <<
        1.,     0.,     0.,     0.,
        0.,     1.,     0.,     0.,
        0.,     0.,     1.,     0.,
        0.,     0.,     0.,     1.;
  }

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

  // Create data matrices from OFF files
  MatrixXf V_bunny = bunny.first;
  MatrixXf F_bunny = bunny.second;
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
      bunny_vertex_holder.col((i*3) + j) << vertices[0], vertices[1], vertices[2];
    }
  }

  MatrixXf V_bumpy = bumpy.first;
  MatrixXf F_bumpy = bumpy.second;
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
      bumpy_vertex_holder.col((i*3) + j) << vertices[0], vertices[1], vertices[2];
    }
  }

  ObjectType t = Unit;
  addNormals(t);
  t = Bunny;
  addNormals(t);
  t = Bumpy;
  addNormals(t);

  N_vertices = unit_vertices;
  N_faces = unit_faces;
  VBO_N_F.update(N_faces);
  VBO_N_V.update(N_vertices);
  cout << "Finished initialization" << endl;

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

    // model.block(0, 4 * (numObjects-1), 4, 4) = translation.block(0, 4 * (numObjects-1), 4, 4) * rotation.block(0, 4 * (numObjects-1), 4, 4) * scaling.block(0, 4 * (numObjects-1), 4, 4);

    MVP.block(0, 4 * (numObjects-1), 4, 4) = projection * view *   model.block(0, 4 * (numObjects-1), 4, 4);
  }
  cout << "Adding model matrix" << endl;
  if (ortho){
    model.block(0, 4 * (numObjects-1), 4, 4) <<
        1.,     0.,     0.,     0.,
        0.,     1.,     0.,     0.,
        0.,     0.,     1.,     0.,
        0.,     0.,     0.,     1.;
  }else {
    model.block(0, 4 * (numObjects-1), 4, 4) <<
        1.,     0.,     0.,     0.,
        0.,     1.,     0.,     0.,
        0.,     0.,     1.,     0.,
        0.,     0.,     0.,     1.;
  }
  cout << "Filling V matrix" << endl;
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
  int unit_idx = 0;
  for(int i = start; i < start + 36; i++){
    N_vertices.col(i) << unit_vertices.col(unit_idx);
    N_faces.col(i) << unit_faces.col(unit_idx);
    unit_idx++;
    C.col(i) << 1.0, 0.0, 0.0;
  }
  if(ortho){
    V.block(0, start, 3, 36) /= ORTHO_FACTOR;
  }
  VBO_N_F.update(N_faces);
  VBO_N_V.update(N_vertices);
  VBO_C.update(C);
  VBO.update(V);
  cout << "UPDATED VBOS" << endl;



}
void addBunny()
{
  ObjectType t = Bunny;
  types.push_back(t);
  RenderType r = Fill;
  renders.push_back(r);

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

    MVP.block(0, 4 * (numObjects-1), 4, 4) = projection * view *   model.block(0, 4 * (numObjects-1), 4, 4);
  }else{
    V.conservativeResize(3, 3000);
    N_faces.conservativeResize(3, 3000);
    N_vertices.conservativeResize(3, 3000);
    C.conservativeResize(3, 3000);
  }
  initBunnyOrtho << 0.0033, -0.0, 0.00017, 1.;
  initBunnyPer <<  0.109091,-0.717025,0.0119573,1.;
  if(ortho){
    model.block(0, 4*(numObjects-1), 4, 4)<<
        1.,    0.,            0.,                0.0015,
        0.,          1.0,     0.,                -0.009,
        0.,           0.,            1.,          0.00017,
        0.,           0.,            0.,                1.;

  }else{
    model.block(0, 4*(numObjects-1), 4, 4)<<
    1,    0.,            0.,                                    0.109091,
      0.,         1,       0.,                                    -0.6,
      0.,         0.,            1,                               0.0119573,
      0.,         0.,            0.,                1.;
  }
  int bunny_idx = 0;
  for(int i = start; i < start + 3000; i++){
    N_vertices.col(i) << bunny_vertices.col(bunny_idx);
    N_faces.col(i) << bunny_faces.col(bunny_idx);
    C.col(i) << 0.0, 0.0, 1.0;
    V.col(i) << bunny_vertex_holder.col(bunny_idx);
    bunny_idx++;
  }

  if(ortho){
    V.block(0, start, 3, 3000) /= ORTHO_FACTOR;
  }

  VBO_N_F.update(N_faces);
  VBO_N_V.update(N_vertices);
  VBO_C.update(C);
  VBO.update(V);


}
void addBumpy()
{
  ObjectType t = Bumpy;
  types.push_back(t);
  RenderType r = Fill;
  renders.push_back(r);

  // Hold onto start index
  int start = 0;
  if(numObjects > 1){
    start = V.cols();
    V.conservativeResize(3, V.cols() + 3000);
    N_faces.conservativeResize(3, N_faces.cols() + 3000);
    N_vertices.conservativeResize(3, N_vertices.cols() + 3000);
    C.conservativeResize(3, C.cols() + 3000);
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


    MVP.block(0, 4 * (numObjects-1), 4, 4) = projection * view *   model.block(0, 4 * (numObjects-1), 4, 4);
  }else{
    V.conservativeResize(3, 3000);
    N_faces.conservativeResize(3, 3000);
    N_vertices.conservativeResize(3, 3000);
    C.conservativeResize(3, 3000);
  }

  if(ortho){
    model.block(0, 4 * (numObjects-1), 4, 4) <<
        1.,   0.,         0.,         0.,
        0.,         1.,   0.,         0.,
        0.,         0.,         1.,   0.,
        0.,         0.,         0.,         1.;
  }else{
    model.block(0, 4 * (numObjects-1), 4, 4) <<
        1.0,   0.,         0.,         0.,
        0.,         1.0,   0.,         0.,
        0.,         0.,         1.0,   0.,
        0.,         0.,         0.,         1.;
  }

  int bumpy_idx = 0;
  for(int i = start; i < start + 3000; i++){
    N_vertices.col(i) << bumpy_vertices.col(bumpy_idx);
    N_faces.col(i) << bumpy_faces.col(bumpy_idx);
    C.col(i) << 0.0, 1.0, 0.0;
    V.col(i) << bumpy_vertex_holder.col(bumpy_idx);
    bumpy_idx++;
  }
  if(ortho){
    V.block(0, start, 3, 3000) /= ORTHO_FACTOR;

  }
  VBO_N_F.update(N_faces);
  VBO_N_V.update(N_vertices);
  VBO_C.update(C);
  VBO.update(V);
}
void rotateTriangle(int axis, float direction){
  // translate back to origin
  if(selected_vertex_index > -1){
    direction = (PI/180) * direction;
    MatrixXf rotation(4,4);
    Vector4f translation;
    if(select_type == Bunny){
      if(ortho){
        translation = initBunnyOrtho;
      }else{
        translation = initBunnyPer;
      }
    }else{
      translation << 0.0, 0.0, 0.0, 1.0;
    }
    if(axis == 0){ //Z
      rotation <<
      cos(direction),      sin(direction),  0.,  0.,
      -sin(direction),     cos(direction),  0.,  0.,
      0.,                     0.,                 1.,  0.,
      0.,                     0.,                 0.,  1.;
    }else if(axis == 1){ //X
      rotation <<
      1.,    0.,                  0.,                 0.,
      0.,    cos(direction),   sin(direction),  0.,
      0.,    -sin(direction),  cos(direction),  0.,
      0.,    0.,                  0.,                 1.;

    }else if(axis == 2){ //Y
      rotation <<
        cos(direction),  0.,  -sin(direction),   0.,
        0.,                 1.,  0.,                   0.,
        sin(direction),  0.,  cos(direction),    0.,
        0.,                 0.,  0.,                   1.;
    }
    model.block(0, selected_index, 4, 4).col(3) -= translation;
    model.block(0, selected_index, 4, 4) = model.block(0, selected_index, 4, 4) * rotation;
    model.block(0, selected_index, 4, 4).col(3) += translation;
  }
}

void scaleTriangle(bool up){
  float factor = .20;
  Vector4f translated;
  MatrixXf scaling(4,4);

  if(selected_vertex_index > -1){

    if(up){
      factor += 1;
      if(select_type == Bunny){
        if(ortho){
          translated = initBunnyOrtho;
        }else{
          translated = initBunnyPer;
        }
      }else{
        translated << 0.0, 0.0, 0.0, 1.0;
      }
      scaling <<
      factor,    0.,       0.,                        0.,
      0.,        factor,   0.,                        0.,
      0.,        0.,       factor,                    0.,
      0.,        0.,       0.,                        1.;

      model.block(0, selected_index, 4, 4).col(3) -= translated;
      model.block(0, selected_index, 4, 4) = model.block(0, selected_index, 4, 4) * scaling;
      model.block(0, selected_index, 4, 4).col(3) += translated;
    }else{
      factor = 1-factor;
      if(select_type == Bunny){
        if(ortho){
          translated = initBunnyOrtho;
        }else{
          translated = initBunnyPer;
        }
      }else{
        translated << 0.0, 0.0, 0.0, 1.0;
      }
      scaling <<
      factor,    0.,       0.,                        0.,
      0.,        factor,   0.,                        0.,
      0.,        0.,       factor,                    0.,
      0.,        0.,       0.,                        1.;

      model.block(0, selected_index, 4, 4).col(3) -= translated;
      model.block(0, selected_index, 4, 4) = model.block(0, selected_index, 4, 4) * scaling;
      model.block(0, selected_index, 4, 4).col(3) += translated;
    }

  }

}

// Translates triangle based on mouse movement
void translateTriangle(int direction)
{
  if(selected_vertex_index > -1){
    float translation_amount = 0.3;
    if(ortho){
      translation_amount /= ORTHO_FACTOR;
    }
    if(direction == 0){ // + x
      model.block(0, selected_index, 4, 4)(0,3) += translation_amount;
    }else if(direction == 1){ // -x
      model.block(0, selected_index, 4, 4)(0,3) -= translation_amount;
    }else if(direction == 2){ //up
      model.block(0, selected_index, 4, 4)(1,3) += translation_amount;
    }else if(direction == 3){ //down
      model.block(0, selected_index, 4, 4)(1,3) -= translation_amount;
    }else if(direction == 4){ //in
      model.block(0, selected_index, 4, 4)(2,3) += translation_amount;
    }else if(direction == 5){ //out
      model.block(0, selected_index, 4, 4)(2,3) -= translation_amount;
    }
  }
}

void changeView(int direction)
{
  float factor = 0.1;

  if(direction == 0){
    eye[0] -= factor;
  }else if(direction == 1){
    eye[0] += factor;
  }else if(direction == 2){
    eye[1] += factor;
  }else if(direction == 3){
    eye[1] -= factor;
  }else if(direction == 4){
    eye[2] += factor;
  }else if(direction == 5){
    eye[2] -= factor;
  }
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

}
void deleteObject()
{

  if(selected_vertex_index > -1){
    switch(select_type){
      case Unit:{
        cout << "Deleting UNIT cube" << endl;
        MatrixXf V_holder(3, (V.cols()-36));
        MatrixXf N_faces_holder(3, (N_faces.cols()-36));
        MatrixXf N_vertices_holder(3, (N_vertices.cols()-36));
        MatrixXf C_holder(3, (C.cols()-36));
        MatrixXf model_holder(4, (model.cols()-4));
        vector<ObjectType> types_holder;
        vector<RenderType> renders_holder;
        int start = selected_vertex_index;
        int end = start + 36;
        int holder_idx = 0;
        for(int i = 0; i < V.cols(); i++){
          if(i >= start && i < end) continue;
          V_holder.col(holder_idx) << V.col(i);
          N_faces_holder.col(holder_idx) << N_faces.col(i);
          N_vertices_holder.col(holder_idx) << N_vertices.col(i);
          C_holder.col(holder_idx) << C.col(i);
          holder_idx++;
        }
        start = selected_index;
        end = selected_index + 4;
        holder_idx = 0;
        for(int i = 0; i < model.cols(); i++){
          if(i >= start && i < end) continue;
          model_holder.col(holder_idx) << model.col(i);
          holder_idx++;
        }
        start = selected_index/4;
        for(int i = 0; i < types.size(); i++){
          if(i == start) continue;
          types_holder.push_back(types[i]);
          renders_holder.push_back(renders[i]);
        }
        types.clear();
        renders.clear();
        for(int i = 0; i < types_holder.size(); i++){
          types.push_back(types_holder[i]);
          renders.push_back(renders_holder[i]);
        }
        V = V_holder;
        N_faces = N_faces_holder;
        N_vertices = N_vertices_holder;
        C = C_holder;
        model = model_holder;
        VBO.update(V);
        VBO_N_F.update(N_faces);
        VBO_N_V.update(N_vertices);
        VBO_C.update(C);
        numObjects--;
        break;
      }
      case Bunny:{
        cout << "Deleting BUNNY" << endl;
        MatrixXf V_holder(3, (V.cols()-3000));
        MatrixXf N_faces_holder(3, (N_faces.cols()-3000));
        MatrixXf N_vertices_holder(3, (N_vertices.cols()-3000));
        MatrixXf C_holder(3, (C.cols()-3000));
        MatrixXf model_holder(4, (model.cols()-4));
        vector<ObjectType> types_holder;
        vector<RenderType> renders_holder;
        int start = selected_vertex_index;
        int end = start + 3000;
        int holder_idx = 0;
        for(int i = 0; i < V.cols(); i++){
          if(i >= start && i < end) continue;
          V_holder.col(holder_idx) << V.col(i);
          N_faces_holder.col(holder_idx) << N_faces.col(i);
          N_vertices_holder.col(holder_idx) << N_vertices.col(i);
          C_holder.col(holder_idx) << C.col(i);
          holder_idx++;
        }
        start = selected_index;
        end = selected_index + 4;
        holder_idx = 0;
        for(int i = 0; i < model.cols(); i++){
          if(i >= start && i < end) continue;
          model_holder.col(holder_idx) << model.col(i);
          holder_idx++;
        }
        start = selected_index/4;
        for(int i = 0; i < types.size(); i++){
          if(i == start) continue;
          types_holder.push_back(types[i]);
          renders_holder.push_back(renders[i]);
        }
        types.clear();
        renders.clear();
        for(int i = 0; i < types_holder.size(); i++){
          types.push_back(types_holder[i]);
          renders.push_back(renders_holder[i]);
        }
        V = V_holder;
        N_faces = N_faces_holder;
        N_vertices = N_vertices_holder;
        C = C_holder;
        model = model_holder;
        VBO.update(V);
        VBO_N_F.update(N_faces);
        VBO_N_V.update(N_vertices);
        VBO_C.update(C);
        numObjects--;
        break;
      }
      case Bumpy:{
        cout << "Deleting BUMPY" << endl;
        MatrixXf V_holder(3, (V.cols()-3000));
        MatrixXf N_faces_holder(3, (N_faces.cols()-3000));
        MatrixXf N_vertices_holder(3, (N_vertices.cols()-3000));
        MatrixXf C_holder(3, (C.cols()-3000));
        MatrixXf model_holder(4, (model.cols()-4));
        vector<ObjectType> types_holder;
        vector<RenderType> renders_holder;
        int start = selected_vertex_index;
        int end = start + 3000;
        int holder_idx = 0;
        for(int i = 0; i < V.cols(); i++){
          if(i >= start && i < end) continue;
          V_holder.col(holder_idx) << V.col(i);
          N_faces_holder.col(holder_idx) << N_faces.col(i);
          N_vertices_holder.col(holder_idx) << N_vertices.col(i);
          C_holder.col(holder_idx) << C.col(i);
          holder_idx++;
        }
        start = selected_index;
        end = selected_index + 4;
        holder_idx = 0;
        for(int i = 0; i < model.cols(); i++){
          if(i >= start && i < end) continue;
          model_holder.col(holder_idx) << model.col(i);
          holder_idx++;
        }
        start = selected_index/4;
        for(int i = 0; i < types.size(); i++){
          if(i == start) continue;
          types_holder.push_back(types[i]);
          renders_holder.push_back(renders[i]);
        }
        types.clear();
        renders.clear();
        for(int i = 0; i < types_holder.size(); i++){
          types.push_back(types_holder[i]);
          renders.push_back(renders_holder[i]);
        }
        V = V_holder;
        N_faces = N_faces_holder;
        N_vertices = N_vertices_holder;
        C = C_holder;
        model = model_holder;
        VBO.update(V);
        VBO_N_F.update(N_faces);
        VBO_N_V.update(N_vertices);
        VBO_C.update(C);
        numObjects--;
        break;
      }
    }
    selected_index = -1;
    selected_vertex_index = -1;
    previousIdx = -1;
    if(numObjects == 0){
      //------------------------------------------
      // VERTEX DATA
      //------------------------------------------
      V.conservativeResize(3,36);
      C.conservativeResize(3,36);
      N_vertices.conservativeResize(3,36);
      N_faces.conservativeResize(3,36);
      model.conservativeResize(4,4);
      if (ortho){
        model <<
            1./70,     0.,     0.,     0.,
            0.,     1./70,     0.,     0.,
            0.,     0.,     1./70,     0.,
            0.,     0.,     0.,     1.;
      }else {
        model <<
            1.,     0.,     0.,     0.,
            0.,     1.,     0.,     0.,
            0.,     0.,     1.,     0.,
            0.,     0.,     0.,     1.;
      }
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

       N_vertices = unit_vertices;
       N_faces = unit_faces;
       VBO_N_F.update(N_faces);
       VBO_N_V.update(N_vertices);


    }
  }

}

pair<int, int> wasSelected(double xworld, double yworld)
{
  selected_index = -1;
  selected_vertex_index = -1;
  int start_idx = 0; //start index of object in V
  int model_idx = 0; //start index of object's model matrix
  double triangle_depth = -1000000;
  // iterate through all objects in the scene
  for(int type = 0; type < types.size(); type++)
  {
    ObjectType t = types[type];
    switch(t){
      case Unit:{
        MatrixXf model_matrix = model.block(0, model_idx, 4, 4);
        for(int v = start_idx; v < start_idx + 36; v+=3){
          Vector4f a_obj = model_matrix * Vector4f(V(0, v), V(1, v), V(2, v), 1.);
          Vector4f b_obj = model_matrix * Vector4f(V(0, v+1), V(1, v+1), V(2, v+1), 1.);
          Vector4f c_obj = model_matrix * Vector4f(V(0, v+2), V(1, v+2), V(2, v+2), 1.);
          Vector3f a(a_obj(0), a_obj(1), a_obj(2));
          Vector3f b(b_obj(0), b_obj(1), b_obj(2));
          Vector3f c(c_obj(0), c_obj(1), c_obj(2));
          double ax = a(0);
          double ay = a(1);
          double az = a(2);
          double bx = b(0);
          double by = b(1);
          double bz = b(2);
          double cx = c(0);
          double cy = c(1);
          double cz = c(2);
          double total_area = abs(((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)) / 2.0);
          double alpha_area = abs(((bx - xworld) * (cy - yworld) - (cx - xworld) * (by - yworld)) / 2.0);
          double beta_area = abs(((xworld - ax) * (cy - ay) - (cx - ax) * (yworld - ay)) / 2.0);
          double gamma_area = abs(((bx - ax) * (yworld - ay) - (xworld - ax) * (by - ay)) / 2.0);
          // Solve system for alpha, beta, and gamma
          double alpha = alpha_area / total_area;
          double beta = beta_area / total_area;
          double gamma = gamma_area / total_area;
          if (abs(alpha + beta + gamma - 1) < 0.003 && alpha >= 0 && beta >= 0 && gamma >= 0) {
             double depth = max(az, max(bz, cz));
             if (depth > triangle_depth) {
                 cout << "SELECTED UNIT CUBE" << endl;
                 selected_index = type *4; //index in types of selected object
                 select_type = types[type];
                 selected_vertex_index = start_idx;
                 // deletion_end_index =  end_index;
                 triangle_depth = depth;
                 // index_of_object = type; //index in V of selected object
             }
          }
        }
        start_idx += 36;
        model_idx += 4;
        break;
      }
      case Bunny:{
        MatrixXf model_matrix = model.block(0, model_idx, 4, 4);
        for(int v = start_idx; v < start_idx + 3000; v+=3){
          Vector4f a_obj = model_matrix * Vector4f(V(0, v), V(1, v), V(2, v), 1.);
          Vector4f b_obj = model_matrix * Vector4f(V(0, v+1), V(1, v+1), V(2, v+1), 1.);
          Vector4f c_obj = model_matrix * Vector4f(V(0, v+2), V(1, v+2), V(2, v+2), 1.);
          Vector3f a(a_obj(0), a_obj(1), a_obj(2));
          Vector3f b(b_obj(0), b_obj(1), b_obj(2));
          Vector3f c(c_obj(0), c_obj(1), c_obj(2));
          double ax = a(0);
          double ay = a(1);
          double az = a(2);
          double bx = b(0);
          double by = b(1);
          double bz = b(2);
          double cx = c(0);
          double cy = c(1);
          double cz = c(2);
          double total_area = abs(((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)) / 2.0);
          double alpha_area = abs(((bx - xworld) * (cy - yworld) - (cx - xworld) * (by - yworld)) / 2.0);
          double beta_area = abs(((xworld - ax) * (cy - ay) - (cx - ax) * (yworld - ay)) / 2.0);
          double gamma_area = abs(((bx - ax) * (yworld - ay) - (xworld - ax) * (by - ay)) / 2.0);
          // Solve system for alpha, beta, and gamma
          double alpha = alpha_area / total_area;
          double beta = beta_area / total_area;
          double gamma = gamma_area / total_area;

          if (abs(alpha + beta + gamma - 1) < 0.003 && alpha >= 0 && beta >= 0 && gamma >= 0) {
             double depth = max(az, max(bz, cz));
             if (depth > triangle_depth) {
                 cout << "SELECTED BUNNY" << endl;
                 selected_index = type*4; //index in types of selected object
                 select_type = types[type];
                 selected_vertex_index = start_idx;
                 // deletion_end_index =  end_index;
                 triangle_depth = depth;
                 // index_of_object = type; //index in V of selected object
             }
          }
        }
        start_idx += 3000;
        model_idx += 4;
        break;
      }
      case Bumpy:{
        MatrixXf model_matrix = model.block(0, model_idx, 4, 4);
        for(int v = start_idx; v < start_idx + 3000; v+=3){
          Vector4f a_obj = model_matrix * Vector4f(V(0, v), V(1, v), V(2, v), 1.);
          Vector4f b_obj = model_matrix * Vector4f(V(0, v+1), V(1, v+1), V(2, v+1), 1.);
          Vector4f c_obj = model_matrix * Vector4f(V(0, v+2), V(1, v+2), V(2, v+2), 1.);
          Vector3f a(a_obj(0), a_obj(1), a_obj(2));
          Vector3f b(b_obj(0), b_obj(1), b_obj(2));
          Vector3f c(c_obj(0), c_obj(1), c_obj(2));
          double ax = a(0);
          double ay = a(1);
          double az = a(2);
          double bx = b(0);
          double by = b(1);
          double bz = b(2);
          double cx = c(0);
          double cy = c(1);
          double cz = c(2);
          double total_area = abs(((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)) / 2.0);
          double alpha_area = abs(((bx - xworld) * (cy - yworld) - (cx - xworld) * (by - yworld)) / 2.0);
          double beta_area = abs(((xworld - ax) * (cy - ay) - (cx - ax) * (yworld - ay)) / 2.0);
          double gamma_area = abs(((bx - ax) * (yworld - ay) - (xworld - ax) * (by - ay)) / 2.0);
          // Solve system for alpha, beta, and gamma
          double alpha = alpha_area / total_area;
          double beta = beta_area / total_area;
          double gamma = gamma_area / total_area;

          if (abs(alpha + beta + gamma - 1) < 0.003 && alpha >= 0 && beta >= 0 && gamma >= 0) {
             double depth = max(az, max(bz, cz));
             if (depth > triangle_depth) {
                 cout << "Selected BUMPY CUBE" << endl;
                 selected_index = type*4; //index in types of selected object
                 select_type = types[type];
                 selected_vertex_index = start_idx;
                 // deletion_end_index =  end_index;
                 triangle_depth = depth;
                 // index_of_object = type; //index in V of selected object
             }
          }
        }
        start_idx += 3000;
        model_idx += 4;
        break;
      }
    }
  } //end of for loop
  return pair<int,int>(selected_vertex_index, selected_index);

}

void window_size_callback(GLFWwindow* window, int width, int height)
{
  cout << "Resizing window" << endl;
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
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
  // Get the position of the mouse in the window
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Get the size of the window
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    Vector4f p_screen(xpos,height-1-ypos,0,1);
    Vector4f p_canonical((p_screen[0]/width)*2-1,(p_screen[1]/height)*2-1,0,1);
    Vector4f p_camera = projection.inverse() * p_canonical;
    Vector4f p_world = view.inverse() * p_camera;

    double x_world, y_world;
    if(action == GLFW_PRESS){
      // Check if an object was clicked on and select it
      if(ortho){
        x_world = p_world[0]/4.17421;
        y_world = p_world[1]/4.17421;
      }else{
        x_world = p_world[0] * 0.5 * 1.91632;
        y_world = p_world[1] * 0.5 * 1.91632;
      }

      // cout << "Previous selected idx: " << selected_vertex_index << endl;
      pair<int, int> result = wasSelected(x_world, y_world);

      selected_vertex_index = result.first;
      selected_index = result.second;
      cout << "Selected vertex is now: \n" << selected_vertex_index << endl;
      // CHANGE SELECTED OBJECT'S COLOR
      if(selected_vertex_index > -1){
        // change the color and update
        cout << "Changing selected object's color" << endl;
        if(previousIdx > -1){
          cout << "Selected nothing, changing previously select object's color back to original" << endl;
          switch(previousType){
            case Unit:{
              cout << "\t Unit" << endl;
              int idx = 0;
              for(int i = previousIdx; i < previousIdx + 36; i++){
                C.col(i) << C_holder.col(idx);
              }
              VBO_C.update(C);
              break;
            }
            case Bunny:{
              cout << "\tBunny" << endl;
              int idx = 0;
              for(int i = previousIdx; i < previousIdx + 3000; i++){
                C.col(i) << C_holder.col(idx);
              }
              VBO_C.update(C);
              break;
            }
            case Bumpy:{
              cout << "\tBumpy" << endl;
              int idx = 0;
              for(int i = previousIdx; i < previousIdx + 3000; i++){
                C.col(i) << C_holder.col(idx);
              }
              VBO_C.update(C);
              break;
            }
          }
        }
        switch(select_type){
          case Unit:{
            int idx = 0;
            for(int i = selected_vertex_index; i < selected_vertex_index + 36; i++){
              C_holder.col(idx) << C.col(i);
              C.col(i) << 1.0, 1.0, 1.0;
            }
            VBO_C.update(C);
            break;
          }
          case Bunny:{
            int idx = 0;
            for(int i = selected_vertex_index; i < selected_vertex_index + 3000; i++){
              C_holder.col(idx) << C.col(i);
              C.col(i) << 1.0, 1.0, 1.0;
            }
            VBO_C.update(C);
            break;
          }
          case Bumpy:{
            int idx = 0;
            for(int i = selected_vertex_index; i < selected_vertex_index + 3000; i++){
              C_holder.col(idx) << C.col(i);
              C.col(i) << 1.0, 1.0, 1.0;
            }
            VBO_C.update(C);
            break;
          }
        }
        previousIdx = selected_vertex_index;
        previousType = select_type;

      }
      else{
        if(previousIdx > -1){
          cout << "Selected nothing, changing previously select object's color back to original" << endl;
          switch(previousType){
            case Unit:{
              cout << "\t Unit" << endl;
              int idx = 0;
              for(int i = previousIdx; i < previousIdx + 36; i++){
                C.col(i) << C_holder.col(idx);
              }
              VBO_C.update(C);
              break;
            }
            case Bunny:{
              cout << "\tBunny" << endl;
              int idx = 0;
              for(int i = previousIdx; i < previousIdx + 3000; i++){
                C.col(i) << C_holder.col(idx);
              }
              VBO_C.update(C);
              break;
            }
            case Bumpy:{
              cout << "\tBumpy" << endl;
              int idx = 0;
              for(int i = previousIdx; i < previousIdx + 3000; i++){
                C.col(i) << C_holder.col(idx);
              }
              VBO_C.update(C);
              break;
            }
          }
          previousIdx = -1;
        }
      }

    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // Update the position of the first vertex if the keys 1,2, or 3 are pressed
    if(action == GLFW_RELEASE){
      switch (key)
      {
        // ADDING OBJECTS
        case  GLFW_KEY_1:{
            cout << "Adding unit cube to the origin" << endl;
            numObjects++;
            addUnitCube();
            break;
        }
        case GLFW_KEY_2:{
            cout << "Adding bumpy cube to the origin" << endl;
            numObjects++;
            addBumpy();
            break;
        }
        case  GLFW_KEY_3:{
            cout << "Adding bunny to the origin" << endl;
            numObjects++;
            addBunny();
            break;
        }
        // RENDERING SETTINGS
        case  GLFW_KEY_4:{
            cout << "Wireframe" << endl;
            RenderType t = Wireframe;
            changeRendering(t);
            break;
        }
        case  GLFW_KEY_5:{
            cout << "Flat Shading" << endl;
            RenderType t = Flat;
            changeRendering(t);
            break;
        }
        case  GLFW_KEY_6:{
            cout << "Phong Shading" << endl;
            RenderType t = Phong;
            changeRendering(t);
            break;
        }
        case  GLFW_KEY_X:{
            cout << "Fill " << endl;
            RenderType t = Fill;
            changeRendering(t);
            break;
        }
        // CAMERA TRANSLATION
        case  GLFW_KEY_7:{
            cout << "Moving camera LEFT" << endl;
            changeView(0);
            break;
        }
        case  GLFW_KEY_8:{
            cout << "Moving camera RIGHT" << endl;
            changeView(1);
            break;
        }
        case  GLFW_KEY_9:{
            cout << "Moving camera UP" << endl;
            changeView(2);
            break;
          }
        case  GLFW_KEY_0:{
            cout << "Moving camera DOWN" << endl;
            changeView(3);
            break;
          }
        case GLFW_KEY_MINUS:{
          cout << "Moving camera IN" << endl;
          changeView(4);
          break;
        }
        case GLFW_KEY_EQUAL:{
          cout << "Moving camera OUT" << endl;
          changeView(5);
          break;
        }
        // OBJECT ROTATION
        case  GLFW_KEY_F:{
            cout << "Rotating 10 degrees along Z-axis " << endl;
            rotateTriangle(0, 10.);
            break;
          }
        case  GLFW_KEY_G:{
            cout << "Rotating -10 degrees along Z-axis" << endl;
            rotateTriangle(0, -10.);
            break;
          }
        case  GLFW_KEY_H:{
            cout << "Rotating 10 degrees along X-axis" << endl;
            rotateTriangle(1, 10.);
            break;
          }
        case  GLFW_KEY_J:{
            cout << "Rotating -10 degrees along X-axis" << endl;
            rotateTriangle(1, -10.);
            break;
          }
        case  GLFW_KEY_K:{
            cout << "Rotating 10 degrees along Y-axis" << endl;
            rotateTriangle(2, 10.);
            break;
          }
        case  GLFW_KEY_L:{
            cout << "Rotating -10 degrees along Y-axis" << endl;
            rotateTriangle(2, -10.);
            break;
          }
        // OBJECT TRANSLATION
        case  GLFW_KEY_W:{
            cout << "Moving object RIGHT " << endl;
            translateTriangle(0);
            break;
          }
        case  GLFW_KEY_E:{
            cout << "Movign object LEFT" << endl;
            translateTriangle(1);
            break;
          }
        case  GLFW_KEY_R:{
            cout << "Moving object UP" << endl;
            translateTriangle(2);
            break;
          }
        case  GLFW_KEY_T:{
            cout << "Moving object DOWN" << endl;
            translateTriangle(3);
            break;
          }
        case  GLFW_KEY_Y:{
            cout << "Moving object IN" << endl;
            translateTriangle(4);
            break;
          }
        case  GLFW_KEY_U:{
            cout << "Moving object OUT" << endl;
            translateTriangle(5);
            break;
          }
        // OBJECT SCALING
        case  GLFW_KEY_S:{
            cout << "Scaling UP" << endl;
            scaleTriangle(true);
            break;
          }
        case  GLFW_KEY_D:{
            cout << "Scaling DOWN" << endl;
            scaleTriangle(false);
            break;
          }
          // DELETION
          case  GLFW_KEY_C:{
            deleteObject();
            break;
        }
        default:{
            break;
          }
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

    //------------------------------------------
    // OFF DATA
    //------------------------------------------
    bunny = read_off_data("../data/bunny.off", true);
    bumpy = read_off_data("../data/bumpy_cube.off", false);

    cout << "Initializing...\n" << endl;
    initialize(window);

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
                    "uniform bool is_flat;"
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
                    "    if(is_flat){"
                    "       Normal = mat3(transpose(inverse(model))) * face_normal;"
                    "    }else{"
                    "       Normal = mat3(transpose(inverse(model))) * vertex_normal;"
                    "    }"
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
                    "uniform bool is_flat;"
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
                  "      if(is_flat){"
                  "         vec3 result = (ambient + diffuse) * objectColor;"
                  "         outColor = vec4(result, 1.0);"
                  "       }else{"
                            // Specular
                  "         float specularStrength = 0.5f;"
                  "         vec3 viewDir = normalize(viewPos - FragPos);"
                  "         vec3 reflectDir = reflect(-lightDir, norm);  "
                  "         float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);"
                  "         vec3 specular = specularStrength * spec * lightColor;  "
                  "         vec3 result = (ambient + diffuse + specular) * objectColor;"
                  "         outColor = vec4(result, 1.0);"
                  "     }"
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

    glUniform3f(program.uniform("lightPos"), lightPos[0] ,lightPos[1], lightPos[2]);
    glUniform3f(program.uniform("viewPos"), eye[0], eye[1], eye[2]);
    glUniform1i(program.uniform("is_flat"), true);

    // Save the current time --- it will be used to dynamically change the triangle color
    auto t_start = std::chrono::high_resolution_clock::now();

    // Register the keyboard callback
    glfwSetKeyCallback(window, key_callback);

    // Register the mouse click callback
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    // Register the window resize callback
    glfwSetWindowSizeCallback(window, window_size_callback);

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
              RenderType type = renders[i];

              switch(type){
                case Fill:{
                  glUniform1i(program.uniform("is_flat"), true);
                  glDrawArrays(GL_TRIANGLES, start, 36);
                  break;
                }
                case Wireframe:{
                  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                  MatrixXf holder = MatrixXf::Zero(3,36);
                  int holder_idx = 0;
                  for(int i = start; i < start + 36; i++){
                    holder.col(holder_idx) = C.col(i);
                    C.col(i) << 0.0, 0.0, 0.0;
                    holder_idx++;
                  }
                  VBO_C.update(C);
                  glDrawArrays(GL_TRIANGLES, start, 36);
                  holder_idx = 0;
                  for(int i = start; i < start + 36; i++){
                    C.col(i) = holder.col(holder_idx);
                    holder_idx++;
                  }
                  VBO_C.update(C);
                  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                  break;
                }
                case Flat:{
                  glUniform1i(program.uniform("is_flat"), true);
                  glDrawArrays(GL_TRIANGLES, start, 36);
                  MatrixXf holder = MatrixXf::Zero(3,36);
                  int idx = 0;
                  for(int i = start; i < start + 36; i++){
                    holder.col(idx) = C.col(i);
                    C.col(i) << 0.0, 0.0, 0.0;
                    idx++;
                  }
                  VBO_C.update(C);

                  glDrawArrays(GL_LINE_LOOP, start, 36);
                  idx = 0;
                  for(int i = start; i < start + 36; i++){
                    C.col(i) = holder.col(idx);
                    idx++;
                  }
                  VBO_C.update(C);
                  break;
                }
                case Phong:{
                  glUniform1i(program.uniform("is_flat"), false);
                  glDrawArrays(GL_TRIANGLES, start, 36);
                  glUniform1i(program.uniform("is_flat"), true);
                  break;
                }
              }

              start += 36;
              break;
            }
            case Bunny:{
              glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, model.block(0, (i * 4), 4, 4).data());
              RenderType type = renders[i];

              switch(type){
                case Fill:{
                  glUniform1i(program.uniform("is_flat"), true);
                  glDrawArrays(GL_TRIANGLES, start, 3000);
                  break;
                }
                case Wireframe:{
                  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                  MatrixXf holder = MatrixXf::Zero(3,3000);
                  int holder_idx = 0;
                  for(int i = start; i < start + 3000; i++){
                    holder.col(holder_idx) = C.col(i);
                    C.col(i) << 0.0, 0.0, 0.0;
                    holder_idx++;
                  }
                  VBO_C.update(C);
                  glDrawArrays(GL_TRIANGLES, start, 3000);
                  holder_idx = 0;
                  for(int i = start; i < start + 3000; i++){
                    C.col(i) = holder.col(holder_idx);
                    holder_idx++;
                  }
                  VBO_C.update(C);
                  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                  break;
                }
                case Flat:{
                  glUniform1i(program.uniform("is_flat"), true);
                  glDrawArrays(GL_TRIANGLES, start, 3000);

                  MatrixXf holder = MatrixXf::Zero(3,3000);
                  int holder_idx = 0;
                  for(int i = start; i < start + 3000; i++){
                    holder.col(holder_idx) = C.col(i);
                    C.col(i) << 0.0, 0.0, 0.0;
                    holder_idx++;
                  }
                  VBO_C.update(C);

                  for(int i = start; i < start + 3000; i+=3){
                    glDrawArrays(GL_LINE_LOOP, i, 3);
                  }
                  holder_idx = 0;
                  for(int i = start; i < start + 3000; i++){
                    C.col(i) = holder.col(holder_idx);
                    holder_idx++;
                  }
                  VBO_C.update(C);
                  break;
                }
                case Phong:{
                  glUniform1i(program.uniform("is_flat"), false);
                  glDrawArrays(GL_TRIANGLES, start, 3000);
                  glUniform1i(program.uniform("is_flat"), true);
                  break;
                }
              }

              start += 3000;
              break;
            }
            case Bumpy:{
              glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, model.block(0, (i * 4), 4, 4).data());
              RenderType type = renders[i];

              switch(type){
                case Fill:{
                  glUniform1i(program.uniform("is_flat"), true);
                  glDrawArrays(GL_TRIANGLES, start, 3000);
                  break;
                }
                case Wireframe:{
                  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                  MatrixXf holder = MatrixXf::Zero(3,3000);
                  int holder_idx = 0;
                  for(int i = start; i < start + 3000; i++){
                    holder.col(holder_idx) = C.col(i);
                    C.col(i) << 0.0, 0.0, 0.0;
                    holder_idx++;
                  }
                  VBO_C.update(C);
                  glDrawArrays(GL_TRIANGLES, start, 3000);
                  holder_idx = 0;
                  for(int i = start; i < start + 3000; i++){
                    C.col(i) = holder.col(holder_idx);
                    holder_idx++;
                  }
                  VBO_C.update(C);
                  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                  break;
                }
                case Flat:{
                  glUniform1i(program.uniform("is_flat"), true);
                  glDrawArrays(GL_TRIANGLES, start, 3000);

                  MatrixXf holder = MatrixXf::Zero(3,3000);
                  int holder_idx = 0;
                  for(int i = start; i < start + 3000; i++){
                    holder.col(holder_idx) = C.col(i);
                    C.col(i) << 0.0, 0.0, 0.0;
                    holder_idx++;
                  }
                  VBO_C.update(C);

                  for(int i = start; i < start + 3000; i+=3){
                    glDrawArrays(GL_LINE_LOOP, i, 3);
                  }
                  holder_idx = 0;
                  for(int i = start; i < start + 3000; i++){
                    C.col(i) = holder.col(holder_idx);
                    holder_idx++;
                  }
                  VBO_C.update(C);
                  break;
                }
                case Phong:{
                  glUniform1i(program.uniform("is_flat"), false);
                  glDrawArrays(GL_TRIANGLES, start, 3000);
                  break;
                }
              }
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
