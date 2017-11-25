
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

// VertexBufferObject wrapper
vector<unsigned int> EBOS;
vector<unsigned int> VBOS;
vector<VertexArrayObject> VAOS;
// unsigned int indices[];

// Orthographic or perspective projection?
bool ortho = false;

// Number of meshes existing in the scene
int numObjects = 0;

// Keeping track of mouse position
double currentX, currentY, previousX, previousY;

//----------------------------------
// VERTEX/TRANSFORMATION/INDEX DATA
//----------------------------------
Eigen::MatrixXf colors(3, 8); // dynamically resized per object
Eigen::Matrix4f orthographic(4,4);
Eigen::Matrix4f perspective(4,4);
Eigen::Matrix4f projection(4,4);
Eigen::Matrix4f view(4,4); // control camera position
Eigen::Matrix4f model(4,4); // dynamically resized per object
Eigen::Matrix4f MVP(4,4); // dynamically resized per object

//----------------------------------
// OFF DATA
//----------------------------------
pair<MatrixXd, MatrixXd> bunny;
pair<MatrixXd, MatrixXd> bumpy;

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
        V(v,j) = (line_data[j]*3);
      }else{
        V(v,j) = (line_data[j]/10);
      }
      if(!enlarge){ //cube
        V(v,0) += 0.1;
        V(v,1) += 0.1;
      }else{ //bunny
        V(v,0) -= 0.15;
        V(v,1) -= 0.05;
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
void initializeMVP(GLFWwindow* window)
{
  // Get the size of the window
  int width, height;
  glfwGetWindowSize(window, &width, &height);
  float aspect = width/height;

  //------------------------------------------
  // PROJECTION MATRIX
  //------------------------------------------
  t = tan(theta/2) * abs(n);
  b = -t;

  r = aspect * t;
  l = -r;

  // Apply projection matrix to corner points
  orthographic <<
  2/(r - l), 0., 0., -((r+l)/(r-l)),
  0., 2/(t - b), 0., -((t+b)/(t-b)),
  0., 0., 2/(abs(n)-abs(f)), -(n+f)/(abs(n)-abs(f)),
  0., 0., 0.,   1.;

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
  Vector3f e(0.0, 0.0, 2.0); //camera position/ eye position
  Vector3f g(0.0, 0.0, 0.0); //target point, where we want to look
  Vector3f t(0.0, 0.5, 0.0); //up vector

  Vector3f w = (e- g).normalized();
  Vector3f u = (t.cross(w).normalized());
  Vector3f v = w.cross(u);

  // cout << "W:" << w << endl;
  // cout << "U:" << u << endl;
  // cout << "V:" << v << endl;

  Matrix4f look;
  look <<
  u[0], u[1], u[2], 0.,
  v[0], v[1], v[2], 0.,
  w[0], w[1], w[2], 0.,
  0.,   0.,    0.,  0.5;

  Matrix4f at;
  at <<
  0.5, 0.0, 0.0, -e[0],
  0.0, 0.5, 0.0, -e[1],
  0.0, 0.0, 0.5, -e[2],
  0.0, 0.0, 0.0, 0.5;
  view = look * at;

  //------------------------------------------
  // MODEL MATRIX
  //------------------------------------------
  // float degree = (PI/180) * -60;
  // model <<
  // cos(degree),  0., sin(degree), 0,
  // 0.,           1.,           0, 0,
  // -sin(degree), 0, cos(degree), 0,
  // 0,          0,              0, 1;

  model <<
  1., 0., 0., 0.,
  0., 1., 0., 0.,
  0., 0., 1., 0.,
  0., 0., 0., 1.;

  //------------------------------------------
  // MVP MATRIX
  //------------------------------------------
  MVP = projection * view * model;

}
void addUnitCube()
{
  float vertices[] = {
    // front
    -0.5, -0.5,  0.5,
     0.5, -0.5,  0.5,
     0.5,  0.5,  0.5,
    -0.5,  0.5,  0.5,
    // back
    -0.5, -0.5, -0.5,
     0.5, -0.5, -0.5,
     0.5,  0.5, -0.5,
    -0.5,  0.5, -0.5,
  };
  if(ortho){
    for(int i = 0; i < sizeof(vertices)/sizeof(vertices[0]); i++){
      vertices[i] = vertices[i]/70;
    }
  }
  unsigned int indices[] = {  // note that we start from 0!
      // front
      0, 1, 2,
      2, 3, 0,
      // top
      1, 5, 6,
      6, 2, 1,
      // back
      7, 6, 5,
      5, 4, 7,
      // bottom
      4, 0, 3,
      3, 7, 4,
      // left
      4, 5, 1,
      1, 0, 4,
      // right
      3, 2, 6,
      6, 7, 3,
  };
  VertexArrayObject VAO;
  VAO.init();


  // Initialize the VBO with the vertices data
  // A VBO is a data container that lives in the GPU memory
  unsigned int EBO;
  glGenBuffers(1, &EBO);

  unsigned int VBO;
  glGenBuffers(1, &VBO);
  VAO.bind();

  // 2. copy our vertices array in a vertex buffer for OpenGL to use
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  // 3. copy our index array in a element buffer for OpenGL to use
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
  // 4. then set the vertex attributes pointers
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  glBindBuffer(GL_ARRAY_BUFFER, 0); // Note that this is allowed, the call to glVertexAttribPointer registered VBO as the currently bound vertex buffer object so afterwards we can safely unbind

  glBindVertexArray(0); // Unbind VAO (it's always a good thing to unbind any buffer/array to prevent strange bugs), remember: do NOT unbind the EBO, keep it bound to this VAO

  numObjects++;

  VAOS.push_back(VAO);
  EBOS.push_back(EBO);
  VBOS.push_back(VBO);

}
void addBunny()
{
  // Create data matrices from OFF files
  MatrixXd V_bunny = bunny.first;
  MatrixXd F_bunny = bunny.second;
}
void addBumpy()
{
  // Create data matrices from OFF files
  MatrixXd V_bumpy = bumpy.first;
  MatrixXd F_bumpy = bumpy.second;
}
void colorCube()
{
  // colors <<
  // 0.583f,  0.771f,  0.014f,
  // 0.609f,  0.115f,  0.436f,
  // 0.327f,  0.483f,  0.844f,
  // 0.822f,  0.569f,  0.201f,
  // 0.435f,  0.602f,  0.223f,
  // 0.310f,  0.747f,  0.185f,
  // 0.597f,  0.770f,  0.761f,
  // 0.559f,  0.436f,  0.730f;
  // 0.359f,  0.583f,  0.152f,
  // 0.483f,  0.596f,  0.789f,
  // 0.559f,  0.861f,  0.639f,
  // 0.195f,  0.548f,  0.859f,
  // 0.014f,  0.184f,  0.576f,
  // 0.771f,  0.328f,  0.970f,
  // 0.406f,  0.615f,  0.116f,
  // 0.676f,  0.977f,  0.133f,
  // 0.971f,  0.572f,  0.833f,
  // 0.140f,  0.616f,  0.489f,
  // 0.997f,  0.513f,  0.064f,
  // 0.945f,  0.719f,  0.592f,
  // 0.543f,  0.021f,  0.978f,
  // 0.279f,  0.317f,  0.505f,
  // 0.167f,  0.620f,  0.077f,
  // 0.347f,  0.857f,  0.137f,
  // 0.055f,  0.953f,  0.042f,
  // 0.714f,  0.505f,  0.345f,
  // 0.783f,  0.290f,  0.734f,
  // 0.722f,  0.645f,  0.174f,
  // 0.302f,  0.455f,  0.848f,
  // 0.225f,  0.587f,  0.040f,
  // 0.517f,  0.713f,  0.338f,
  // 0.053f,  0.959f,  0.120f,
  // 0.393f,  0.621f,  0.362f,
  // 0.673f,  0.211f,  0.457f,
  // 0.820f,  0.883f,  0.371f,
  // 0.982f,  0.099f,  0.879f;
  // VBO_C.update(colors);

}
void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos)
{
  // Get the size of the window
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // Convert screen position to world coordinates
  Eigen::Vector4f p_screen(xpos,height-1-ypos,0,1);
  Eigen::Vector4f p_canonical((p_screen[0]/width)*2-1,(p_screen[1]/height)*2-1,0,1);
  Eigen::Vector4f p_world = view.inverse()*p_canonical;

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
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
  // Get the position of the mouse in the window
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Get the size of the window
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // Convert screen position to world coordinates
    Eigen::Vector4f p_screen(xpos,height-1-ypos,0,1);
    Eigen::Vector4f p_canonical((p_screen[0]/width)*2-1,(p_screen[1]/height)*2-1,0,1);
    Eigen::Vector4f p_world = view.inverse()*p_canonical;

    double xworld = p_world[0];
    double yworld = p_world[1];

    if(action == GLFW_RELEASE){
      // Check if an object was clicked on and select it

    }else if(action == GLFW_PRESS){
      // If an object is selected, translate it

    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // Update the position of the first vertex if the keys 1,2, or 3 are pressed
    if(action == GLFW_RELEASE){
      switch (key)
      {
        case  GLFW_KEY_1:
            cout << "Adding unit cube to the origin" << endl;
            addUnitCube();
            numObjects++;
            break;
        case GLFW_KEY_2:
            cout << "Adding bumpy cube to the origin" << endl;
            addBumpy();
            numObjects++;
            break;
        case  GLFW_KEY_3:
            cout << "Adding bunny to the origin" << endl;
            addBunny();
            numObjects++;
            break;
        case  GLFW_KEY_7:
            cout << "Wireframe" << endl;
            break;
        case  GLFW_KEY_8:
            cout << "Flat Shading" << endl;
            break;
        case  GLFW_KEY_9:
            cout << "Phong Shading" << endl;
            break;
        case  GLFW_KEY_H:
            cout << "Rotating 10 degrees clockwise" << endl;
            break;
        case  GLFW_KEY_J:
            cout << "Rotating 10 degrees counter-clockwise" << endl;
            break;
        case  GLFW_KEY_K:
            cout << "Scaling UP by 25 percent" << endl;
            break;
        case  GLFW_KEY_L:
            cout << "Scaling DOWN by 25 percent" << endl;
            break;
        case  GLFW_KEY_O:
            cout << "Orthographic Projection" << endl;
            ortho = true;
            break;
        case  GLFW_KEY_P:
            cout << "Perspective Projection" << endl;
            ortho = false;
            break;
        case GLFW_KEY_RIGHT:
          cout << "Moving camera right" << endl;
          break;
        case GLFW_KEY_LEFT:
          cout << "Moving camera left" << endl;
          break;
        case GLFW_KEY_UP:
          cout << "Moving camera up" << endl;
          break;
        case GLFW_KEY_DOWN:
          cout << "Moving camera down" << endl;
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

    // Initialize the MVP uniform
    initializeMVP(window);

    //------------------------------------------
    // OFF DATA
    //------------------------------------------
    bunny = read_off_data("../data/bunny.off", true);
    bumpy = read_off_data("../data/bumpy_cube.off", false);
    // V = bunny/bumpy.first - holds 3D coordinates
    // F = bunny/bumpy.second - holds indices of triangles
    // indices from F matrix into 3D coordinates from V
    cout << "Bunny matrix size: " << bunny.second.size() << endl;
    cout << "Bumpy cube matrix size: " << bumpy.second.size() << endl;

    // Initialize the OpenGL Program
    // A program controls the OpenGL pipeline and it must contains
    // at least a vertex shader and a fragment shader to be valid
    Program program;
    const GLchar* vertex_shader =
            "#version 150 core\n"
                    "in vec3 position;"
                    "uniform mat4 MVP;"
                    // "in vec3 color;"
                    // "out vec3 f_color;"
                    "void main()"
                    "{"
                    "    gl_Position = MVP * vec4(position, 1.0);"
                    // "    f_color = color;"
                    "}";
    const GLchar* fragment_shader =
            "#version 150 core\n"
                    // "in vec3 f_color;"
                    "out vec4 outColor;"
                    "uniform vec3 triangleColor;"
                    "void main()"
                    "{"
                    "    outColor = vec4(triangleColor, 1.0);"
                    "}";

    // Compile the two shaders and upload the binary to the GPU
    // Note that we have to explicitly specify that the output "slot" called outColor
    // is the one that we want in the fragment buffer (and thus on screen)
    program.init(vertex_shader,fragment_shader,"outColor");
    program.bind();

    // The vertex shader wants the position of the vertices as an input.
    // The following line connects the VBO we defined above with the position "slot"
    // in the vertex shader
    cout << "DRAWING" << endl;
    // program.bindVertexAttribArray("position",VBO);
    // program.bindVertexAttribArray("color",VBO_C);

    // Save the current time --- it will be used to dynamically change the triangle color
    auto t_start = std::chrono::high_resolution_clock::now();

    // Register the keyboard callback
    glfwSetKeyCallback(window, key_callback);

    // Register the mouse callback
    glfwSetMouseButtonCallback(window, mouse_button_callback);
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

      for(int i = 0; i < VAOS.size(); i++){
          VertexArrayObject VAO = VAOS[i];
          VAO.bind();
          // Set MVP matrix uniform
          glUniformMatrix4fv(program.uniform("MVP"), 1, GL_FALSE, MVP.data());
          // Draw triangles
          glDrawElements(GL_TRIANGLES, 12, GL_UNSIGNED_INT, 0);
          glBindVertexArray(0);
      }
      // Swap front and back buffers
      glfwSwapBuffers(window);

      // Poll for and process events
      glfwPollEvents();

    }
    // Deallocate opengl memory
    program.free();
    for(int i = 0; i < VAOS.size(); i++){
      VertexArrayObject VAO = VAOS[i];
      unsigned int EBO = EBOS[i];
      unsigned int  VBO = VBOS[i];
      glDeleteBuffers(1, &VBO);
      glDeleteBuffers(1, &EBO);
      VAO.free();
    }

    // Deallocate glfw internals
    glfwTerminate();
    return 0;
}
