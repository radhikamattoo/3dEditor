
// OpenGL Helpers to reduce the clutter
#include "Helpers.h"

// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>

// Linear Algebra Library
#include <Eigen/Core>

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
VertexBufferObject VBO;

// Orthographic or perspective projection?
bool ortho = true;
// Number of meshes existing in the scene
int numMeshes = 1;
// A cube has 6 faces, 2 triangles per face gives 12 triangles
Eigen::MatrixXf Meshes(3,36);
Eigen::Matrix4f viewport(4,4);
Eigen::Matrix4f orthographic(4,4);
Eigen::Matrix4f perspective(4,4);
Eigen::Matrix4f projection(4,4);
Eigen::Matrix4f view(4,4);
Eigen::Matrix4f model(4,4);
Eigen::Matrix4f MVP(4,4);

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

void drawUnitCube()
{
  // Our vertices. Three consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
  // A cube has 6 faces with 2 triangles each, so this makes 6*2=12 triangles, and 12*3 vertices
  Meshes <<
  // -0.5, -0.5,  0.5,
  // -0.5, -0.5, -0.5,
  // -0.5,  0.5,  0.5, // end T1
  //
  // -0.5,  0.5, -0.5,
  // -0.5, -0.5, -0.5,
  // -0.5,  0.5,  0.5, //end T2
  //
  // -0.5, -0.5, -0.5,
  // -0.5,  0.5, -0.5,
  //  0.5,  0.5, -0.5, //end T3
  //
  //  -0.5, -0.5, -0.5,
  //   0.5, -0.5 -0.5,
  //   0.5, 0.5, -0.5, //end T4
  //
  //   0.5, 0.5, -0.5,
  //   0.5, -0.5, -0.5,
  //   0.5, -0.5, 0.5, //end T5
  //
  //   0.5, 0.5, -0.5,
  //   0.5, 0.5, 0.5,
  //   0.5, -0.5, 0.5, //end T6
  //
  //   0.5, 0.5, 0.5,
  //   9.5, -0.5, 0.5,
  //   -0.5, 0.5, 0.5, //end T7
  //
  //   0.5, -0.5, 0.5,
  //   -0.5, -0.5, 0.5,
  //   -0.5, 0.5, 0.5, //end T8
  //
  //   -0.5, -0.5, 0.5,
  //   0.5, -0.5, 0.5,
  //   0.5, -0.5, -0.5, //end T9
  //
  //   -0.5, -0.5, 0.5,
  //   0.5, -0.5, -0.5,
  //   -0.5, -0.5, -0.5, //end T10
  //
  //   -0.5, 0.5, 0.5,
  //   0.5, 0.5, 0.5,
  //   0.5, 0.5, -0.5,  //end T11
  //
  //   -0.5, 0.5, 0.5,
  //   -0.5, 0.5, -0.5,
  //   0.5, 0.5, -0.5; //end T12
    -0.5f,-0.5f,-0.5f,
		-0.5f,-0.5f, 0.5f,
		-0.5f, 0.5f, 0.5f,
		 0.5f, 0.5f,-0.5f,
		-0.5f,-0.5f,-0.5f,
		-0.5f, 0.5f,-0.5f,
		 0.5f,-0.5f, 0.5f,
		-0.5f,-0.5f,-0.5f,
		 0.5f,-0.5f,-0.5f,
		 0.5f, 0.5f,-0.5f,
		 0.5f,-0.5f,-0.5f,
		-0.5f,-0.5f,-0.5f,
		-0.5f,-0.5f,-0.5f,
		-0.5f, 0.5f, 0.5f,
		-0.5f, 0.5f,-0.5f,
		 0.5f,-0.5f, 0.5f,
		-0.5f,-0.5f, 0.5f,
		-0.5f,-0.5f,-0.5f,
		-0.5f, 0.5f, 0.5f,
		-0.5f,-0.5f, 0.5f,
		 0.5f,-0.5f, 0.5f,
		 0.5f, 0.5f, 0.5f,
		 0.5f,-0.5f,-0.5f,
		 0.5f, 0.5f,-0.5f,
		 0.5f,-0.5f,-0.5f,
		 0.5f, 0.5f, 0.5f,
		 0.5f,-0.5f, 0.5f,
		 0.5f, 0.5f, 0.5f,
		 0.5f, 0.5f,-0.5f,
		-0.5f, 0.5f,-0.5f,
		 0.5f, 0.5f, 0.5f,
		-0.5f, 0.5f,-0.5f,
		-0.5f, 0.5f, 0.5f,
		 0.5f, 0.5f, 0.5f,
		-0.5f, 0.5f, 0.5f,
		 0.5f,-0.5f, 0.5f;
  VBO.update(Meshes);
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
    double xworld = ((xpos/double(width))*2)-1;
    double yworld = (((height-1-ypos)/double(height))*2)-1; // NOTE: y axis is flipped in glfw

    // Update the position of the first vertex if the left button is pressed
    // if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
        // V.col(0) << xworld, yworld;

    // Upload the change to the GPU
    // VBO.update(V);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // Update the position of the first vertex if the keys 1,2, or 3 are pressed
    switch (key)
    {
        case  GLFW_KEY_1:
            cout << "Adding unit cube to the origin" << endl;
            break;
        case GLFW_KEY_2:
            cout << "Adding bumpy cube to the origin" << endl;
            break;
        case  GLFW_KEY_3:
            cout << "Adding bunny to the origin" << endl;
            break;
        default:
            break;
    }

    // Upload the change to the GPU
    // VBO.update(V);
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

    // Initialize the VAO
    // A Vertex Array Object (or VAO) is an object that describes how the vertex
    // attributes are stored in a Vertex Buffer Object (or VBO). This means that
    // the VAO is not the actual object storing the vertex data,
    // but the descriptor of the vertex data.
    VertexArrayObject VAO;
    VAO.init();
    VAO.bind();

    // Initialize the VBO with the vertices data
    // A VBO is a data container that lives in the GPU memory
    VBO.init();

    drawUnitCube();

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

    Vector4f near(l, b, n, 1.); //-0.1
    Vector4f far(r, t, f, 1.);

    cout << "Near orthographic map: \n"<< orthographic * near << endl;
    cout << "Far orthographic map: \n"<<  orthographic * far << endl;
    cout << "Near perspective map: \n" << perspective * near << endl;
    cout << "Far perspective map: \n" << perspective * far << endl;

    // projection = orthographic;
    projection =  perspective;
    // cout << "Orthographic: \n" << orthographic << endl;
    // cout << "Perspective: \n" << perspective << endl;
    // cout << "Projection: \n"<< projection << endl;
    // cout << "Determinant: \n"<< projection.determinant() << endl;

    //------------------------------------------
    // VIEW/CAMERA MATRIX
    //------------------------------------------
    Vector3f e(0.0, 0.0, 2.0); //camera position/ eye position
    Vector3f g(0.0, 0.0, 0.0); //target point, where we want to look
    Vector3f t(0.0, 0.5, 0.0); //up vector

    Vector3f w = (e - g).normalized();
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

    // cout <<"Determinant: " <<view.determinant() << endl;
    //
    // cout << "First: " << look << endl;
    // cout << "Second: " << at << endl;
    // cout << "View: " << view << endl;

    //------------------------------------------
    // MODEL MATRIX
    //------------------------------------------
    float degree = (PI/180) * 45;
    model <<
    cos(degree),  0., sin(degree), 0,
    0.,           1.,           0, 0,
    -sin(degree), 0, cos(degree), 0,
    0,          0,              0, 1;

    //------------------------------------------
    // MVP MATRIX
    //------------------------------------------
    // MVP = projection * view * model;
    // MVP = view * model;
    MVP = projection * view * model;
    // TODO: Fix artifacts/problems with unit cube - vertices?

    // Initialize the OpenGL Program
    // A program controls the OpenGL pipeline and it must contains
    // at least a vertex shader and a fragment shader to be valid
    Program program;
    const GLchar* vertex_shader =
            "#version 150 core\n"
                    "in vec3 position;"
                    "uniform mat4 MVP;"
                    "void main()"
                    "{"
                    "    gl_Position = MVP * vec4(position, 0.5);"
                    "}";
    const GLchar* fragment_shader =
            "#version 150 core\n"
                    "out vec4 outColor;"
                    "uniform vec3 triangleColor;"
                    "void main()"
                    "{"
                    "    outColor = vec4(triangleColor, 0.5);"
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

    // Save the current time --- it will be used to dynamically change the triangle color
    auto t_start = std::chrono::high_resolution_clock::now();

    // Register the keyboard callback
    glfwSetKeyCallback(window, key_callback);

    // Register the mouse callback
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    // Loop until the user closes the window
    while (!glfwWindowShouldClose(window))
    {
        // Bind your VAO (not necessary if you have only one)
        VAO.bind();

        // Bind your program
        program.bind();

        // Clear the framebuffer
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        auto t_now = std::chrono::high_resolution_clock::now();
        float time = std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start).count();
        glUniform3f(program.uniform("triangleColor"), (float)(sin(time * 4.0f) + 0.5f) / 2.0f, 0.0f, 0.0f);
        // Set MVP matrix uniform
        glUniformMatrix4fv(program.uniform("MVP"), 1, GL_FALSE, MVP.data());
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // Draw triangles

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }

    // Deallocate opengl memory
    program.free();
    VAO.free();
    VBO.free();

    // Deallocate glfw internals
    glfwTerminate();
    return 0;
}
