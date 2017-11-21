
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
VertexBufferObject VBO_I;
VertexBufferObject VBO_C;
vector<float> indices;
// Orthographic or perspective projection?
bool ortho = true;
// Number of meshes existing in the scene
int nummeshes = 1;
// A cube has 6 faces, 2 triangles per face gives 12 triangles
Eigen::MatrixXf meshes(3,36);
Eigen::MatrixXf colors(3, 36);
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
float n = -0.5;
float f = -100.;
// right and left
float r;
float l;
// top and bottom
float t;
float b;

void drawUnitCube()
{
  // TODO: USE VBO INDEX TO PREVENT REPEAT VERTICES
  // // front
  // meshes <<
  // -0.5, -0.5,  0.5,
  //  0.5, -0.5,  0.5,
  //  0.5,  0.5,  0.5,
  // -0.5,  0.5,  0.5,
  // // back
  // -0.5, -0.5, -0.5,
  //  0.5, -0.5, -0.5,
  //  0.5,  0.5, -0.5,
  // -0.5,  0.5, -0.5,
  // indices = {
  //    // front
  //    0, 1, 2,
  //    2, 3, 0,
  //    // top
  //    1, 5, 6,
  //    6, 2, 1,
  //    // back
  //    7, 6, 5,
  //    5, 4, 7,
  //    // bottom
  //    4, 0, 3,
  //    3, 7, 4,
  //    // left
  //    4, 5, 1,
  //    1, 0, 4,
  //    // right
  //    3, 2, 6,
  //    6, 7, 3
  //  };
  meshes <<
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
  VBO.update(meshes);

}
void colorCube()
{
  colors <<
  0.583f,  0.771f,  0.014f,
  0.609f,  0.115f,  0.436f,
  0.327f,  0.483f,  0.844f,
  0.822f,  0.569f,  0.201f,
  0.435f,  0.602f,  0.223f,
  0.310f,  0.747f,  0.185f,
  0.597f,  0.770f,  0.761f,
  0.559f,  0.436f,  0.730f,
  0.359f,  0.583f,  0.152f,
  0.483f,  0.596f,  0.789f,
  0.559f,  0.861f,  0.639f,
  0.195f,  0.548f,  0.859f,
  0.014f,  0.184f,  0.576f,
  0.771f,  0.328f,  0.970f,
  0.406f,  0.615f,  0.116f,
  0.676f,  0.977f,  0.133f,
  0.971f,  0.572f,  0.833f,
  0.140f,  0.616f,  0.489f,
  0.997f,  0.513f,  0.064f,
  0.945f,  0.719f,  0.592f,
  0.543f,  0.021f,  0.978f,
  0.279f,  0.317f,  0.505f,
  0.167f,  0.620f,  0.077f,
  0.347f,  0.857f,  0.137f,
  0.055f,  0.953f,  0.042f,
  0.714f,  0.505f,  0.345f,
  0.783f,  0.290f,  0.734f,
  0.722f,  0.645f,  0.174f,
  0.302f,  0.455f,  0.848f,
  0.225f,  0.587f,  0.040f,
  0.517f,  0.713f,  0.338f,
  0.053f,  0.959f,  0.120f,
  0.393f,  0.621f,  0.362f,
  0.673f,  0.211f,  0.457f,
  0.820f,  0.883f,  0.371f,
  0.982f,  0.099f,  0.879f;
  VBO_C.update(colors);

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

    //------------------------------------------
    // MODEL MATRIX
    //------------------------------------------
    float degree = (PI/180) * 40;
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
    // QUESTIONS:
    // TODO: Difference between orthographic and perspective projection
    // with unit cube is ridiculous ?
    // TODO: What should the default view be - orthographic or perspective?


    // TODO: VBO INDEXING!!!!!! Vertices missing


    //------------------------------------------
    // COLOR MATRIX
    //------------------------------------------
    VBO_C.init();
    colorCube();


    // Initialize the OpenGL Program
    // A program controls the OpenGL pipeline and it must contains
    // at least a vertex shader and a fragment shader to be valid
    Program program;
    const GLchar* vertex_shader =
            "#version 150 core\n"
                    "in vec3 position;"
                    "uniform mat4 MVP;"
                    "in vec3 color;"
                    "out vec3 f_color;"
                    "void main()"
                    "{"
                    "    gl_Position = MVP * vec4(position, 0.5);"
                    "    f_color = color;"
                    "}";
    const GLchar* fragment_shader =
            "#version 150 core\n"
                    "in vec3 f_color;"
                    "out vec4 outColor;"
                    "uniform vec3 triangleColor;"
                    "void main()"
                    "{"
                    "    outColor = vec4(f_color, 1.0);"
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
        glDepthFunc(GL_LESS);
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
