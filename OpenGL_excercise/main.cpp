#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <cmath>
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>
#include <Windows.h>

struct vertex { double x; double y; };
struct velocity { double x; double y; };
struct triangle { vertex aA; vertex bB; vertex cC; vertex zZ; };
void compensator(struct vertex* ver, vertex ref, double og);

/// <summary>
/// Creates a triangle struct object representing an equilateral triangle from a center point and side length
/// </summary>
/// <param name="center">center point</param>
/// <param name="len">side length</param>
/// <returns>an equilateral triangle struct object</returns>
triangle makeEquiTri(struct vertex* center, double len) {
    triangle myEqui;
    myEqui.zZ = *center;
    myEqui.aA = { center->x - len / 2, center->y - len * sqrt(3) / 6 };
    myEqui.bB = { center->x + len / 2, center->y - len * sqrt(3) / 6 };
    myEqui.cC = { center->x, center->y + len * sqrt(3) / 3 };
    return myEqui;
}

/// <summary>
/// Performs rotation on a point around the origin (at the center of the window)
/// </summary>
/// <param name="phi">rotation angle</param>
/// <param name="ver">pointer to the point to be rotated</param>
void rotation(double phi, struct vertex* ver, vertex* ref) {
    double side = (pow(ver->x - ref->x, 2) + pow(ver->y - ref->y, 2));
    ver->x = ref->x + (ver->x - ref->x) * cos(phi) - (ver->y - ref->y) * sin(phi);
    ver->y = ref->y + (ver->x - ref->x) * sin(phi) + (ver->y - ref->y) * cos(phi);
    compensator(ver, *ref, side);
}

/// <summary>
/// Eliviates inaccuracies due to rounding errors that accumulate after rounds
/// </summary>
/// <param name="ver">the vertex to be adjusted</param>
/// <param name="ref">reference vertex</param>
/// <param name="og">original distance</param>
void compensator(struct vertex* ver, vertex ref, double og) {
    double magnif = (pow(ver->x - ref.x, 2) + pow(ver->y - ref.y, 2)) / og;
    if (magnif < 0.99999 || magnif > 1.00001) {
        ver->x = ref.x + (ver->x - ref.x) / sqrt(magnif);
        ver->y = ref.y + (ver->y - ref.y) / sqrt(magnif);
    }
}

/// <summary>
/// Rotate a whole equilateral triangle struct object 
/// </summary>
/// <param name="tria">the equilateral triangle to be rotated</param>
/// <param name="phi">rotation angle</param>
void triRotate(struct triangle* tria, double phi) {
    vertex* pA = &tria->aA, * pB = &tria->bB, * pC = &tria->cC, * pZ = &tria->zZ;
    rotation(phi, pA, pZ);
    rotation(phi, pB, pZ);
    rotation(phi, pC, pZ);
    //delete pA, pB,pC,pZ;
}

/// <summary>
/// Resize the given triangle with a given ratio
/// </summary>
/// <param name="tria">the triangle to be resized</param>
/// <param name="coeff">resize ratio</param>
void triResize(struct triangle* tria, double coeff) {
    tria->aA.x = tria->zZ.x + (tria->aA.x - tria->zZ.x) * coeff;
    tria->bB.x = tria->zZ.x + (tria->bB.x - tria->zZ.x) * coeff;
    tria->cC.x = tria->zZ.x + (tria->cC.x - tria->zZ.x) * coeff;
    tria->aA.y = tria->zZ.y + (tria->aA.y - tria->zZ.y) * coeff;
    tria->bB.y = tria->zZ.y + (tria->bB.y - tria->zZ.y) * coeff;
    tria->cC.y = tria->zZ.y + (tria->cC.y - tria->zZ.y) * coeff;
}

/// <summary>
/// Move the given triangle away some distance
/// </summary>
/// <param name="tria">the triangle to be moved</param>
/// <param name="velo">move distance</param>
void triTranslate(struct triangle* tria, struct velocity velo) {
    tria->aA.x = tria->aA.x + velo.x;
    tria->bB.x = tria->bB.x + velo.x;
    tria->cC.x = tria->cC.x + velo.x;
    tria->zZ.x = tria->zZ.x + velo.x;
    tria->aA.y = tria->aA.y + velo.y;
    tria->bB.y = tria->bB.y + velo.y;
    tria->cC.y = tria->cC.y + velo.y;
    tria->zZ.y = tria->zZ.y + velo.y;
}

/// <summary>
/// A help function to translate the triangle in special cases when the triangle hits window borders
/// </summary>
/// <param name="tria">the triangle struct object</param>
/// <param name="dist">relocation parameter</param>
/// <param name="yRicht">true if the triangle hits the upper/lower boundaries, false it if hits left/right boundaries</param>
void triReflect(struct triangle* tria, double dist, bool yRicht) {
    if (yRicht) {
        tria->aA.y = tria->aA.y * dist;
        tria->bB.y = tria->bB.y * dist;
        tria->cC.y = tria->cC.y * dist;
        tria->zZ.y = tria->zZ.y * dist;
    }
    else {
        tria->aA.x = tria->aA.x * dist;
        tria->bB.x = tria->bB.x * dist;
        tria->cC.x = tria->cC.x * dist;
        tria->zZ.x = tria->zZ.x * dist;
    }
}

/// <summary>
/// Handles the cases when the triangle collides with window borders
/// </summary>
/// <param name="tria">the triangle object</param>
/// <param name="velo">the triangle's translational velocity</param>
/// <param name="breite">horizontal dimension of the window</param>
/// <param name="hoehe">vertical dimension of the window</param>
/// <param name="omega">the triangle's angular velocity</param>
/// <returns>true if the triangle is in collision with border, false if otherwise</returns>
bool triCollision(struct triangle* tria, velocity* velo, double breite, double hoehe,
    double* omega) {
    vertex* pA = &tria->aA, * pB = &tria->bB, * pC = &tria->cC, * pZ = &tria->zZ;
    if (pA->y <= -hoehe || pA->y >= hoehe) {
        double alpha = atan((pA->y - pZ->y) / (pA->x - pZ->x));
        velo->y = -velo->y;
        while (pA->y <= -hoehe || pA->y >= hoehe) 
            triReflect(tria, 0.99, true);
        *omega = -*omega;
        return true;
    }
    if (pB->y <= -hoehe || pB->y >= hoehe) {
        double alpha = atan((pB->y - pZ->y) / (pB->x - pZ->x));
        velo->y = -velo->y;
        while (pB->y <= -hoehe || pB->y >= hoehe) 
            triReflect(tria, 0.99, true);
        *omega = -*omega; return true;
    }
    if (pC->y <= -hoehe || pC->y >= hoehe) {
        double alpha = atan((pC->y - pZ->y) / (pC->x - pZ->x));
        velo->y = -velo->y;
        while (pC->y <= -hoehe || pC->y >= hoehe) 
            triReflect(tria, 0.99, true);
        *omega = -*omega; return true;
    }
    if (pA->x <= -breite || pA->x >= breite) {
        velo->x = -velo->x;
        while (pA->x <= -breite || pA->x >= breite) 
            triReflect(tria, 0.99, false);
        *omega = -*omega;
        return true;
    }
    if (pB->x <= -breite || pB->x >= breite) {
        velo->x = -velo->x;
        while (pB->x <= -breite || pB->x >= breite) 
            triReflect(tria, 0.99, false);
        *omega = -*omega; return true;
    }
    if (pC->x <= -breite || pC->x >= breite) {
        velo->x = -velo->x;
        while (pC->x <= -breite || pC->x >= breite) 
            triReflect(tria, 0.99, false);
        *omega = -*omega; return true;
    }
    return false;
}

/// <summary>
/// main operational function. An equilateral triangle object is created.
/// A running loop will update the triangle's position and orientation 
/// and handle inputs to increase or decrease triangle's size as well as rotational velocity.
/// </summary>
/// <param name=""></param>
/// <returns>0</returns>
int main(void)
{
    MessageBox(0, "RIGHT to grow triangle \n LEFT to shrink triangle \n UP to speed up rotation\n DOWN to reduce rotation speed ", "How to use", MB_OK);
    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    int count = 0;
    double omega = M_PI, omega0 = omega / 3;  double* angularV = &omega;

    std::cout << glfwGetVersionString() << std::endl;
    

    /* Create a windowed mode window and its OpenGL context */
    const double breite = 1280, hoehe = 720;
    double magniB = 100 / breite, magniH = 100 / hoehe;
    window = glfwCreateWindow((int)breite, (int)hoehe, "My Canvas", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }
    glClearColor(0, 0, 0, 0);

    vertex center;    center.x = 0.0; center.y = 0.0;
    std::uniform_real_distribution<double> unif(1, 2);
    std::default_random_engine re(time(0));
    long double velX = 150 * unif(re);    long double* velXP = &velX;
    long double velY = 150 * unif(re);     long double* velYP = &velY;
    velocity velo = { velX, velY }; velocity* velP = &velo;

    vertex* vp1, * vp2, * vp3, * vpc;
    vpc = &center;
    double iniLen = 100;
    triangle myEqui = makeEquiTri(vpc, iniLen); triangle* equiP = &myEqui;

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    if (glewInit != GLEW_OK)
        std::cout << "ERR" << std::endl;
    //glBegin(GL_TRIANGLES);

    double prevTime = 0;

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);
        double side = sqrt(pow(myEqui.aA.x - myEqui.bB.x, 2) + pow(myEqui.aA.y - myEqui.bB.y, 2));

        glBegin(GL_TRIANGLES);
        glVertex2d(myEqui.aA.x / breite, myEqui.aA.y / hoehe);
        glVertex2d(myEqui.bB.x / breite, myEqui.bB.y / hoehe);
        glVertex2d(myEqui.cC.x / breite, myEqui.cC.y / hoehe);
        glEnd();

        double currTime = glfwGetTime();

        triRotate(equiP, omega * (currTime - prevTime));  

        currTime = glfwGetTime();
        velocity dist = { velo.x * (currTime - prevTime), velo.y * (currTime - prevTime) };

        if (!triCollision(equiP, velP, breite, hoehe, angularV))
            triTranslate(equiP, dist);

        int stateR = glfwGetKey(window, GLFW_KEY_RIGHT), stateL = glfwGetKey(window, GLFW_KEY_LEFT);
        int stateU = glfwGetKey(window, GLFW_KEY_UP), stateD = glfwGetKey(window, GLFW_KEY_DOWN);
        if (stateR == GLFW_PRESS && side < 5 * iniLen && count % 100 == 0) {
            triResize(equiP, 1.1);
        }
        if (stateL == GLFW_PRESS && side > 0.5 * iniLen && count % 100 == 0) {
            triResize(equiP, 0.9);
        }
        if (stateU == GLFW_PRESS && abs(omega) < 4 * M_PI && count % 100 == 0)
            omega = (omega >= 0) ? omega + omega0 : omega - omega0;
        if (stateD == GLFW_PRESS && abs(omega) > 0.2 * M_PI && count % 100 == 0)
            omega = (omega >= 0) ? omega - omega0 : omega + omega0;

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
        prevTime = currTime;
        if (count >= 100)
            count = 0;
        count++;
    }

    glfwTerminate();
    return 0;
}