#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <cmath>
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>


struct vertex { double x; double y; };
struct velocity { double x; double y; };
struct triangle { vertex aA; vertex bB; vertex cC; vertex zZ;  };
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
    myEqui.aA = { center->x -len / 2, center->y -len*sqrt(3) / 6 };
    myEqui.bB = { center->x + len / 2, center->y - len* sqrt(3) / 6 };
    myEqui.cC = { center->x, center->y + len*sqrt(3) / 3};
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

void triRotate(struct triangle* tria, double phi) {
    vertex* pA = &tria->aA, * pB = &tria->bB, * pC = &tria->cC, * pZ = &tria->zZ;
    rotation(phi, pA, pZ);
    rotation(phi, pB, pZ);
    rotation(phi, pC, pZ);
    //delete pA, pB,pC,pZ;
}

void triResize(struct triangle* tria, double coeff) {
    tria->aA.x = tria->zZ.x + (tria->aA.x - tria->zZ.x) * coeff;
    tria->bB.x = tria->zZ.x + (tria->bB.x - tria->zZ.x) * coeff;
    tria->cC.x = tria->zZ.x + (tria->cC.x - tria->zZ.x) * coeff;
    tria->aA.y = tria->zZ.y + (tria->aA.y - tria->zZ.y) * coeff;
    tria->bB.y = tria->zZ.y + (tria->bB.y - tria->zZ.y) * coeff;
    tria->cC.y = tria->zZ.y + (tria->cC.y - tria->zZ.y) * coeff;
}

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
void triReflect(struct triangle* tria, double dist, bool yRicht) {
    if (yRicht) {
        tria->aA.y = tria->aA.y* dist;
        tria->bB.y = tria->bB.y* dist;
        tria->cC.y = tria->cC.y* dist;
        tria->zZ.y = tria->zZ.y * dist;
    }
    else {
        tria->aA.x = tria->aA.x* dist;
        tria->bB.x = tria->bB.x * dist;
        tria->cC.x = tria->cC.x * dist;
        tria->zZ.x = tria->zZ.x * dist;
    }
}

bool triCollision(struct triangle* tria, velocity* velo, double breite, double hoehe, 
                    double* omega, double times) {
    vertex* pA = &tria->aA, * pB = &tria->bB, * pC = &tria->cC, * pZ =&tria->zZ;
    double len = sqrt(pow(pA->x - pZ->x, 2) + pow(pA->y - pZ->y, 2));
    double coVel = 0.9, coAn = 0.5;
    if (pA->y <= -hoehe || pA->y >= hoehe) {
        std::cout << velo->y << " _____ ";
        double alpha = atan((pA->y - pZ->y) / (pA->x - pZ->x));
        /*velo->y = coVel*(velo->y * cos(alpha) * cos(alpha) - 2 * sqrt(3) * len * cos(alpha) * *omega - 3 * velo->y)
            / (3 + cos(alpha) * cos(alpha));
        *omega = -coAn*(3 * *omega - *omega * cos(alpha) * cos(alpha) - 2 * sqrt(3) * velo->y * cos(alpha))
                /(3+cos(alpha)*cos(alpha));*/
        velo->y = -velo->y; 
        while (pA->y <= -hoehe || pA->y >= hoehe) {
            triReflect(tria, 0.99,true);
            std::cout << velo->y << "y@A" << std::endl;
        }
        *omega = -*omega;
        return true;
    }
    if (pB->y <= -hoehe || pB->y>= hoehe ) {
        std::cout << velo->y << " _____ ";
        double alpha = atan((pB->y - pZ->y) / (pB->x - pZ->x));
        /*velo->y = coVel*(velo->y * cos(alpha) * cos(alpha) - 2 * sqrt(3) * len * cos(alpha) * *omega - 3 * velo->y)
            / (3 + cos(alpha) * cos(alpha));
        *omega = -coAn*(3 * *omega - *omega * cos(alpha) * cos(alpha) - 2 * sqrt(3) * velo->y * cos(alpha)) 
            / (3 + cos(alpha) * cos(alpha));*/
        velo->y = -velo->y; 
        while (pB->y <= -hoehe || pB->y >= hoehe) {
            triReflect(tria, 0.99, true);
            std::cout << velo->y << "y@B" << std::endl;
        }
        *omega = -*omega; return true;
    }
    if (pC->y  <= -hoehe  || pC->y  >= hoehe ) {
        std::cout << velo->y << " _____ ";
        double alpha = atan((pC->y - pZ->y) / (pC->x - pZ->x));
        /*velo->y = coVel*(velo->y * cos(alpha) * cos(alpha) - 2 * sqrt(3) * len * cos(alpha) * *omega - 3 * velo->y)
            / (3 + cos(alpha) * cos(alpha));
        *omega = -coAn*(3 * *omega - *omega * cos(alpha) * cos(alpha) - 2 * sqrt(3) * velo->y * cos(alpha)) 
            / (3 + cos(alpha) * cos(alpha));*/
        velo->y = -velo->y; 
        while (pC->y <= -hoehe || pC->y >= hoehe) {
            triReflect(tria, 0.99, true);
            std::cout << velo->y << "y@C" << std::endl;
        }
        *omega = -*omega; return true;
    }
    if (pA->x <= -breite || pA->x >= breite) {
        velo->x = -velo->x; 
        while (pA->x <= -breite|| pA->x >= breite) {
            triReflect(tria, 0.99, false);
            std::cout << velo->x << "x@A" << std::endl;
        }
        *omega = -*omega;
        return true;
    }
    if (pB->x <= -breite || pB->x >= breite) {
        velo->x = -velo->x;
        while (pB->x <= -breite || pB->x >= breite) {
            triReflect(tria,0.99, false);
            std::cout << velo->x << "x@B" << std::endl;
        } 
        *omega = -*omega; return true;
    }
    if (pC->x <= -breite || pC->x >= breite) {
        velo->x = -velo->x; 
        while (pC->x <= -breite || pC->x >= breite) {
            triReflect(tria,0.99, false);
            std::cout << velo->x << "x@C" << std::endl;
        }  
        *omega = -*omega; return true;
    }
    
    return false;
}
//void triCompensator(struct triangle* tria, double og) {
//    vertex* pA = &tria->aA, * pB = &tria->bB, * pC = &tria->cC;
//    compensator(pA, tria->zZ, og);
//    compensator(pB, tria->zZ, og);
//    compensator(pC, tria->zZ, og);
//}

//void key_callback(GLFWwindow* window, int key, int action)
//{
//    if (key == GLFW_KEY_E && action == GLFW_PRESS)
//        std::cout << "pressed!" << std::endl;
//}

//void makeVertex(double x, double y)

int main(void)
{
    GLFWwindow* window;


    /* Initialize the library */
    if (!glfwInit())
        return -1;


    int count = 0;
    double omega = M_PI , omega0 = omega/3;  double* angularV = &omega;
    
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
    glClearColor(0, 0, 0,0);
    vertex center;    center.x = 0.0; center.y = 0.0;
    std::uniform_real_distribution<double> unif(1, 2);
    std::default_random_engine re(time(0));
    long double velX = 100*unif(re);    long double* velXP = &velX;
    long double velY = 100*unif(re);     long double* velYP = &velY;
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
        //std::cout << side <<" . " << velo.x << std::endl;

        //std::cout << prevTime << "     " << 1 << std::endl;
        glBegin(GL_TRIANGLES);
        glVertex2d(myEqui.aA.x /breite, myEqui.aA.y /hoehe);
        glVertex2d(myEqui.bB.x /breite, myEqui.bB.y /hoehe);
        glVertex2d(myEqui.cC.x /breite, myEqui.cC.y /hoehe);
        glEnd();

        double currTime = glfwGetTime();

        triRotate(equiP, omega * (currTime - prevTime));   //triCompensator(equiP, iniLen);

        currTime = glfwGetTime();
        velocity dist = { velo.x * (currTime - prevTime), velo.y * (currTime - prevTime) };

        if (!triCollision(equiP, velP, breite, hoehe, angularV, currTime - prevTime)) {
            triTranslate(equiP, dist);
        }
        else std::cout << "Collide!" << std::endl;
        
        int stateR = glfwGetKey(window, GLFW_KEY_RIGHT), stateL = glfwGetKey(window, GLFW_KEY_LEFT);
        int stateU = glfwGetKey(window, GLFW_KEY_UP), stateD = glfwGetKey(window, GLFW_KEY_DOWN);
        if (stateR == GLFW_PRESS && side < 5*iniLen && count % 100 == 0) {
            triResize(equiP, 1.1);
        }
        if (stateL == GLFW_PRESS && side > 0.5*iniLen && count % 100 == 0) {
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