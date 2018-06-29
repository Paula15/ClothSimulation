/**
 * Each frame:
 * 
 * 1. add forces -> acceleration
 * 2. timestep
 *     (2.1) all particles: satisfy constraints
 *     (2.2) all particles: Verlet Integration
 * 3. flush
 */
#include <vector>
#include <iostream>

#include <GL/glut.h> 

#include "vec3.h"

using namespace std;

const int kNumParticlesW = 55;
const int kNumParticlesH = 45;
const int kNumConstraintIterations = 15;
const float kDamping = 0.01;

class Cloth {

    int w, h;

    struct Particle {
        bool movable;
        Vec3 pos, old_pos;
        Vec3 normal;
        Vec3 a;
        float m;

        Particle() = default;

        Particle(const Vec3 &_pos): movable(true), pos(_pos), old_pos(_pos), normal(Vec3(0, 0, 0)),
            a(Vec3(0, 0, 0)), m(1) {}

        void TimeStep(float timestep2) {
            if (!movable) return;
            Vec3 new_pos = pos + (pos - old_pos) * (1 - kDamping) + a * timestep2;
            old_pos = pos; pos = new_pos;
            a = Vec3(0, 0, 0);
        }

        Vec3 GetNormal() const {
            return normal.normalized();
        }
    };

    struct Constraint {
        float rest_distance;
        Particle *a, *b;

        Constraint(Particle *a, Particle *b): a(a), b(b) {
            rest_distance = (a->pos - b->pos).length();
        }

        void SatisfyConstraint() {
            Vec3 dir = a->pos - b->pos;
            Vec3 delta = dir * (1 - rest_distance / dir.length()) / 2.0;
            if (a->movable) a->pos -= delta; 
            if (b->movable) b->pos += delta;
        }
    };

    vector<vector<Particle>> particles_;
    vector<Constraint> constraints_;

    void InitParticles() {
        particles_.resize(kNumParticlesH);
        for (int i = 0; i < kNumParticlesH; i++) {
            particles_[i].resize(kNumParticlesW);
        }
        for (int i = 0; i < kNumParticlesH; i++) {
            for (int j = 0; j < kNumParticlesW; j++) {
                float x = j * w / float(kNumParticlesW);
                float y = -i * h / float(kNumParticlesH);
                particles_[i][j] = Particle(Vec3(x, y, 0));
            }
        }
    }

    void InitConstraints() {
        auto MakeConstraint = [this](int i, int j, int x, int y) {
            constraints_.push_back(Constraint(&particles_[i][j], &particles_[x][y]));
        };

        for (int d: {1, 2}) {
            for (int i = 0; i < kNumParticlesH; i++) {
                for (int j = 0; j < kNumParticlesW; j++) {
                    if (i + d < kNumParticlesH) MakeConstraint(i, j, i + d, j);
                    if (j + d < kNumParticlesW) MakeConstraint(i, j, i, j + d);
                    if (i + d < kNumParticlesH && j + d < kNumParticlesW) {
                        MakeConstraint(i, j, i + d, j + d);
                        MakeConstraint(i + d, j, i, j + d);
                    }
                }
            }
        }
    }

    void InitUnmovables() {
        // making the upper left most three and right most three particles unmovable
        for (int i = 0; i < 3; i++) {
            Particle &left = particles_[0][i];
            Particle &right = particles_[0][kNumParticlesW - i - 1];

            left.pos += Vec3(0.5, 0, 0);
            left.movable = false;
            right.pos += Vec3(-0.5, 0, 0);
            right.movable = false;
        }
    }

    Vec3 GetNormal(const Particle &a, const Particle &b, const Particle &c) {
        return (b.pos - a.pos).cross(c.pos - a.pos).normalized();
    }

public:
    Cloth() = default;

    Cloth(int w, int h): w(w), h(h) {
        InitParticles();
        InitConstraints();        
        InitUnmovables();
    }

    void AddForce(const Vec3 &force) {
        for (int i = 0; i < kNumParticlesH; i++) {
            for (int j = 0; j < kNumParticlesW; j++) {
                particles_[i][j].a += force / particles_[i][j].m;
            }
        }
    }

    void WindForce(const Vec3 &wind) {
        // (i, j)   +--+ (i, j+1)
        //          | /|
        //          |/ |
        // (i+1, j) +--+ (i+1, j+1)
        auto WindForceTriangle = [this](const Vec3 &wind, Particle *a, Particle *b, Particle *c) {
            Vec3 normal = GetNormal(*a, *b, *c);
            Vec3 force = normal * (wind.dot(normal));
            a->a += force / a->m;
            b->a += force / b->m;
            c->a += force / c->m;
        };

        for (int i = 0; i < kNumParticlesH - 1; i++) {
            for (int j = 0; j < kNumParticlesW - 1; j++) {
                WindForceTriangle(wind, &particles_[i][j], &particles_[i + 1][j], &particles_[i][j + 1]);
                WindForceTriangle(wind, &particles_[i + 1][j], &particles_[i + 1][j + 1], &particles_[i][j + 1]);
            }
        }
    }

    void TimeStep(float timestep2) {
        // (2.1) all constraints: satisfy constraints
        for (int iter = 0; iter < kNumConstraintIterations; ++iter) {
            for (auto c = constraints_.begin(); c < constraints_.end(); ++c) {
                c->SatisfyConstraint();
            }
        }

        // (2.2) all particles: Verlet Integration
        for (int i = 0; i < kNumParticlesH; i++) {
            for (int j = 0; j < kNumParticlesW; j++) {
                particles_[i][j].TimeStep(timestep2);
            }
        }        
    }

    void GLDraw() {
        // (i, j)   +--+ (i, j+1)
        //          | /|
        //          |/ |
        // (i+1, j) +--+ (i+1, j+1)
        for (int i = 0; i < kNumParticlesH; i++) {
            for (int j = 0; j < kNumParticlesW; j++) {
                particles_[i][j].normal = Vec3(0, 0, 0);
            }
        }

        for (int i = 0; i < kNumParticlesH - 1; i++) {
            for (int j = 0; j < kNumParticlesW - 1; j++) {
                Vec3 normal = GetNormal(particles_[i][j], particles_[i + 1][j], particles_[i][j + 1]);
                particles_[i][j].normal += normal;
                particles_[i + 1][j].normal += normal;
                particles_[i][j + 1].normal += normal;

                normal = GetNormal(particles_[i + 1][j + 1], particles_[i][j + 1], particles_[i + 1][j]);
                particles_[i + 1][j + 1].normal += normal;
                particles_[i][j + 1].normal += normal;
                particles_[i + 1][j].normal += normal;
            }
        }

        auto GLDrawTriangle = [](const Particle &a, const Particle &b, const Particle &c, const Vec3 &color) {
            glColor3fv((GLfloat*) &color);
            glNormal3fv((GLfloat*) &(a.GetNormal()));
            glVertex3fv((GLfloat*) &(a.pos));
            glNormal3fv((GLfloat*) &(b.GetNormal()));
            glVertex3fv((GLfloat*) &(b.pos));
            glNormal3fv((GLfloat*) &(c.GetNormal()));
            glVertex3fv((GLfloat*) &(c.pos));
        };

        glBegin(GL_TRIANGLES);
        for (int i = 0; i < kNumParticlesH - 1; i++) {
            for (int j = 0; j < kNumParticlesW - 1; j++) {
                const Vec3 color(0.7f, 0.2f, 0.2f);
                GLDrawTriangle(particles_[i][j], particles_[i + 1][j], particles_[i][j + 1], color);
                GLDrawTriangle(particles_[i + 1][j + 1], particles_[i][j + 1], particles_[i + 1][j], color);
            }
        }
        glEnd();
    }
};


class Engine {
    Cloth cloth_; 

    const int kClothW;
    const int kClothH;
    const float kTimeStep2;

public:
    Engine(): kClothW(14), kClothH(10), kTimeStep2(0.5 * 0.5) {
        cloth_ = Cloth(kClothW, kClothH);
    }

    void GLInit(GLvoid) {
        glShadeModel(GL_SMOOTH);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_COLOR_MATERIAL);
        
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        GLfloat lightPos[4] = {-1.0,1.0,0.5,0.0};
        glLightfv(GL_LIGHT0,GL_POSITION,(GLfloat *) &lightPos);

        glEnable(GL_LIGHT1);

        GLfloat lightAmbient1[4] = {0.0,0.0,0.0,0.0};
        GLfloat lightPos1[4] = {1.0,0.0,-0.2,0.0};
        GLfloat lightDiffuse1[4] = {0.5,0.5,0.3,0.0};

        glLightfv(GL_LIGHT1,GL_POSITION,(GLfloat *) &lightPos1);
        glLightfv(GL_LIGHT1,GL_AMBIENT,(GLfloat *) &lightAmbient1);
        glLightfv(GL_LIGHT1,GL_DIFFUSE,(GLfloat *) &lightDiffuse1);
    }

    void GLDisplay(void) {
        // 1. add forces
        cloth_.AddForce(Vec3(0, -0.2, 0) * kTimeStep2);    // gravity. TODO: why not mg?
        cloth_.WindForce(Vec3(0.05, 0, 0.02) * kTimeStep2);  // wind.

        // 2. timestep
        cloth_.TimeStep(kTimeStep2);

        // 3. draw
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	    glLoadIdentity();

        glTranslatef(-6.5, 6, -9.0f);
        glRotatef(25, 0, 1, 0);
        cloth_.GLDraw();

        glutSwapBuffers();
	    glutPostRedisplay();
    }

    void GLReshape(int w, int h) {
        glViewport(0, 0, w, h);
        glMatrixMode(GL_PROJECTION); 
        glLoadIdentity();  
        if (h == 0)  
            gluPerspective(80, (float)w, 1.0, 5000.0);
        else
            gluPerspective(80, (float)w / (float)h, 1.0, 5000.0);
        glMatrixMode(GL_MODELVIEW);  
        glLoadIdentity(); 
    }
};

Engine e;
void DisplayFunc(void) { e.GLDisplay(); }
void ReshapeFunc(int w, int h) { e.GLReshape(w, h); }

int main(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH); 
	glutInitWindowSize(1280, 720); 
    glutCreateWindow("Cloth Tutorial from cg.alexandra.dk");

    e.GLInit();
    glutDisplayFunc(DisplayFunc);
    glutReshapeFunc(ReshapeFunc);

    glutMainLoop();
}