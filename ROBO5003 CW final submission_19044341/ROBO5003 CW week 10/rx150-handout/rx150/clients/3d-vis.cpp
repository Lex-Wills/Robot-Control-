#include <qapplication.h>
#include <iostream>

#include "../detail/opengl/SceneViewer.h"


#include <../rx150/RX150Kinematics.h>
#include <tcp/RX150RemoteInterface.h>



using namespace std;


void renderCylinderZ(double fromZ, double toZ, double width, double resolution=8){
    glPushMatrix();
    GLUquadric* obj = gluNewQuadric();
    glTranslated(0,0,fromZ);
    gluCylinder(obj, width, width, toZ-fromZ, resolution, resolution);
    gluDeleteQuadric(obj);
    glPopMatrix();
}

void renderArrow(double scale = 1.0){
    double length = scale*0.6;
    double width = scale*0.02;
    GLUquadric* obj = gluNewQuadric();
    gluCylinder(obj, width, width, length, 6, 6);
    glPushMatrix();
    glTranslated(0.0,0.0,length);
    gluCylinder(obj, width*2, 0.0, width*3, 6, 6);
    glPopMatrix();
    gluDeleteQuadric(obj);
}

void renderCoordinates(double scale = 1.0){
    glPushAttrib(GL_CURRENT_BIT);
    glColor3d(0,0,1);
    renderArrow(scale); // z axis

    glPushMatrix();
    glRotated(-90, 1.0, 0.0, 0.0);
    glColor3d(0,1,0);
    renderArrow(scale); // y axis
    glPopMatrix();

    glPushMatrix();
    glRotated(90, 0.0, 1.0, 0.0);
    glColor3d(1,0,0);
    renderArrow(scale); // x axis
    glPopMatrix();

    glPopAttrib();
}

void fillDHMatGL(double *mat, double alpha, double a, double theta, double d){
    Eigen::MatrixXd m = RX150Kinematics::getTransformMatrixFromDH(alpha, a, theta, d);
    for(uint i=0;i<4;i++){
        for(uint j=0;j<4;j++){
            mat[i+4*j] = m(i,j); // insert in transposed order for OpenGL
        }
    }
}

void renderBox(double lowX, double lowY, double lowZ, double highX, double highY, double highZ){
    glPushMatrix();

    glBegin(GL_POLYGON);
    glNormal3d(0,0,-1);
    glVertex3f( lowX,  lowY, lowZ);       // P1
    glVertex3f( lowX,  highY, lowZ);       // P2
    glVertex3f( highX, highY, lowZ);       // P3
    glVertex3f( highX, lowY, lowZ);       // P4
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3d(0,0,1);
    glVertex3f( highX, lowY,  highZ );
    glVertex3f( highX, highY, highZ );
    glVertex3f( lowX,  highY, highZ );
    glVertex3f( lowX,  lowY,  highZ );
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3d(1,0,0);
    glVertex3f( highX, lowY,  lowZ );
    glVertex3f( highX, highY, lowZ );
    glVertex3f( highX, highY, highZ );
    glVertex3f( highX, lowY,  highZ );
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3d(-1,0,0);
    glVertex3f( lowX, lowY,  highZ );
    glVertex3f( lowX, highY, highZ );
    glVertex3f( lowX, highY, lowZ );
    glVertex3f( lowX, lowY,  lowZ );
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3d(0,1,0);
    glVertex3f( highX, highY, highZ );
    glVertex3f( highX, highY, lowZ );
    glVertex3f( lowX,  highY, lowZ);
    glVertex3f( lowX,  highY, highZ );
    glEnd();
    glBegin(GL_POLYGON);
    glNormal3d(0,-1,0);
    glVertex3f( highX, lowY, lowZ );
    glVertex3f( highX, lowY, highZ );
    glVertex3f( lowX, lowY,  highZ );
    glVertex3f( lowX, lowY,  lowZ );
    glEnd();

    glPopMatrix();
}

class RX150Drawable : public Drawable {
public:
    RX150Drawable(std::shared_ptr<RX150RobotInterface> robotP){
        obj = gluNewQuadric();
        robot = robotP;
    }
    ~RX150Drawable(){
        gluDeleteQuadric(obj);
    }
    void renderBase(){
        // TODO
        renderBox(-0.13,-0.11,0.0,0.11,0.11,0.01);
        renderBox(-0.13,-0.08,0.0,-0.25,0.08,0.01);

        renderCylinderZ(0.01,0.08,0.1);
        renderBox(-0.12,-0.05,0.01,-0.23,0.05,0.05);
    }
    void renderFrame1(){
        // TODO
        renderBox(0.02,0.0,0.03,-0.02,-0.1,-0.03);

        renderCylinderZ(-0.03,0.03,0.025);
    }
    void renderFrame2(){
        // TODO
        glPushMatrix();
        glRotated(+19.0,0.0,0.0,1.0);
        renderBox(0.02,0.0,0.03,-0.02,0.07,-0.03);
        glPopMatrix();

        glPushMatrix();
        glRotated(+19.0,0.0,0.0,1.0);
        renderBox(0.02,0.07,0.03,-0.15,0.03,-0.03);
        glPopMatrix();

        renderCylinderZ(-0.03,0.03,0.025);
    }
    void renderFrame3(){
        // TODO
        renderBox(0.0,0.02,0.03,-0.16,-0.02,-0.03);

        renderCylinderZ(-0.03,0.03,0.025);
    }
    void renderFrame4(){
        // TODO
        renderBox(0.02,0.03,0.0,-0.02,-0.03,0.135);
    }
    void renderFrame5(){
        // TODO
        renderBox(0.025,0.035,0.0,-0.025,-0.035,0.01);

        renderBox(0.015,0.032,0.01,-0.015,0.025,0.08);
        renderBox(0.015,-0.025,0.01,-0.015,-0.032,0.08);

        glPushMatrix();
        glRotated(+90.0,1.0,0.0,0.0);
        glTranslated(0.0,0.025,0.0);
        renderCylinderZ(-0.039,0.039,0.008);
        glPopMatrix();
    }
    virtual void draw(QGLViewer* viewer){

        Eigen::VectorXd q = robot->getCurrentJointAngles();

        glEnable(GL_DEPTH_TEST);

        renderCoordinates(0.1);
        robotMaterial();
        renderBase();

        for(int i=0; i<5; i++){
            glPushMatrix();
            double mat[16];
            fillDHMatGL(mat,
                        RX150Kinematics::alpha[i],
                        RX150Kinematics::a[i],
                        q[i]+RX150Kinematics::thetaOff[i],
                        RX150Kinematics::d[i]);
            glMultMatrixd(mat);

            renderCoordinates(0.1);

            robotMaterial();
            switch(i){
                case 0:
                    renderFrame1();
                    break;
                case 1:
                    renderFrame2();
                    break;
                case 2:
                    renderFrame3();
                    break;
                case 3:
                    renderFrame4();
                    break;
                case 4:
                    renderFrame5();
                    break;
            }

        }
        for(int i=0; i<5; i++){
            glPopMatrix();
        }

    }
private:
    void robotMaterial(){
        glColor3d(0.5,0.5,0.5);

        float mat_ambient[] ={0.25f, 0.25f, 0.25f, 1.0f  };
        float mat_diffuse[] ={0.4f, 0.4f, 0.4f, 1.0f };
        float mat_specular[] ={0.774597f, 0.774597f, 0.774597f, 1.0f };
        float shine = 0.768f;
        glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
        glMaterialf(GL_FRONT, GL_SHININESS, shine * 128.0);
    }

    double r,g,b;
    GLUquadric* obj;
    std::shared_ptr<RX150RobotInterface> robot;
};

class SphereDrawable : public Drawable {
public:
    SphereDrawable(double x, double y, double z, int c){
        this->x = x;
        this->y = y;
        this->z = z;
        switch(c){
            case 1:
                r=1.0; g=0.0; b=0.0;
                break;
            case 2:
                r=0.0; g=1.0; b=0.0;
                break;
            case 3:
                r=0.0; g=0.0; b=1.0;
                break;
            default:
                r=0.5; g=0.5; b=0.5;
        }

        obj = gluNewQuadric();
    }
    ~SphereDrawable(){
        gluDeleteQuadric(obj);
    }
    virtual void draw(QGLViewer* viewer){

        glPushMatrix();
        glTranslated(x,y,z);

        //glShadeModel(GL_SMOOTH);
        //glShadeModel(GL_FLAT);

        const unsigned sphereSlices = 12;
        const unsigned sphereStacks = 12;
        const double radius = 0.05;
        glColor3d(r,g,b);
        gluSphere(obj, radius,sphereSlices,sphereStacks);


        glPopMatrix();
    }
private:
    double x;
    double y;
    double z;
    double r,g,b;
    GLUquadric* obj;
};

int main(int argc, char** argv){
    QApplication application(argc,argv);

    string remoteHost = "localhost";
    if(argc==2){
        remoteHost = argv[1];
    }

    boost::shared_ptr<SceneViewer> viewer = SceneViewer::create();

//    viewer->addDrawable(new SphereDrawable(0.0, 0.0, 0.0, 0));
    viewer->addDrawable(new SphereDrawable(1.0, 0.0, 0.0, 1));
    viewer->addDrawable(new SphereDrawable(0.0, 1.0, 0.0, 2));
    viewer->addDrawable(new SphereDrawable(0.0, 0.0, 1.0, 3));

    viewer->addDrawable(new RX150Drawable(RX150RemoteInterface::create(remoteHost)));

    viewer->setSceneRadius(5.0);
    viewer->show();
    viewer->restoreStateFromFile();

    return application.exec();
}
