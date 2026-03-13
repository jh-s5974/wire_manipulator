#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <tuple>


using namespace Eigen;
using namespace std;

namespace kin_2rsu {
    static const double PI = 3.14159265358979323846;

    Matrix2f ankle_J(float R, float P, float t1, float t2, char rl){
        float l1 = 46.25;
        float l2_l = 298.457;
        float l2_s = 210.997;
        float l3 = 300;
        float w1 = 64.75;
        float Cx = 37;
        float Cy = 55.5;
        float Cz = -22.5;
        float origin_t = (31.29*PI/180);

        Matrix3f ROF;
        Vector3f OB1;
        Vector3f OB2;
        Vector3f FC1;
        Vector3f FC2;
        Vector3f OF;
        Vector3f BC1;
        Vector3f BC2;
        Vector3f AB1;
        Vector3f AB2;
        Vector2f A;
        Vector3f B1;
        Vector3f B2;
        Matrix2f C;

        ROF << cos(P), sin(P)*sin(R), cos(R)*sin(P),
                0,        cos(R),       -sin(R),
                -sin(P), cos(P)*sin(R), cos(P)*cos(R);

        if (rl == 'r'){
            OB1 << l1*cos(t1), w1, -l1*sin(t1);
            OB2 << l1*cos(t2), -w1, -87.5-l1*sin(t2);
        }
        else if (rl == 'l'){
            OB1 << l1*cos(t1), w1, -87.5-l1*sin(t1);
            OB2 << l1*cos(t2), -w1, -l1*sin(t2);
        }

        FC1 << Cx, Cy, Cz;
        FC2 << Cx, -Cy, Cz;

        OF << 0,0,-l3;

        BC1 = OF + ROF*FC1 - OB1;
        BC2 = OF + ROF*FC2 - OB2;

        AB1 << l1*cos(t1+origin_t), 0, -l1*sin(t1+origin_t);
        AB2 << l1*cos(t2+origin_t), 0, -l1*sin(t2+origin_t);
        // AB1 << l1*cos(t1), 0, -l1*sin(t1);
        // AB2 << l1*cos(t2), 0, -l1*sin(t2);

        Vector3f X(1,0,0);
        Vector3f Y(0,1,0);

        A << Y.dot(AB1.cross(BC1)),
            Y.dot(AB2.cross(BC2));

        B1 << FC1.cross(BC1);
        B2 << FC2.cross(BC2);

        C << X.dot(B1), Y.dot(B1),
            X.dot(B2), Y.dot(B2);

        Matrix2f J_1;
        J_1.row(0) = C.row(0) / A(0);
        J_1.row(1) = C.row(1) / A(1);
        Matrix2f J = J_1.inverse();

        return J;
    }

    Vector2f ankle_ik(float R, float P, char rl){
        float l1 = 46.25;
        float l2_l = 298.457;
        float l2_s = 210.997;
        float l3 = 300;
        float w1 = 64.75;
        float Cx = 37;
        float Cy = 55.5;
        float Cz = -22.5;
        float origin_t = (31.29*PI/180);
        
        double A1;
        double B1;
        double C1;
        double A2;
        double B2;
        double C2;

        if(rl == 'r'){
            A1 = Cx*Cx + Cy*Cy + Cz*Cz + l1*l1 - l2_l*l2_l + l3*l3 + w1*w1 - Cz*l3*cos(P + R) - Cy*l3*sin(P + R) - 2*Cy*w1*cos(R) + 2*Cx*l3*sin(P) - Cz*l3*cos(P - R) + 2*Cz*w1*sin(R) + Cy*l3*sin(P - R);
            B1 = - 2*l1*(l3*sin((1043*PI)/6000) + Cx*cos((1043*PI)/6000)*cos(P) + Cx*sin((1043*PI)/6000)*sin(P) + Cz*cos((1043*PI)/6000)*cos(R)*sin(P) - Cz*sin((1043*PI)/6000)*cos(P)*cos(R) + Cy*cos((1043*PI)/6000)*sin(P)*sin(R) - Cy*sin((1043*PI)/6000)*cos(P)*sin(R));
            C1 = 2*l1*(Cx*sin((1043*PI)/6000)*cos(P) - Cx*cos((1043*PI)/6000)*sin(P) - l3*cos((1043*PI)/6000) + Cz*cos((1043*PI)/6000)*cos(P)*cos(R) + Cy*cos((1043*PI)/6000)*cos(P)*sin(R) + Cz*sin((1043*PI)/6000)*cos(R)*sin(P) + Cy*sin((1043*PI)/6000)*sin(P)*sin(R));

            A2 = Cx*Cx + Cy*Cy + Cz*Cz + l1*l1 - l2_s*l2_s + l3*l3 + w1*w1 + (175*Cz*cos(P + R))/2 - 175*l3 - (175*Cy*sin(P + R))/2 - 175*Cx*sin(P) + (175*Cz*cos(P - R))/2 + (175*Cy*sin(P - R))/2  - Cz*l3*cos(P + R) + Cy*l3*sin(P + R) - 2*Cy*w1*cos(R) + 2*Cx*l3*sin(P) - Cz*l3*cos(P - R) - 2*Cz*w1*sin(R) - Cy*l3*sin(P - R) + 30625/4;
            B2 = -l1*(2*l3*sin((1043*PI)/6000) - 175*sin((1043*PI)/6000) + 2*Cx*cos((1043*PI)/6000)*cos(P) + 2*Cx*sin((1043*PI)/6000)*sin(P) + 2*Cz*cos((1043*PI)/6000)*cos(R)*sin(P) - 2*Cz*sin((1043*PI)/6000)*cos(P)*cos(R) - 2*Cy*cos((1043*PI)/6000)*sin(P)*sin(R) + 2*Cy*sin((1043*PI)/6000)*cos(P)*sin(R));
            C2 = l1*(175*cos((1043*PI)/6000) - 2*l3*cos((1043*PI)/6000) - 2*Cx*cos((1043*PI)/6000)*sin(P) + 2*Cx*sin((1043*PI)/6000)*cos(P) + 2*Cz*cos((1043*PI)/6000)*cos(P)*cos(R) - 2*Cy*cos((1043*PI)/6000)*cos(P)*sin(R) + 2*Cz*sin((1043*PI)/6000)*cos(R)*sin(P) - 2*Cy*sin((1043*PI)/6000)*sin(P)*sin(R));
        }
        else if(rl == 'l'){
            A1 = Cx*Cx + Cy*Cy + Cz*Cz + l1*l1 - l2_s*l2_s + l3*l3 + w1*w1 + (175*Cz*cos(P + R))/2- 175*l3+ (175*Cy*sin(P + R))/2- 175*Cx*sin(P)+ (175*Cz*cos(P - R))/2- (175*Cy*sin(P - R))/2- Cz*l3*cos(P + R)- Cy*l3*sin(P + R)- 2*Cy*w1*cos(R)+ 2*Cx*l3*sin(P)- Cz*l3*cos(P - R)+ 2*Cz*w1*sin(R)+ Cy*l3*sin(P - R)+ 30625/4;
            B1 = -l1*(2*l3*sin((1043*PI)/6000) - 175*sin((1043*PI)/6000) + 2*Cx*cos((1043*PI)/6000)*cos(P) + 2*Cx*sin((1043*PI)/6000)*sin(P) + 2*Cz*cos((1043*PI)/6000)*cos(R)*sin(P) - 2*Cz*sin((1043*PI)/6000)*cos(P)*cos(R) + 2*Cy*cos((1043*PI)/6000)*sin(P)*sin(R) - 2*Cy*sin((1043*PI)/6000)*cos(P)*sin(R));
            C1 = l1*(175*cos((1043*PI)/6000) - 2*l3*cos((1043*PI)/6000) - 2*Cx*cos((1043*PI)/6000)*sin(P) + 2*Cx*sin((1043*PI)/6000)*cos(P) + 2*Cz*cos((1043*PI)/6000)*cos(P)*cos(R) + 2*Cy*cos((1043*PI)/6000)*cos(P)*sin(R) + 2*Cz*sin((1043*PI)/6000)*cos(R)*sin(P) + 2*Cy*sin((1043*PI)/6000)*sin(P)*sin(R));
            
            A2 = Cx*Cx + Cy*Cy + Cz*Cz + l1*l1 - l2_l*l2_l + l3*l3 + w1*w1 - Cz*l3*cos(P + R) + Cy*l3*sin(P + R) - 2*Cy*w1*cos(R) + 2*Cx*l3*sin(P) - Cz*l3*cos(P - R) - 2*Cz*w1*sin(R) - Cy*l3*sin(P - R);
            B2 = (2*Cz*l1*sin((1043*PI)/6000)*cos(P)*cos(R) - 2*Cx*l1*cos((1043*PI)/6000)*cos(P) - 2*Cx*l1*sin((1043*PI)/6000)*sin(P) - 2*Cz*l1*cos((1043*PI)/6000)*cos(R)*sin(P) - 2*l1*l3*sin((1043*PI)/6000) + 2*Cy*l1*cos((1043*PI)/6000)*sin(P)*sin(R) - 2*Cy*l1*sin((1043*PI)/6000)*cos(P)*sin(R));
            C2 = (2*Cx*l1*sin((1043*PI)/6000)*cos(P) - 2*Cx*l1*cos((1043*PI)/6000)*sin(P) - 2*l1*l3*cos((1043*PI)/6000) + 2*Cz*l1*cos((1043*PI)/6000)*cos(P)*cos(R) - 2*Cy*l1*cos((1043*PI)/6000)*cos(P)*sin(R) + 2*Cz*l1*sin((1043*PI)/6000)*cos(R)*sin(P) - 2*Cy*l1*sin((1043*PI)/6000)*sin(P)*sin(R));
        }

        Vector2f t1;
        t1 << (-2*C1 + sqrt(4*C1*C1 - 4*(A1-B1)*(A1+B1)))/(2*(A1-B1)), 
            (-2*C1 - sqrt(4*C1*C1 - 4*(A1-B1)*(A1+B1)))/(2*(A1-B1));
        
        Vector2f t2;
        t2 << (-2*C2 + sqrt(4*C2*C2 - 4*(A2-B2)*(A2+B2)))/(2*(A2-B2)), 
            (-2*C2 - sqrt(4*C2*C2 - 4*(A2-B2)*(A2+B2)))/(2*(A2-B2));

        Vector2f theta;
        if(abs(2*atan(t1(0))*180/PI) < 80){
            theta(0) = 2*atan(t1(0));
        }
        else if(abs(2*atan(t1(1))*180/PI) < 80){
            theta(0) = 2*atan(t1(1));
        }
        if(abs(2*atan(t2(0))*180/PI) < 80){
            theta(1) = 2*atan(t2(0));
        }
        else if(2*atan(t2(1)*180/PI) < 80){
            theta(1) = 2*atan(t2(1));
        }

        return theta;
    }

    tuple<Vector2f,Matrix2f> ankle_fk(float t1, float t2, char rl){
        Vector2f theta;
        float alpha = 1;
        Matrix2f jac;
        Vector2f error;
        Vector2f ankle_rp(0,0); 
        for(int i = 0; i < 12; i++){
            theta = ankle_ik(ankle_rp(0), ankle_rp(1), rl);
            error << t1-theta(0), t2-theta(1);
            jac = ankle_J(ankle_rp(0), ankle_rp(1), theta(0), theta(1), rl);
            ankle_rp += alpha*jac*error;
            if(sqrt(pow(error(0),2) + pow(error(1),2)) < 1e-4){
                break;
            }
        }
        return make_tuple(ankle_rp, jac);
    }
};