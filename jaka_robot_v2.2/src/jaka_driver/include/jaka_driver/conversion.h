#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"
#include "cmath"
#include "iostream"

RotMatrix Angaxis2Rot(Eigen::Vector3d &agax3){
    Eigen::Vector3d rvec;
    double angle;
    //如果输入全为0，angle为0，不可做分母
    if(!agax3[0] && !agax3[1] && !agax3[2]){
        RotMatrix r;
        r = {1,0,0,
            0,1,0,
            0,0,1};
        return r;
    }
    else{
        angle = sqrt(pow(agax3[0],2)+pow(agax3[1],2)+pow(agax3[2],2));
        rvec = Eigen::Vector3d(agax3[0]/angle,agax3[1]/angle,agax3[2]/angle);
        Eigen::AngleAxisd rotationVector(angle,rvec.normalized());
    
        Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
        
        rotationMatrix = rotationVector.toRotationMatrix();

        CartesianTran a,b,c;
        a.x = rotationMatrix(0,0);
        a.y = rotationMatrix(1,0);
        a.z = rotationMatrix(2,0);
        b.x = rotationMatrix(0,1);
        b.y = rotationMatrix(1,1);
        b.z = rotationMatrix(2,1);
        c.x = rotationMatrix(0,2);
        c.y = rotationMatrix(1,2);
        c.z = rotationMatrix(2,2);

        RotMatrix r;
        r.x = a;
        r.y = b;
        r.z = c;
        return r;
    }
}

Eigen::Vector3d Rot2Angaxis(RotMatrix &r){
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix<<r.x.x, r.y.x, r.z.x,
		            r.x.y, r.y.y, r.z.y,
                    r.x.z, r.y.z, r.z.z;
    // std::cout << "h.rotationMatrix = " << rotationMatrix << std::endl;               
    Eigen::AngleAxisd rotationVector(rotationMatrix);
    double angle = rotationVector.angle();
    //std::cout << "h.angle = " << angle << std::endl;
    Eigen::Vector3d axis = rotationVector.axis();
    //std::cout << "h.axis = " << axis << std::endl;
    Eigen::Vector3d v;
    v = angle*axis;

    
    return v;
}

