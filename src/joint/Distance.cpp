#include "joint/Distance.h"
#include "rigidbody/RigidBody.h"

#include <Eigen/Dense>

Distance::Distance() : Joint()
{

}

Distance::Distance(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Vector3f& _r1, float _d) : Joint(_body0, _body1, _r0, Eigen::Quaternionf::Identity(), _r1, Eigen::Quaternionf::Identity(), kDistance), d(_d)
{
    dim = 1;
    J0.setZero(1, 6);
    J1.setZero(1, 6);
    J0Minv.setZero(1, 6);
    J1Minv.setZero(1, 6);
    phi.setZero(1);
    lambda.setZero(1);
}

void Distance::computeJacobian()
{
    // TODO Objective #2
    // Compute the Jacobians J0,J1, J0Minv, J1Min and constraint error phi
    // of the distance constraint.
    //

    // Positions des points d'attachement dans le repère du monde
    Eigen::Vector3f p0 = body0->x + body0->theta * r0;
    Eigen::Vector3f p1 = body1->x + body1->theta * r1;

    // Vecteur entre les deux points d'attachement
    Eigen::Vector3f dVec = p1 - p0;

    // Longueur actuelle du vecteur
    float dist = dVec.norm();

    // Direction de la contrainte
    Eigen::Vector3f n = dVec / dist;

    // Calcul de l'erreur de contrainte
    phi[0] = dist - d;

    // Jacobien pour body0
    J0.block<1, 3>(0, 0) = -n.transpose();
    J0.block<1, 3>(0, 3) = -(body0->theta * r0).cross(n).transpose();

    // Jacobien pour body1
    J1.block<1, 3>(0, 0) = n.transpose();
    J1.block<1, 3>(0, 3) = (body1->theta * r1).cross(n).transpose();

    // Précalcul des termes J0Minv et J1Minv
    J0Minv.block<1, 3>(0, 0) = J0.block<1, 3>(0, 0) * (1.0f / body0->mass);
    J0Minv.block<1, 3>(0, 3) = J0.block<1, 3>(0, 3) * body0->Iinv;
    J1Minv.block<1, 3>(0, 0) = J1.block<1, 3>(0, 0) * (1.0f / body1->mass);
    J1Minv.block<1, 3>(0, 3) = J1.block<1, 3>(0, 3) * body1->Iinv;
}

