#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"

float Contact::mu = 0.8f;

Contact::Contact() : p(), n(), t(), b()
{

}

Contact::Contact(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _p, const Eigen::Vector3f& _n, float _pene) :
    body0(_body0), body1(_body1),
    p(_p), n(_n), t(), b(), pene(_pene)
{
    J0.setZero();
    J1.setZero();
    J0Minv.setZero();
    J1Minv.setZero();
    lambda.setZero();
    phi.setZero();
    phi(0) = _pene;

    // Track list of contacts for each body.
    // This makes it easy to find coupled constraints and contacts during the solve.
    //
    // Note: Unlike joints, which have the list maintained by RigidBodySystem::addJoint()
    //  here we update the list in the constructor since contacts are determined during simulation.
    //
    body0->contacts.push_back(this);
    body1->contacts.push_back(this);
}

Contact::~Contact()
{

}

void Contact::computeJacobian()
{
    // TODO Objective #5
    // Compute the Jacobian for a contact constraint.

    // Compute the contact frame, which consists of an orthonormal basis formed by the vectors n, t, and b.
    t = n.unitOrthogonal(); // Compute the first tangent direction t.
    b = n.cross(t);         // Compute the second tangent direction b.

    // Initialize the Jacobian blocks J0 and J1.
    J0.setZero(3, 6);
    J1.setZero(3, 6);

    // Compute the Jacobian for body0
    if (body0)
    {
        Eigen::Matrix3f skewSymmetric = Eigen::Matrix3f::Zero();
        skewSymmetric(0, 1) = -p[2];
        skewSymmetric(0, 2) = p[1];
        skewSymmetric(1, 0) = p[2];
        skewSymmetric(1, 2) = -p[0];
        skewSymmetric(2, 0) = -p[1];
        skewSymmetric(2, 1) = p[0];

        J0.block<1, 3>(0, 0) = -n.transpose();
        J0.block<1, 3>(1, 0) = -t.transpose();
        J0.block<1, 3>(2, 0) = -b.transpose();
        J0.block<1, 3>(0, 3) = -(skewSymmetric * n).transpose();
        J0.block<1, 3>(1, 3) = -(skewSymmetric * t).transpose();
        J0.block<1, 3>(2, 3) = -(skewSymmetric * b).transpose();
    }

    // Compute the Jacobian for body1
    if (body1)
    {
        Eigen::Matrix3f skewSymmetric = Eigen::Matrix3f::Zero();
        skewSymmetric(0, 1) = -p[2];
        skewSymmetric(0, 2) = p[1];
        skewSymmetric(1, 0) = p[2];
        skewSymmetric(1, 2) = -p[0];
        skewSymmetric(2, 0) = -p[1];
        skewSymmetric(2, 1) = p[0];

        J1.block<1, 3>(0, 0) = n.transpose();
        J1.block<1, 3>(1, 0) = t.transpose();
        J1.block<1, 3>(2, 0) = b.transpose();
        J1.block<1, 3>(0, 3) = (skewSymmetric * n).transpose();
        J1.block<1, 3>(1, 3) = (skewSymmetric * t).transpose();
        J1.block<1, 3>(2, 3) = (skewSymmetric * b).transpose();
    }

    // Compute the blocks J0Minv and J1Minv
    if (body0)
    {
        J0Minv.block<1, 3>(0, 0) = J0.block<1, 3>(0, 0) * (1.0f / body0->mass);
        J0Minv.block<1, 3>(1, 0) = J0.block<1, 3>(1, 0) * (1.0f / body0->mass);
        J0Minv.block<1, 3>(2, 0) = J0.block<1, 3>(2, 0) * (1.0f / body0->mass);
        J0Minv.block<1, 3>(0, 3) = J0.block<1, 3>(0, 3) * body0->Iinv;
        J0Minv.block<1, 3>(1, 3) = J0.block<1, 3>(1, 3) * body0->Iinv;
        J0Minv.block<1, 3>(2, 3) = J0.block<1, 3>(2, 3) * body0->Iinv;
    }

    if (body1)
    {
        J1Minv.block<1, 3>(0, 0) = J1.block<1, 3>(0, 0) * (1.0f / body1->mass);
        J1Minv.block<1, 3>(1, 0) = J1.block<1, 3>(1, 0) * (1.0f / body1->mass);
        J1Minv.block<1, 3>(2, 0) = J1.block<1, 3>(2, 0) * (1.0f / body1->mass);
        J1Minv.block<1, 3>(0, 3) = J1.block<1, 3>(0, 3) * body1->Iinv;
        J1Minv.block<1, 3>(1, 3) = J1.block<1, 3>(1, 3) * body1->Iinv;
        J1Minv.block<1, 3>(2, 3) = J1.block<1, 3>(2, 3) * body1->Iinv;
    }
}
