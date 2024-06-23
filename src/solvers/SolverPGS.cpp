#include "solvers/SolverPGS.h"

#include "contact/Contact.h"
#include "joint/Joint.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <Eigen/Dense>


namespace
{
    // TODO 
    // Solve the Boxed LCP problem for a single contact and isotropic Coulomb friction.
    // The solution vector, @a x, contains the impulse the non-interpenetration constraint in x(0), and
    // the friction constraints in x(1) and x(2)
    // 
    // The solution of the non-interpenetration impulses must be positive.
    // The solution of friction must projected to the lower and upper bounds imposed by the box model.
    // 
    // Inputs: 
    //    A - diagonal block of the global JMinvJT matrix corresponding to this contact
    //    x - contains three impulse variables (non-interpenetration + two friction)
    //    b - updated rhs vector minus contributions of coupled joints and contacts
    //    mu - the friction coefficient
    //
    static inline void solveContact(const Eigen::Matrix3f& A, const Eigen::Vector3f& b, Eigen::Vector3f& x, const float mu)
    {
        Eigen::Vector3f new_x = A.ldlt().solve(b);

        // Projeter les impulsions de friction dans les bornes du modèle de boîte
        float friction_magnitude = std::sqrt(new_x[1] * new_x[1] + new_x[2] * new_x[2]);
        if (friction_magnitude > mu * new_x[0])
        {
            new_x[1] *= mu * new_x[0] / friction_magnitude;
            new_x[2] *= mu * new_x[0] / friction_magnitude;
        }

        // Les impulsions de non-interpénétration doivent être positives
        new_x[0] = std::max(0.0f, new_x[0]);

        x = new_x;
    }

    // TODO 
    // Solve for the constraint impluses of a bilateral constraint.
    // The solution vector, @a x, contains the impulse the non-interpenetration constraint in x(0), and
    // the friction constraints in x(1) and x(2)
    // 
    // The solution of the non-interpenetration impulses must be positive.
    // The solution of friction must projected to the lower and upper bounds imposed by the box model.
    // 
    // Inputs: 
    //    A - diagonal block of the global JMinvJT matrix corresponding to this joint
    //    x - contains the constraint impulses (lambda)
    //    b - updated rhs vector minus contributions of coupled joints and contacts
    //
    template <typename VectorBlockType>
    static inline void solveJoint(const Eigen::MatrixXf& A, const Eigen::VectorXf& b, VectorBlockType& x)
    {
        x = A.ldlt().solve(b);
    }
}

SolverPGS::SolverPGS(RigidBodySystem* _rigidBodySystem) : Solver(_rigidBodySystem)
{

}

void SolverPGS::solve(float h)
{
        std::vector<Contact>& contacts = m_rigidBodySystem->getContacts();
    std::vector<Joint*>& joints = m_rigidBodySystem->getJoints();
    const int numContacts = contacts.size();
    const int numJoints = joints.size();

    // Build diagonal matrices for joints
    if (numJoints > 0)
    {
        Ajoint.resize(numJoints);
        for (int i = 0; i < numJoints; ++i)
        {
            Joint* j = joints[i];
            const int dim = j->dim;
            const float eps = 1e-6f;

            Eigen::MatrixXf A = eps * Eigen::MatrixXf::Identity(dim, dim);
            if (!j->body0->fixed)
            {
                A += j->J0Minv.block(0, 0, dim, 6) * j->J0.block(0, 0, dim, 6).transpose();
            }
            if (!j->body1->fixed)
            {
                A += j->J1Minv.block(0, 0, dim, 6) * j->J1.block(0, 0, dim, 6).transpose();
            }
            Ajoint[i] = A;
        }
    }

    // Build diagonal matrices for contacts
    if (numContacts > 0)
    {
        Acontact.resize(numContacts);
        for (int i = 0; i < numContacts; ++i)
        {
            Contact* c = &contacts[i];
            Acontact[i].setZero();
            if (!c->body0->fixed)
            {
                Acontact[i] += c->J0Minv * c->J0.transpose();
            }
            if (!c->body1->fixed)
            {
                Acontact[i] += c->J1Minv * c->J1.transpose();
            }
        }
    }

    // Compute right-hand side vector for contacts
    bcontact.resize(numContacts);
    for (int i = 0; i < numContacts; ++i)
    {
        Contact* c = &contacts[i];
        c->lambda.setZero();
        bcontact[i] = -c->gamma * c->phi / h - c->J * c->vel - h * c->JMinvJT * c->force;
    }

    // Compute right-hand side vector for joints
    bjoint.resize(numJoints);
    for (int i = 0; i < numJoints; ++i)
    {
        Joint* j = joints[i];
        bjoint[i] = -j->gamma * j->phi / h - j->J * j->vel - h * j->JMinvJT * j->force;
    }

    // PGS main loop
    for(int iter = 0; iter < m_maxIter; ++iter)
    {
        // Solve for joint impulses
        for (int i = 0; i < numJoints; ++i)
        {
            Joint* j = joints[i];
            solveJoint(Ajoint[i], bjoint[i], j->lambda);
        }

        // Solve for contact impulses
        for(int i = 0; i < numContacts; ++i)
        {
            Contact* c = &contacts[i];
            solveContact(Acontact[i], bcontact[i], c->lambda, c->mu);
        }
    }
}


