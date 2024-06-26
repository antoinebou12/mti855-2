#pragma once

#include <Eigen/Dense>
#include <vector>

class Contact;
class RigidBody;
class RigidBodySystem;

//
// The main collision detection class,
// it will test the geometries of each pair of rigid bodies
// and populate an array with contacts.
//
class CollisionDetect
{
public:

    // Constructor.
    //
    CollisionDetect(RigidBodySystem* rigidBodySystem);

    // Tests for collisions between all pairs of bodies in the rigid body system
    // and generates contacts for any intersecting geometries.
    // The array of generated contacts can be retrieved by calling getContacts().
    //
    void detectCollisions();

    void clear();

    // Compute the Jacobians for contacts.
    void computeContactJacobians();

    // Returns all contacts following the current collision detection pass (read-only).
    const std::vector<Contact>& getContacts() const { return m_contacts; }

    // Returns all contacts following the current collision detection pass.
    std::vector<Contact>& getContacts() { return m_contacts; }

private:

    explicit CollisionDetect();

    // Sphere-sphere collision test.
    // Assumes that both @a body0 and @a body1 have a Sphere collision geometry.
    //
    void collisionDetectSphereSphere(RigidBody* body0, RigidBody* body1);

    // Sphere-box collision test.
    // Assumes that @a body0 has sphere geometry, and @a body1 has Box geometry.
    //
    void collisionDetectSphereBox(RigidBody* body0, RigidBody* body1);

    // Sphere-plane collision test.
    // Assumes that @a body0 has sphere geometry, and @a body1 has Plane geometry.
    //
    void collisionDetectSpherePlane(RigidBody* body0, RigidBody* body1);

    // Cylinder-plane collision test
    // Assumes that @a body0 has Cylinder geometry, and @a body1 has Plane geometry.
    //
    void collisionDetectCylinderPlane(RigidBody* body0, RigidBody* body1);

    // Cylinder-sphere collision
    // Assumes that @a body0 has Cylinder geometry, and @a body1 has Sphere geometry.
    //
    void collisionDetectCylinderSphere(RigidBody* body0, RigidBody* body1);

    // Cylinder-sphere collision
    // Assumes that @a body0 has Box geometry, and @a body1 has Plane geometry.
    //
    void collisionDetectBoxPlane(RigidBody* body0, RigidBody* body1);


private:

    RigidBodySystem* m_rigidBodySystem;                 // The rigid body system.
    std::vector<Contact> m_contacts;                    // Cached array of contacts.
    size_t m_maxContacts;

};
