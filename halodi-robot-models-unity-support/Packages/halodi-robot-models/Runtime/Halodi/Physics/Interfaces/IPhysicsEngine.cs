using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Halodi.Physics.Interfaces
{
    public abstract class IPhysicsEngine : MonoBehaviour
    {
        public abstract IRevoluteJointPhysics AddRevoluteJoint(ArticulationRevoluteJoint joint);
        public abstract IPrismaticJointPhysics AddPrismaticJoint(ArticulationPrismaticJoint joint);

        public abstract IFixedJointPhysics AddFixedJoint(ArticulationFixedJoint joint);

        public abstract IFloatingJointPhysics AddFloatingJoint(ArticulationFloatingJoint joint);

        public abstract void Simulate(float step);

    }

}