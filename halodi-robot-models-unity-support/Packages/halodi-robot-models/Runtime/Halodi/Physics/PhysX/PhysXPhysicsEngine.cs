using System.Collections;
using System.Collections.Generic;
using Halodi.Physics.Interfaces;
using UnityEngine;

namespace Halodi.Physics.PhysX
{
    public class PhysXPhysicsEngine : IPhysicsEngine
    {

        private T CreateJoint<T>(ArticulationJoint joint) where T : PhysxArticulationJoint
        {
            T physics = joint.gameObject.AddComponent<T>();
            return physics;
        }

        public override IFixedJointPhysics AddFixedJoint(ArticulationFixedJoint joint)
        {
            return CreateJoint<PhysxFixedJoint>(joint).SetupJoint(joint);
        }

        public override IFloatingJointPhysics AddFloatingJoint(ArticulationFloatingJoint joint)
        {
            return CreateJoint<PhysxFloatingJoint>(joint).SetupJoint(joint);
        }

        public override IPrismaticJointPhysics AddPrismaticJoint(ArticulationPrismaticJoint joint)
        {
            return CreateJoint<PhysxPrismaticJoint>(joint).SetupJoint(joint);
        }

        public override IRevoluteJointPhysics AddRevoluteJoint(ArticulationRevoluteJoint joint)
        {
            return CreateJoint<PhysxRevoluteJoint>(joint).SetupJoint(joint);
        }
    }

}