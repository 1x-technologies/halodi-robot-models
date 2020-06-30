using System.Collections;
using System.Collections.Generic;
using Halodi.Physics.Interfaces;
using UnityEngine;

namespace Halodi.Physics
{
    public class ArticulationFixedJoint : ArticulationJoint
    {
        public override bool isRoot => false;

        public override IJointPhysics physics => fixedJointPhysics;

        public IFixedJointPhysics fixedJointPhysics = null;

       
        protected override void SetPhysics(IPhysicsEngine physicsInterface)
        {
            fixedJointPhysics = physicsInterface.AddFixedJoint(this);
        }

    }

}