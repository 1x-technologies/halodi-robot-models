using System;
using System.Collections;
using System.Collections.Generic;
using Halodi.Physics.Interfaces;
using UnityEngine;

namespace Halodi.Physics
{
    public class ArticulationFloatingJoint : ArticulationJoint
    {
        public override bool isRoot => true;

        public bool immovable = false;

        public override IJointPhysics physics => floatingJointPhysics;

        public IFloatingJointPhysics floatingJointPhysics = null;
        

        protected override void SetPhysics(IPhysicsEngine physicsInterface)
        {
            floatingJointPhysics = physicsInterface.AddFloatingJoint(this);
        }
    }
}