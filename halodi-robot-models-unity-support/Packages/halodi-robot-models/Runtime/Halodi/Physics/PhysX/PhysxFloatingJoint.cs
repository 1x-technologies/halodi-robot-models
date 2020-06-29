using System;
using System.Collections;
using System.Collections.Generic;
using Halodi.Physics.Interfaces;
using UnityEngine;

namespace Halodi.Physics.PhysX
{
    public class PhysxFloatingJoint : PhysxArticulationJoint, IFloatingJointPhysics
    {
        internal PhysxFloatingJoint SetupJoint(ArticulationFloatingJoint joint)
        {
            CreateBody(joint);
            body.immovable = joint.immovable;
            return this;
        }
    }
}