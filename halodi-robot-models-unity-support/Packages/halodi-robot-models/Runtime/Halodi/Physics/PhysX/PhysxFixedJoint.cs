using System;
using System.Collections;
using System.Collections.Generic;
using Halodi.Physics.Interfaces;
using UnityEngine;

namespace Halodi.Physics.PhysX
{
    public class PhysxFixedJoint : PhysxArticulationJoint, IFixedJointPhysics
    {
        internal PhysxFixedJoint SetupJoint(ArticulationFixedJoint joint)
        {
            CreateBody(joint);

            body.jointType = ArticulationJointType.FixedJoint;
            
            return this;
        }
    }
}