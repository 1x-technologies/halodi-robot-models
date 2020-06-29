using System.Collections;
using System.Collections.Generic;
using Halodi.Physics.Interfaces;
using UnityEngine;

namespace Halodi.Physics.PhysX
{
    public class PhysxPrismaticJoint : PhysxOneDOFJoint, IPrismaticJointPhysics
    {
        
        internal PhysxPrismaticJoint SetupJoint(ArticulationPrismaticJoint joint)
        {
            CreateBody(joint);

            body.jointType = ArticulationJointType.PrismaticJoint;

            body.linearLockX = ArticulationDofLock.LimitedMotion;
            body.linearLockY = ArticulationDofLock.LockedMotion;
            body.linearLockZ = ArticulationDofLock.LockedMotion;
            body.twistLock = ArticulationDofLock.LockedMotion;
        
            ArticulationDrive xDrive = new ArticulationDrive
            {
                damping = 0.0f,
                stiffness = 0.0f,
                forceLimit = float.MaxValue,
                lowerLimit = joint.lowerLimit,
                upperLimit = joint.upperLimit
            };

            body.xDrive = xDrive;

            return this;
        }
    }
}