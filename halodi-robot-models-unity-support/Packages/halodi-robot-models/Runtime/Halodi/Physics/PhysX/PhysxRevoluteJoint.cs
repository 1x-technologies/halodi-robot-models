using System.Collections;
using System.Collections.Generic;
using Halodi.Physics.Interfaces;
using UnityEngine;

namespace Halodi.Physics.PhysX
{
    public class PhysxRevoluteJoint : PhysxOneDOFJoint, IRevoluteJointPhysics
    {
        
        internal PhysxRevoluteJoint SetupJoint(ArticulationRevoluteJoint joint)
        {
            CreateBody(joint);
            
            body.jointType = ArticulationJointType.RevoluteJoint;

            ArticulationDrive xDrive = new ArticulationDrive
            {
                damping = 0.0f,
                stiffness = 0.0f,
                forceLimit = float.MaxValue
            };

            if(joint.continous)
            {
                xDrive.lowerLimit = float.MinValue;
                xDrive.upperLimit = float.MaxValue;
                body.twistLock = ArticulationDofLock.FreeMotion;
            }
            else
            {
                xDrive.lowerLimit = joint.lowerLimit;
                xDrive.upperLimit = joint.upperLimit;
                body.twistLock = ArticulationDofLock.LimitedMotion;
                
            }


            body.xDrive = xDrive;

            return this;
        }  
    }
}