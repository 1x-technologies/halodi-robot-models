using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Halodi.Physics
{
    

    public class ArticulationPrismaticJoint : ArticulationOneDOFJoint
    {
        

        public override void SetKinematicPosition(float position)
        {
            // Prismatic joints always move in the X direction(!)
            Vector3 desiredPosition = new Vector3(position, 0.0f, 0.0f);
            
            Quaternion newRotation = parentAnchorRotation * Quaternion.Inverse(anchorRotation);
            Vector3 newPosition = parentAnchorPosition + parentAnchorRotation * desiredPosition + newRotation * (-1.0f * anchorPosition);

            transform.localRotation = newRotation;
            transform.localPosition = newPosition;
        }

        protected override void SetupJoint(ArticulationBody body)
        {
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
                lowerLimit = lowerLimit,
                upperLimit = upperLimit
            };

            body.xDrive = xDrive;
        }
    }

}