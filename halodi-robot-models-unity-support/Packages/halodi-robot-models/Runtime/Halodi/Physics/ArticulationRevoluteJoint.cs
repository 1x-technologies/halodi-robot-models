using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Halodi.Physics
{

    public class ArticulationRevoluteJoint : ArticulationOneDOFJoint
    {

        public bool continous;


        protected override void setPosition(float angle)
        {
            Quaternion desiredRotation = Quaternion.Euler(Mathf.Rad2Deg * angle, 0.0f, 0.0f);
            
            Quaternion newRotation = parentAnchorRotation * desiredRotation * Quaternion.Inverse(anchorRotation);
            Vector3 newPosition = parentAnchorPosition + newRotation * (-1.0f * anchorPosition);

            transform.localPosition = newPosition;
            transform.localRotation = newRotation;
        }

        protected override void SetupJoint(ArticulationBody articulationBody)
        {
            
            articulationBody.jointType = ArticulationJointType.RevoluteJoint;

            ArticulationDrive xDrive = new ArticulationDrive
            {
                damping = 0.0f,
                stiffness = 0.0f,
                forceLimit = float.MaxValue
            };

            if(continous)
            {
                xDrive.lowerLimit = float.MinValue;
                xDrive.upperLimit = float.MaxValue;
                articulationBody.twistLock = ArticulationDofLock.FreeMotion;
            }
            else
            {
                xDrive.lowerLimit = lowerLimit;
                xDrive.upperLimit = upperLimit;
                articulationBody.twistLock = ArticulationDofLock.LimitedMotion;
                
            }


            articulationBody.xDrive = xDrive;
        }     
    }

}