using System.Collections;
using System.Collections.Generic;
using Halodi.Physics.Interfaces;
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

        protected override void SetPhysics(IPhysicsEngine physicsInterface)
        {
            oneDOFJointPhysics = physicsInterface.AddPrismaticJoint(this);            
        }
    }

}