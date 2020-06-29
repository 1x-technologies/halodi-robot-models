using System.Collections;
using System.Collections.Generic;
using Halodi.Physics.Interfaces;
using UnityEngine;

namespace Halodi.Physics
{

    public class ArticulationRevoluteJoint : ArticulationOneDOFJoint
    {

        public bool continous;


        public override void SetKinematicPosition(float angle)
        {
            Quaternion desiredRotation = Quaternion.Euler(Mathf.Rad2Deg * angle, 0.0f, 0.0f);

            Quaternion newRotation = parentAnchorRotation * desiredRotation * Quaternion.Inverse(anchorRotation);
            Vector3 newPosition = parentAnchorPosition + newRotation * (-1.0f * anchorPosition);

            transform.localPosition = newPosition;
            transform.localRotation = newRotation;
        }

        protected override void SetPhysics(IPhysicsEngine physicsInterface)
        {
            oneDOFJointPhysics = physicsInterface.AddRevoluteJoint(this);
        }
    }

}