using System.Collections;
using System.Collections.Generic;
using Halodi.Physics.Interfaces;
using UnityEngine;

namespace Halodi.Physics.PhysX
{
    [RequireComponent(typeof(ArticulationBody))]
    [RequireComponent(typeof(ArticulationJoint))]
    public abstract class PhysxArticulationJoint : MonoBehaviour, IJointPhysics
    {
        private static float MinimumMass = 0.02f;

        private ArticulationBody body_ = null;

        public ArticulationBody body
        {
            get
            {
                if (body_ == null)
                {
                    body_ = GetComponent<ArticulationBody>();
                }
                return body_;
            }
        }

        

        protected void CreateBody(ArticulationJoint joint)
        {
            if (body == null)
            {
                gameObject.AddComponent<ArticulationBody>();
            }


            body.useGravity = true;

            // Disable damping and friction
            body.linearDamping = 0;
            body.angularDamping = 0;
            body.jointFriction = 0;

            if (joint.mass > MinimumMass)
            {
                body.mass = joint.mass;
                body.centerOfMass = joint.centerOfMass;
                body.inertiaTensor = joint.inertiaTensor;
                body.inertiaTensorRotation = joint.inertiaTensorRotation;
            }
            else
            {
                body.mass = MinimumMass;
                body.centerOfMass = joint.centerOfMass;
                body.inertiaTensor = (MinimumMass / joint.mass) * joint.inertiaTensor;
                body.inertiaTensorRotation = joint.inertiaTensorRotation;
            }

            body.anchorPosition = joint.anchorPosition;
            body.anchorRotation = joint.anchorRotation;
            body.parentAnchorPosition = joint.parentAnchorPosition;
            body.parentAnchorRotation = joint.parentAnchorRotation;
        }

    }
}
