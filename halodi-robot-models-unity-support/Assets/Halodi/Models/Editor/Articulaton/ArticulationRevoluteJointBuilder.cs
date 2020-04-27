using System.Collections;
using System.Collections.Generic;
using Halodi.Physics;
using RosSharp;
using RosSharp.Urdf;
using UnityEngine;
using static RosSharp.Urdf.UrdfJoint;

namespace Halodi.Models.Editor.Articulation
{
    public class ArticulationRevoluteJointBuilder
    {
        public static ArticulationRevoluteJoint Create(UrdfJoint urdfJoint)
        {
            if (urdfJoint.JointType != JointTypes.Revolute && urdfJoint.JointType != JointTypes.Continuous)
            {
                Debug.LogError("Cannot create joint " + urdfJoint.JointName + ". Invalid type " + urdfJoint.JointType);
                return null;
            }



            GameObject link = urdfJoint.gameObject;

            var hingeJoint = link.GetComponent<HingeJoint>();
            Rigidbody rigidbody = link.GetComponent<Rigidbody>();
            if (hingeJoint == null || rigidbody == null)
            {
                Debug.LogError("Cannot create joint " + urdfJoint.JointName);
                return null;
            }

            ArticulationRevoluteJoint joint = ArticulationBodyBuilder.Create<ArticulationRevoluteJoint>(link);
            joint.JointName = urdfJoint.JointName;

            HingeJointLimitsManager hingeJointLimitsManager = link.GetComponent<HingeJointLimitsManager>();

            float lowerLimit, upperLimit;
            if (hingeJointLimitsManager != null)
            {
                lowerLimit = hingeJointLimitsManager.LargeAngleLimitMin;
                upperLimit = hingeJointLimitsManager.LargeAngleLimitMax;

            }
            else
            {
                lowerLimit = float.MinValue;
                upperLimit = float.MaxValue;
            }

            if (urdfJoint.JointType == JointTypes.Continuous)
            {
                joint.continous = true;
            }
            else
            {
                joint.continous = false;
            }

            joint.angleLimitMin = lowerLimit;
            joint.angleLimitMax = upperLimit;



            joint.anchorPosition = hingeJoint.anchor;
            joint.anchorRotation = Quaternion.FromToRotation(Vector3.right, hingeJoint.axis.normalized);
            joint.UpdateParentAnchor();




            return joint;
        }

    }
}