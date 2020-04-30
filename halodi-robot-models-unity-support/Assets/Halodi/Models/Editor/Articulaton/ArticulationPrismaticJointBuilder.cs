using System.Collections;
using System.Collections.Generic;
using Halodi.Physics;
using RosSharp;
using RosSharp.Urdf;
using UnityEngine;
using static RosSharp.Urdf.UrdfJoint;

namespace Halodi.Models.Editor.Articulation
{
    public class ArticulationPrismaticJointBuilder
    {
        public static ArticulationPrismaticJoint Create(UrdfJoint urdfJoint)
        {
            if (urdfJoint.JointType != JointTypes.Prismatic)
            {
                Debug.LogError("Cannot create joint " + urdfJoint.JointName + ". Invalid type " + urdfJoint.JointType);
                return null;
            }



            GameObject link = urdfJoint.gameObject;

            var configurableJoint = link.GetComponent<ConfigurableJoint>();
            Rigidbody rigidbody = link.GetComponent<Rigidbody>();
            if (configurableJoint == null || rigidbody == null)
            {
                Debug.LogError("Cannot create joint " + urdfJoint.JointName);
                return null;
            }

            ArticulationPrismaticJoint joint = ArticulationBodyBuilder.Create<ArticulationPrismaticJoint>(link);
            joint.JointName = urdfJoint.JointName;

            PrismaticJointLimitsManager prismaticJointLimitsManager = link.GetComponent<PrismaticJointLimitsManager>();

            float lowerLimit, upperLimit;
            if (prismaticJointLimitsManager != null)
            {
                lowerLimit = prismaticJointLimitsManager.PositionLimitMin;
                upperLimit = prismaticJointLimitsManager.PositionLimitMax;

            }
            else
            {
                lowerLimit = float.MinValue;
                upperLimit = float.MaxValue;
            }

            joint.lowerLimit = lowerLimit;
            joint.upperLimit = upperLimit;


            joint.axis = configurableJoint.axis;
            joint.anchorPosition = configurableJoint.anchor;
            joint.anchorRotation = Quaternion.FromToRotation(Vector3.right, configurableJoint.axis.normalized);
            joint.UpdateParentAnchor();




            return joint;
        }

    }
}