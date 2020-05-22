using System.Collections;
using System.Collections.Generic;
using Halodi.Physics;
using RosSharp;
using RosSharp.Urdf;
using UnityEngine;
using static RosSharp.Urdf.UrdfJoint;

namespace Halodi.Models.Editor.Articulation
{
    public class ArticulationFixedJointBuilder
    {
        public static ArticulationFixedJoint Create(UrdfJoint urdfJoint)
        {
            if (urdfJoint.JointType != JointTypes.Fixed)
            {
                Debug.LogError("Cannot create joint " + urdfJoint.JointName + ". Invalid type " + urdfJoint.JointType);
                return null;
            }


            GameObject link = urdfJoint.gameObject;

            ArticulationFixedJoint joint = ArticulationBodyBuilder.Create<ArticulationFixedJoint>(link);
            joint.JointName = urdfJoint.JointName;

            var fixedJoint = link.GetComponent<FixedJoint>();
            Rigidbody rigidbody = link.GetComponent<Rigidbody>();
            if (fixedJoint == null || rigidbody == null)
            {
                Debug.LogError("Cannot create joint " + urdfJoint.JointName);
                return null;
            }

            joint.anchorPosition = fixedJoint.anchor;
            joint.anchorRotation = Quaternion.FromToRotation(Vector3.right, fixedJoint.axis.normalized);
            joint.UpdateParentAnchor();


            return joint;
        }

    }
}