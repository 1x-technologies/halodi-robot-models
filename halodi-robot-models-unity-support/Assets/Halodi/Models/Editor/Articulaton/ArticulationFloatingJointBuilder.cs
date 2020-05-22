using System.Collections;
using System.Collections.Generic;
using Halodi.Physics;
using RosSharp.Urdf;
using UnityEngine;

namespace Halodi.Models.Editor.Articulation
{
    public class ArticulationFloatingJointBuilder
    {

        public static ArticulationFloatingJoint Create(UrdfLink urdfLink)
        {
            GameObject link = urdfLink.gameObject;
            Rigidbody rigidbody = link.GetComponent<Rigidbody>();
            if (rigidbody == null)
            {
                Debug.LogError("Cannot create floating joint for link " + link.name);
            }

            ArticulationFloatingJoint floatingJoint = ArticulationBodyBuilder.Create<ArticulationFloatingJoint>(link);
            floatingJoint.JointName = link.name;

            return floatingJoint;
        }

    }
}