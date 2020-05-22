using System.Collections;
using System.Collections.Generic;
using Halodi.Physics;
using RosSharp.Urdf;
using UnityEngine;


namespace Halodi.Models.Editor.Articulation
{
    public class ArticulationBodyBuilder
    {



        public static T Create<T>(GameObject link) where T : ArticulationJoint
        {
            UrdfLink urdfLink = link.GetComponent<UrdfLink>();

            T joint = link.AddComponent<T>();



            UrdfInertial inertial = link.GetComponent<UrdfInertial>();

            if (inertial != null)
            {
                joint.centerOfMass = inertial.CenterOfMass;
                joint.inertiaTensor = inertial.InertiaTensor;
                joint.inertiaTensorRotation = inertial.InertiaTensorRotation;

            }

            Rigidbody rigidbody = link.GetComponent<Rigidbody>();
            if (rigidbody != null)
            {
                joint.mass = rigidbody.mass;
            }

            return joint;
        }

    }
}
