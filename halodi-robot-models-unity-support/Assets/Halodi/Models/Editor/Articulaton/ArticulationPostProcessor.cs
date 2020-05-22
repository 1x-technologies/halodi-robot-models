using System.Collections;
using System.Collections.Generic;
using Halodi.Physics;
using RosSharp;
using RosSharp.Urdf;
using UnityEditor;
using UnityEngine;
using static RosSharp.Urdf.UrdfJoint;

namespace Halodi.Models.Editor.Articulation
{
    public class ArticulationPostProcessor
    {



        internal static void PostProcess(string robotName, GameObject robotRoot)
        {
            UrdfRobot urdfRobotScript = robotRoot.GetComponent<UrdfRobot>();

            // Switch to convex colliders
            urdfRobotScript.SetCollidersConvex(true);

            UrdfLink[] links = robotRoot.GetComponentsInChildren<UrdfLink>();
            List<GameObject> rootLinks = new List<GameObject>();
            foreach (UrdfLink link in links)
            {
                Transform parentTransform = link.transform.parent;
                UrdfLink parentLink = parentTransform.gameObject.GetComponent<UrdfLink>();

                if (parentLink == null)
                {
                    PostProcessRootLink(link);
                }
            }

            robotRoot.AddComponent<ArticulationRoot>();

            RemoveUnusedComponents(robotRoot);
        }





        private static void PostProcessRootLink(UrdfLink link)
        {
            ArticulationFloatingJoint rootJoint = ArticulationFloatingJointBuilder.Create(link);

            // Get all links from rootJoint
            UrdfLink[] links = rootJoint.GetComponentsInChildren<UrdfLink>();
            foreach (UrdfLink child in links)
            {
                CreateArticulationLink(child);
            }
        }


        private static void CreateArticulationLink(UrdfLink link)
        {
            UrdfJoint joint = link.GetComponent<UrdfJoint>();
            if (joint != null)
            {
                CreateArticulationJoint(joint);
            }

        }


        private static void CreateArticulationJoint(UrdfJoint joint)
        {
            switch (joint.JointType)
            {
                case JointTypes.Fixed:
                    ArticulationFixedJointBuilder.Create(joint);
                    break;
                case JointTypes.Continuous:
                case JointTypes.Revolute:
                    ArticulationRevoluteJointBuilder.Create(joint);
                    break;
                case JointTypes.Prismatic:
                    ArticulationPrismaticJointBuilder.Create(joint);
                    break;
                default:
                    Debug.LogError(joint.JointName + ": Unsupported joint type: " + joint.JointType);
                    break;
            }

        }


        private static void RemoveUnusedComponents(GameObject root)
        {

            RemoveRosSharpFromModelPostProccessor.DestroyComponents<HingeJointLimitsManager>(root);
            RemoveRosSharpFromModelPostProccessor.DestroyComponents<PrismaticJointLimitsManager>(root);

            RemoveRosSharpFromModelPostProccessor.DestroyComponents<UrdfInertial>(root);
            RemoveRosSharpFromModelPostProccessor.DestroyComponents<UrdfJoint>(root);



            RemoveRosSharpFromModelPostProccessor.DestroyComponents<HingeJoint>(root);
            RemoveRosSharpFromModelPostProccessor.DestroyComponents<ConfigurableJoint>(root);
            RemoveRosSharpFromModelPostProccessor.DestroyComponents<FixedJoint>(root);
            RemoveRosSharpFromModelPostProccessor.DestroyComponents<Rigidbody>(root);



        }



    }
}