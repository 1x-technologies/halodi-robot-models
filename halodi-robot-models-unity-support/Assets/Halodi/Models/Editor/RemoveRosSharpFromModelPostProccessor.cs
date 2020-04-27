using System.Collections;
using System.Collections.Generic;
using RosSharp;
using RosSharp.Urdf;
using UnityEngine;

namespace Halodi.Models.Editor
{
    public class RemoveRosSharpFromModelPostProccessor
    {

        internal static void PostProccess(string robotName, GameObject root)
        {

            DestroyComponents<UrdfLink>(root);

            DestroyComponents<UrdfVisuals>(root);
            DestroyComponents<UrdfVisual>(root);

            DestroyComponents<UrdfCollisions>(root);
            DestroyComponents<UrdfCollision>(root);

            DestroyComponents<UrdfPlugin>(root);
            DestroyComponents<UrdfPlugins>(root);



            Object.DestroyImmediate(root.GetComponent<UrdfRobot>());
        }

        public static void DestroyComponents<T>(GameObject root) where T : Component
        {
            T[] components = root.GetComponentsInChildren<T>();
            foreach (T component in components)
            {
                Object.DestroyImmediate(component);
            }
        }
    }
}