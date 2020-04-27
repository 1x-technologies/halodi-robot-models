using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Halodi.Physics
{
    [CustomEditor(typeof(ArticulationOneDOFJoint), true)]
    public class ArticulationOneDOFJointEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            ArticulationOneDOFJoint manager = (ArticulationOneDOFJoint)target;




            EditorGUILayout.LabelField("Is kinematic: ", manager.kinematic.ToString());


            if (Application.isPlaying)
            {
                if (manager.kinematic)
                {
                    manager.position = EditorGUILayout.FloatField("q: ", manager.position);
                }
                else
                {
                    EditorGUILayout.LabelField("q: ", manager.position.ToString());
                }
                EditorGUILayout.LabelField("qd: ", manager.velocity.ToString());
                EditorGUILayout.LabelField("qdd: ", manager.acceleration.ToString());


                if (manager.kinematic)
                {
                    EditorGUILayout.LabelField("tau: ", manager.tau.ToString());
                }
                else
                {
                    manager.tau = EditorGUILayout.FloatField("tau: ", manager.tau);
                }

            }
            else
            {
                EditorGUILayout.LabelField("q: ", "-");
                EditorGUILayout.LabelField("qd: ", "-");
                EditorGUILayout.LabelField("qdd: ", "-");
                EditorGUILayout.LabelField("tau: ", "-");
            }

        }
    }
}