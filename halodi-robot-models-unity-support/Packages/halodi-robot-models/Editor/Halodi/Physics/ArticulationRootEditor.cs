using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Halodi.Physics
{
    [CustomEditor(typeof(ArticulationRoot))]
    public class ArticulationRootEditor : UnityEditor.Editor
    {

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            ArticulationRoot root = (ArticulationRoot)target;

#if UNITY_2020_1_OR_NEWER

            if (GUILayout.Button("Set kinematic"))
            {
                root.SetKinematic(true);
            }

            if (GUILayout.Button("Set dynamic"))
            {
                root.SetKinematic(false);
            }
#endif            
            
        }
    }

}