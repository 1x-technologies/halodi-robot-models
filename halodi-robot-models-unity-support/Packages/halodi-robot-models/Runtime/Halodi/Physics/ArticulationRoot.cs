using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Halodi.Physics
{
    /// <summary>
    /// Script to manage the root of a articulation path.
    /// 
    /// </summary>

    public class ArticulationRoot : MonoBehaviour
    {

        public void SetKinematic(bool kinematic)
        {
            ArticulationFloatingJoint[] rootJoints = GetComponentsInChildren<ArticulationFloatingJoint>();

            foreach (var rootJoint in rootJoints)
            {
                rootJoint.kinematic = kinematic;               
            }
        }

        public ArticulationOneDOFJoint GetJointByName(string name)
        {
            ArticulationOneDOFJoint[] joints = GetComponentsInChildren<ArticulationOneDOFJoint>();

            foreach (var joint in joints)
            {
                if(joint.JointName.Equals(name))
                {
                    return joint;
                }
            }

            return null;
        }
    }

}