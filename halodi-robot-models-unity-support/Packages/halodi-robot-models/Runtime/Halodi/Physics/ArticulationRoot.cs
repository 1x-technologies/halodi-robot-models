using System.Collections;
using System.Collections.Generic;
using Halodi.Physics.Interfaces;
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

        public void Awake()
        {
            IPhysicsEngine physicsEngine = GetComponent<IPhysicsEngine>();

            if (physicsEngine != null)
            {
                ArticulationFloatingJoint[] rootJoints = GetComponentsInChildren<ArticulationFloatingJoint>();

                foreach (var rootJoint in rootJoints)
                {
                    rootJoint.PhysicsEngine = physicsEngine;
                }
            }
        }


        public ArticulationOneDOFJoint GetJointByName(string name)
        {
            ArticulationOneDOFJoint[] joints = GetComponentsInChildren<ArticulationOneDOFJoint>();

            foreach (var joint in joints)
            {
                if (joint.JointName.Equals(name))
                {
                    return joint;
                }
            }

            return null;
        }
    }

}