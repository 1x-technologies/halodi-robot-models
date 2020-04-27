using System.Collections;
using System.Collections.Generic;
using UnityEngine;


#if !UNITY_2020_1_OR_NEWER

namespace Halodi.Physics
{
    /// <summary>
    /// This interface implements a minimal interface mimicking ArticulationBody to allow compilation for Unity version before 2020.
    /// </summary>
    internal class ArticulationBody : MonoBehaviour
    {
        internal bool useGravity;
        internal float linearDamping;
        internal float angularDamping;
        internal float jointFriction;
        internal float mass;
        internal Vector3 centerOfMass;
        internal Vector3 inertiaTensor;
        internal Quaternion inertiaTensorRotation;
        internal Vector3 anchorPosition;
        internal Quaternion anchorRotation;
        internal Vector3 parentAnchorPosition;
        internal Quaternion parentAnchorRotation;
        internal bool isRoot = false;
        internal ArticulationJointType jointType;
        internal ArticulationDofLock linearLockX;
        internal ArticulationDofLock linearLockY;
        internal ArticulationDofLock linearLockZ;
        internal ArticulationDofLock twistLock;
        internal ArticulationDrive xDrive;

        internal ArticulationReducedSpace jointForce { get; set; }

        internal ArticulationReducedSpace jointPosition { get; set; }
        internal ArticulationReducedSpace jointVelocity { get; set; }
        internal ArticulationReducedSpace jointAcceleration { get; set; }
        

    }


    public enum ArticulationJointType
    {
        PrismaticJoint,
        FixedJoint,
        RevoluteJoint
    }

    public enum ArticulationDofLock
    {
        LimitedMotion,
        LockedMotion,
        FreeMotion
    }

    public struct ArticulationReducedSpace
    {
        public ArticulationReducedSpace(float value)
        {

        }

        public float this[int index] {
            get
            {
                return 0.0f;
            }
            set 
            {

            }
        }
    }

    public struct ArticulationDrive
    {
        internal float damping;
        internal float stiffness;
        internal float forceLimit;
        internal float lowerLimit;
        internal float upperLimit;
    }

}

#endif