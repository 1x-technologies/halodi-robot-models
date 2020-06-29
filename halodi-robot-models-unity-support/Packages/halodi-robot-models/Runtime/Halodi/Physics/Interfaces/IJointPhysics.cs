using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Halodi.Physics.Interfaces
{
    public interface IJointPhysics
    {
        Vector3 GetPointVelocity(Vector3 position);
        void AddForceAtPosition(Vector3 force, Vector3 position);

        Vector3 inertiaTensor
        {
            set;
        }

        Quaternion inertiaTensorRotation
        {
            set;
        }

        void TeleportRoot(Vector3 position, Quaternion rotation);
    }
}