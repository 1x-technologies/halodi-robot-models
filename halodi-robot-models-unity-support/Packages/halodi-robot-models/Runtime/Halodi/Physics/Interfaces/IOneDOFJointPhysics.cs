using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Halodi.Physics.Interfaces
{
    public interface IOneDOFJointPhysics : IJointPhysics
    {
        float jointPosition { get; set; }
        float jointVelocity { get; set; }
        float jointAcceleration { get; set; }

        float jointEffort { get; set; }
        float stiffness { set; }

        float damping { set; }

        float desiredPosition { set; }
        float desiredVelocity { set; }
    }

}