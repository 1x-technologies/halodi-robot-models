using System.Collections;
using System.Collections.Generic;
using Halodi.Physics.Interfaces;
using UnityEngine;

namespace Halodi.Physics.PhysX
{
    public abstract class PhysxOneDOFJoint : PhysxArticulationJoint, IOneDOFJointPhysics
    {
        public float jointPosition
        {
            get
            {
                return body.jointPosition[0];
            }
            set
            {
                body.jointPosition = new ArticulationReducedSpace(value);
            }
        }
        public float jointVelocity
        {
            get
            {
                return body.jointVelocity[0];
            }
            set
            {
                body.jointVelocity = new ArticulationReducedSpace(value);
            }
        }
        public float jointAcceleration
        {
            get
            {
                return body.jointAcceleration[0];
            }
            set
            {
                body.jointAcceleration = new ArticulationReducedSpace(value);
            }
        }
        public float jointEffort
        {
            get
            {
                return body.jointForce[0];
            }
            set
            {
                body.jointForce = new ArticulationReducedSpace(value);
            }
        }

        public float stiffness
        {
            set
            {
                var xDrive = body.xDrive;
                xDrive.stiffness = value;
                body.xDrive = xDrive;
            }
        }

        public float damping
        {
            set
            {
                var xDrive = body.xDrive;
                xDrive.damping = value;
                body.xDrive = xDrive;
            }
        }
        public float desiredPosition
        {
            set
            {
                var xDrive = body.xDrive;
                xDrive.target = value;
                body.xDrive = xDrive;
            }

        }
        public float desiredVelocity
        {
            set
            {
                var xDrive = body.xDrive;
                xDrive.targetVelocity = value;
                body.xDrive = xDrive;
            }
        }
    }
}