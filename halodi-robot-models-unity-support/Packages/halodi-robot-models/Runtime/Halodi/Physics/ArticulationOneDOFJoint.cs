using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Halodi.Physics
{
    public abstract class ArticulationOneDOFJoint : ArticulationJoint
    {
        public override bool isRoot => false;

        /// <summary>
        /// Joint position
        /// 
        /// Writeable only when in kinematic mode. If set in non-kinematic mode an error is raised.
        /// </summary>
        /// <value>Position [radian]</value>
        public float position
        {
            get
            {
                if(kinematic)
                {
                    return kinematicPosition;
                }
                else
                {
                    return body.jointPosition[0];
                }
            }
            set
            {
                if(kinematic)
                {
                    kinematicPosition = value;
                }
                else
                {
                    Debug.LogError("Joint is dynamic. Cannot set position.");
                }
            }
        }

        /// <summary>
        /// Current joint velocity.
        /// 
        /// In kinematic mode this field is mutable but has no effect on the physics.
        /// </summary>
        /// <value>Velocity [rad/s]</value>
        public float velocity
        {
            get
            {
                if(kinematic)
                {
                    return kinematicVelocity;
                }
                else
                {
                    return body.jointVelocity[0];
                }
            }
            set
            {
                if(kinematic)
                {
                    kinematicVelocity = value;
                }
                else
                {
                    Debug.LogError("Joint is dynamic. Cannot set velocity.");
                }
            }
        }

        /// <summary>
        /// Current joint acceleration.
        /// 
        /// In kinematic mode this field is mutable but has no effect on the physics.
        /// </summary>
        /// <value>Acceleration [rad/s^2]</value>
        public float acceleration
        {
            get
            {
                if(kinematic)
                {
                    return kinematicAcceleration;
                }
                else
                {
                    return body.jointAcceleration[0];
                }
            }    
            set
            {
                if(kinematic)
                {
                    kinematicAcceleration = value;
                }
                else
                {
                    Debug.LogError("Joint is dynamic. Cannot set acceleration.");
                }
            }
        }


        /// <summary>
        /// Joint force for prismatic joints, joint angle for revolute joints.
        /// 
        /// In kinematic mode this field is mutable but has no effect on the physics.
        /// </summary>
        /// <value></value>
        public float tau
        {
            get
            {
                if(kinematic)
                {
                    return kinematicTau;
                }
                else
                {
                    return body.jointForce[0];
                }
            }
            set
            {
                if(kinematic)
                {
                    kinematicTau = value;
                }
                else
                {
                    body.jointForce = new ArticulationReducedSpace(value);
                }
            }
        }

        public float stiffness
        {
            set
            {
                if(!kinematic)
                {
                    var xDrive = body.xDrive;
                    xDrive.stiffness = value;
                    body.xDrive = xDrive;
                }
            }
        }

        public float damping
        {
            set
            {
                if(!kinematic)
                {
                    var xDrive = body.xDrive;
                    xDrive.damping = value;
                    body.xDrive = xDrive;
                }
            }
        }

        public float desiredPosition
        {
            set
            {
                if(!kinematic)
                {
                    var xDrive = body.xDrive;
                    xDrive.target = value;
                    body.xDrive = xDrive;
                }
            }
        }

        public float desiredVelocity
        {
            set
            {
                if(!kinematic)
                {
                    var xDrive = body.xDrive;
                    xDrive.targetVelocity = value;
                    body.xDrive = xDrive;
                }
            }
        }


        private float kinematicPosition = 0;
        private float kinematicVelocity = 0;

        private float kinematicAcceleration = 0;
        private float kinematicTau = 0;

        void Awake()
        {
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            if(kinematic)
            {
                setPosition(position);
            }
            
        }

        protected abstract void setPosition(float position);

    }

}