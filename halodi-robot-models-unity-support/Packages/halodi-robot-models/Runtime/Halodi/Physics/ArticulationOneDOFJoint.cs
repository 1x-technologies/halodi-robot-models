using System.Collections;
using System.Collections.Generic;
using Halodi.Physics.Interfaces;
using UnityEngine;

namespace Halodi.Physics
{
    public abstract class ArticulationOneDOFJoint: ArticulationJoint
    {
        public override bool isRoot => false;

        public Vector3 axis;

        public float lowerLimit;
        public float upperLimit;
        
        public override IJointPhysics physics => oneDOFJointPhysics;


        public IOneDOFJointPhysics oneDOFJointPhysics = null;

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
                    return oneDOFJointPhysics.jointPosition;// body.jointPosition[0];
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
                    oneDOFJointPhysics.jointPosition = value;// new ArticulationReducedSpace(value);
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
                    return  kinematicVelocity;
                }
                else
                {
                    return oneDOFJointPhysics.jointVelocity; 
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
                    oneDOFJointPhysics.jointVelocity = value;
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
                    return oneDOFJointPhysics.jointAcceleration;
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
                    oneDOFJointPhysics.jointAcceleration = value;
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
                    return oneDOFJointPhysics.jointEffort;
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
                  oneDOFJointPhysics.jointEffort = value;
                }
            }
        }

        public float stiffness
        {
            set
            {
                if(!kinematic)
                {
                    oneDOFJointPhysics.stiffness = value;
                }
            }
        }

        public float damping
        {
            set
            {
                if(!kinematic)
                {
                    oneDOFJointPhysics.damping = value;
                }
            }
        }

        public float desiredPosition
        {
            set
            {
                if(!kinematic)
                {
                    oneDOFJointPhysics.desiredPosition = value;
                }
            }
        }

        public float desiredVelocity
        {
            set
            {
                if(!kinematic)
                {
                    oneDOFJointPhysics.desiredVelocity = value;
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
        void LateUpdate()
        {
            if(kinematic)
            {
                SetKinematicPosition(position);
            }
            
        }

        public abstract void SetKinematicPosition(float position);

    }

}