using System.Collections;
using System.Collections.Generic;
using Halodi.Physics.Interfaces;
using UnityEngine;


namespace Halodi.Physics
{
    /// <summary>
    /// This class holds all data for a articulated body. 
    /// 
    /// </summary>
    public abstract class ArticulationJoint: MonoBehaviour
    {

        /// <summary>
        /// Name of this joint.
        ///
        /// 
        /// If imported from an URDF, it is equal to the joint name in the URDF.
        /// </summary>
        public string JointName;

        
        public float mass;

        public Vector3 centerOfMass;
        public Vector3 inertiaTensor;
        public Quaternion inertiaTensorRotation;

        public Vector3 anchorPosition;
        public Quaternion anchorRotation;

        public bool computeParentAnchor = true;

        public Vector3 parentAnchorPosition;
        public Quaternion parentAnchorRotation;

        
        public abstract bool isRoot
        {
            get;
        }

        private bool kinematic_;

        /// <summary>
        /// Kinematic is true if the ArticulationBody is disabled.
        /// </summary>
        /// <value>True if the articulation body is disabled, false if enabled.</value>
        public bool kinematic
        {
            get
            {
                // If not kinematic, see if body is set (ArticulationBody is attached). If not, set kinematic_ to true to avoid unneccessary GetComponent calls.
                if(!kinematic_)
                {
                    kinematic_ = physics == null;
                }

                return kinematic_;
            }        
        }

        public abstract IJointPhysics physics
        {
             get;
        }

        public bool dynamic
        {
            get
            {
                return !kinematic;
            }
        }

        
        public IPhysicsEngine PhysicsEngine
        {
            set 
            {
                SetPhysics(value);
                SetChilderenPhysicsEngine(gameObject, value);
            }
        }

        private static void SetChilderenPhysicsEngine(GameObject next, IPhysicsEngine physicsEngine)
        {
            foreach(Transform childTransform in next.transform)
            {
                ArticulationJoint childJoint = childTransform.gameObject.GetComponent<ArticulationJoint>();
                if(childJoint != null)
                {
                    childJoint.PhysicsEngine = physicsEngine;
                }
                else
                {
                    SetChilderenPhysicsEngine(childTransform.gameObject, physicsEngine);
                }
            }
        }
        

        // Start is called before the first frame update
        public void Start()
        {
        }


        public void OnValidate()
        {
            if(Application.isEditor)
            {
                if(computeParentAnchor)
                {
                    UpdateParentAnchor();
                }
            }
        }

        public void UpdateParentAnchor()
        {
            computeParentAnchor = true;
            if(!isRoot)
            {
                Vector3 anchorWorldPosition = transform.TransformPoint(anchorPosition);
                parentAnchorPosition = transform.parent.InverseTransformPoint(anchorWorldPosition);

                Quaternion anchorWorldRotation = transform.rotation * anchorRotation;
                parentAnchorRotation = Quaternion.Inverse(transform.parent.rotation) * anchorWorldRotation;
            }
        }
        
        protected abstract void SetPhysics(IPhysicsEngine physics);


    }
}
