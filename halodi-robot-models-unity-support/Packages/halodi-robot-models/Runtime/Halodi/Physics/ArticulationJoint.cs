using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Halodi.Physics
{
    /// <summary>
    /// This class holds all data for a articulated body. 
    /// 
    /// This allows switching between dynamic and kinematic mode by removing the ArticulationBody from the tree. This is more robust than disabling the body.
    ///
    /// 
    /// </summary>
    public abstract class ArticulationJoint : MonoBehaviour
    {
        private static float MinimumMass = 0.02f;

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
            set
            {
                if(value)
                {
                    SetKinematic();
                }
                else
                {
                    SetDynamic();
                }
            }
            get
            {
                // If not kinematic, see if body is set (ArticulationBody is attached). If not, set kinematic_ to true to avoid unneccessary GetComponent calls.
                if(!kinematic_)
                {
                    kinematic_ = body == null;
                }

                return kinematic_;
            }        
        }

        public bool dynamic
        {
            set
            {
                kinematic = !value;
            }
            get
            {
                return !kinematic;
            }
        }

        
        private ArticulationBody body_ = null;
        public ArticulationBody body 
        {
            get
            {
                if(body_ == null)
                {
                    body_ = GetComponent<ArticulationBody>();
                }

                return body_;
            }
        }

        

        // Start is called before the first frame update
        public void Start()
        {
            UpdateRigidBodyData();
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

        public void UpdateRigidBodyData()
        {
            if(!kinematic)
            {
                body.useGravity = true;


                // Disable damping and friction
                body.linearDamping = 0;
                body.angularDamping = 0;
                body.jointFriction = 0;

                if(mass > MinimumMass)
                {
                    body.mass = mass;
                    body.centerOfMass = centerOfMass;
                    body.inertiaTensor = inertiaTensor;
                    body.inertiaTensorRotation = inertiaTensorRotation;
                }
                else
                {
                    body.mass = MinimumMass;
                    body.centerOfMass = centerOfMass;
                    body.inertiaTensor = (MinimumMass/mass) * inertiaTensor;
                    body.inertiaTensorRotation = inertiaTensorRotation;
                }

                body.anchorPosition = anchorPosition;
                body.anchorRotation = anchorRotation;
                body.parentAnchorPosition = parentAnchorPosition;
                body.parentAnchorRotation = parentAnchorRotation;
            }
        }

        private void SetKinematic()
        {
            SetChildrenKinematic(gameObject, true);

            if(body == null)
            {
                return;
            }

            if(Application.isPlaying)
            {
                Object.Destroy(body);
            }
            else
            {
                Object.DestroyImmediate(body);
            }

            body_ = null;
            kinematic_ = true;
        }

        private void SetDynamic()
        {
            if(body == null)
            {
                body_ = gameObject.AddComponent<ArticulationBody>();
                kinematic_ = false;
            }

            if(isRoot != body.isRoot)
            {
                SetKinematic();
                Debug.LogError("Cannot enable dynamic mode for " + JointName + " without enabling dyanmic mode on parent.");
                return;
            }

            
            UpdateRigidBodyData();
            SetupJoint(body);

            // Enable dynamic on all childeren
            SetChildrenKinematic(gameObject, false);
        }


        private void SetChildrenKinematic(GameObject next, bool kinematic)
        {
            foreach(Transform childTransform in next.transform)
            {
                ArticulationJoint childJoint = childTransform.gameObject.GetComponent<ArticulationJoint>();
                if(childJoint != null)
                {
                    childJoint.kinematic = kinematic;
                }
                else
                {
                    SetChildrenKinematic(childTransform.gameObject, kinematic);
                }
            }

        }

        protected abstract void SetupJoint(ArticulationBody body);

    }
}
