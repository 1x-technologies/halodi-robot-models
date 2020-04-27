using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Halodi.Physics
{
    public class ArticulationFixedJoint : ArticulationJoint
    {
        public override bool isRoot => false;
       
        protected override void SetupJoint(ArticulationBody articulationBody)
        {
            articulationBody.jointType = ArticulationJointType.FixedJoint;
        }

    }

}