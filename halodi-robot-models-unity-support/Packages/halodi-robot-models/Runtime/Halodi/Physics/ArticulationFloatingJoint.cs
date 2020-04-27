using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Halodi.Physics
{
    public class ArticulationFloatingJoint : ArticulationJoint
    {
        public override bool isRoot => true;
        protected override void SetupJoint(ArticulationBody body)
        {
            
        }
    }
}