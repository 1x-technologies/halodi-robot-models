using System.Collections;
using System.Collections.Generic;
using Halodi.Models.Editor.Articulation;
using RosSharp.Urdf;
using UnityEditor;
using UnityEngine;

namespace Halodi.Models.Editor
{
    /// <summary>
    /// Helper class called for each robot that has been imported.
    /// 
    /// Allows changing the robots model before saving as a prefab.
    /// </summary>
    public class RobotModelPostProcessorRunner
    {
        internal static void PostProcess(string robotName, GameObject robotRoot, string urdfFilename)
        {
            ArticulationPostProcessor.PostProcess(robotName, robotRoot);
            RemoveRosSharpFromModelPostProccessor.PostProccess(robotName, robotRoot);
            ExportMeshToFilePostProcessor.PostProccess(robotName, robotRoot, urdfFilename);
        }
    }
}