/*
Copyright 2019 Halodi Robotics AS
Author: Jesper Smith (jesper@halodi.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
using UnityEditor;
using UnityEngine;
using System.IO;
using System;

namespace Halodi.Models.Editor
{
    internal class ImportModels
    {


        internal static readonly string PackageDirectory = Path.Combine(new String[] { Application.dataPath, "..", "Packages", "halodi-robot-models", "Runtime", "Halodi", "Models" });
        internal static readonly string MainProjectPath = new DirectoryInfo(Application.dataPath).Parent.Parent.FullName;


        [MenuItem("Halodi/Update Halodi Robot Models")] //creates a new menu tab
        internal static void EditPackageConfiguration()
        {
            string MainProjectPath = new DirectoryInfo(Application.dataPath).Parent.Parent.FullName;

            ImportHalodiRobotModels.ImportRobotModels(MainProjectPath, PackageDirectory);
        }

    }
}
