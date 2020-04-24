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
using RosSharp.Urdf.Editor;
using RosSharp.Urdf;

namespace Halodi.RobotModels
{
    internal class ImportHalodiRobotModels
    {

        internal static readonly string PackageXML = "package.xml";


        internal static readonly string PackageDirectory = Path.Combine(new String[] {Application.dataPath, "..", "Packages", "halodi-robot-models", "Runtime", "halodi", "models"});
        internal static readonly string TargetDirectory = Path.Combine(new string[] {Application.dataPath, "Temp", "models"});

        internal static readonly string AssetDatabaseRoot = new DirectoryInfo(Application.dataPath).Parent.FullName;

        internal static readonly string[] IgnoredFiles = { "build", "cmake", "CMakeLists.txt", "package.xml", "model.config", "dummy.urdf", "urdf.in", "urdf", "sdf" };

        internal static readonly string InputURDFExtension = ".in.urdf";


        [MenuItem("Halodi/Update Halodi Robot Models")] //creates a new menu tab
        internal static void EditPackageConfiguration()
        {
            // The URDF importer assumes everythings lives in Assets. Move existing to Assets to maintain GUID's first.
            MovePackageToAssetDirectory();
            

            try
            {
                string MainProjectPath = new DirectoryInfo(Application.dataPath).Parent.Parent.FullName;

                string[] directories = Directory.GetDirectories(MainProjectPath);
                foreach(string directory in directories)
                {
                    string PackageDescription = Path.Combine(directory, PackageXML);

                    if(File.Exists(PackageDescription))
                    {
                        ImportModel(directory);
                    }
                };

                foreach(string file in Directory.GetFiles(TargetDirectory))
                {
                    FileInfo info = new FileInfo(file);
                    if(Path.GetExtension(file).ToLowerInvariant().Equals(".urdf"))
                    {
                            LoadURDF(file);                   
                    }
                }

            }
            finally
            {

            }

        }

        private static void MovePackageToAssetDirectory()
        {
            if(Directory.Exists(TargetDirectory))
            {
                throw new IOException(TargetDirectory + " already exists. This probably means a previous import has failed. Revert the changes in your git repository, remove this directory and try again.");
            }

            // Create parent of temp directory
            Directory.CreateDirectory(Path.Combine(TargetDirectory, ".."));


            if(Directory.Exists(PackageDirectory))
            {
                Debug.Log("[Pre-import] Moving " + PackageDirectory + " to " + TargetDirectory);
                Directory.Move(PackageDirectory, TargetDirectory);
            }
            else
            {
                Directory.CreateDirectory(TargetDirectory);
            }

            AssetDatabase.Refresh();
        }

        private static void MoveAssetToPackageDirectory()
        {
            // Make sure parent exists
            Directory.CreateDirectory(Path.Combine(PackageDirectory, ".."));

            if(Directory.Exists(TargetDirectory))
            {
                Debug.Log("[Post-import] Moving " + TargetDirectory + " to " + PackageDirectory);
                Directory.Move(TargetDirectory, PackageDirectory);
            }
        }

        private static string RelativeToAssetDatabase(string path)
        {
            if(path.StartsWith(AssetDatabaseRoot))
            {
                if(AssetDatabaseRoot.EndsWith(Path.DirectorySeparatorChar.ToString()))
                {
                    return path.Substring(AssetDatabaseRoot.Length);
                }
                else
                {
                    return path.Substring(AssetDatabaseRoot.Length + 1);
                }
            }
            else
            {
                throw new Exception("Not an asset database path: " + path);
            }
        }

        private static bool IgnoreFile(string path, string[] Ignored)
        {
            string name = new DirectoryInfo(path).Name;
            return Array.Exists(Ignored, e => e.Equals(name));
        }

        private static void LoadURDF(string URDF)
        {
            try
            {
                UrdfRobotExtensions.Create(URDF);
            }
            catch
            {
                Debug.LogWarning("Cannot import " + URDF + ". Skipping");

                if(Selection.activeGameObject != null)
                {
                    if(Selection.activeGameObject.GetComponent<UrdfRobot>() != null)
                    {
                        GameObject.DestroyImmediate(Selection.activeGameObject);
                    }
                }

                return;
            }
            UrdfRobot robot = Selection.activeGameObject.GetComponent<UrdfRobot>();
            if(robot == null)
            {
                throw new Exception("Cannot instantiate URDF");
            }
            robot.SetRigidbodiesIsKinematic(true);
            robot.SetUseUrdfInertiaData(true);
            robot.SetRigidbodiesUseGravity(false);

            string AssetTarget = RelativeToAssetDatabase(TargetDirectory);

            string PrefabName = robot.name;
            string PrefabAsset = Path.Combine(AssetTarget, PrefabName + ".prefab");

            

            bool success;
            PrefabUtility.SaveAsPrefabAsset(robot.gameObject, PrefabAsset, out success);
            
            if(!success)
            {
                throw new Exception("Cannot create prefab of " + URDF);
            }

             GameObject.DestroyImmediate(robot.gameObject);
        }


        private static void ImportModel(string source)
        {
            CopyDirectory(source, TargetDirectory, IgnoredFiles);

            string[] urdfModels = Directory.GetFiles(source, "*.urdf", SearchOption.AllDirectories);

            foreach(string urdfModel in urdfModels)
            {
                if(!urdfModel.EndsWith(InputURDFExtension))
                {
                    string URDF = ImportFile(urdfModel, TargetDirectory);
                }
            }

        }

        private static void CopyDirectory(string source, string target, string[] Ignored)
        {
            string SourceName = Path.GetFileName(source);

            string TargetPath = Path.Combine(target, SourceName);
            if(!Directory.Exists(TargetPath))
            {
                AssetDatabase.CreateFolder(RelativeToAssetDatabase(target), SourceName);
            }

            string[] files = Directory.GetFiles(source);
            foreach(string file in files)
            {
                if(!Path.GetExtension(file).ToLowerInvariant().Equals(".obj"))
                {
                    if(!IgnoreFile(file, Ignored))
                    {
                        ImportFile(file, TargetPath);
                    }
                }
            }

            // Defer copying and loading of .obj files till after the textures are imported
            foreach(string file in files)
            {
                if(Path.GetExtension(file).ToLowerInvariant().Equals(".obj"))
                {
                    ImportFile(file, TargetPath);
                }
            }


            string[] directories = Directory.GetDirectories(source);
            foreach(string directory in directories)
            {
                if(!IgnoreFile(directory, Ignored))
                {
                    CopyDirectory(directory, TargetPath, new string[0]);
                }
            }
        }

        private static string ImportFile(string file, string target)
        {
            string FileName = new FileInfo(file).Name;
            string TargetFile = Path.Combine(target, FileName);
            string AssetPath = RelativeToAssetDatabase(TargetFile);
            
            File.Copy(file, TargetFile, true);
            AssetDatabase.ImportAsset(AssetPath);

            return TargetFile;
        }
    }
}
