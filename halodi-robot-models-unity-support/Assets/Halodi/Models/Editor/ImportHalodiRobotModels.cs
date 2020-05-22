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
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;

namespace Halodi.Models.Editor
{
    internal class ImportHalodiRobotModels
    {

        internal static readonly string PackageXML = "package.xml";


        internal static readonly string PackageDirectory = Path.Combine(new String[] { Application.dataPath, "..", "Packages", "halodi-robot-models", "Runtime", "Halodi", "Models" });

        internal static readonly string TargetDirectory = Path.Combine(new string[] { Application.dataPath, "Temp", "Models" });

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
                foreach (string directory in directories)
                {
                    string PackageDescription = Path.Combine(directory, PackageXML);

                    if (File.Exists(PackageDescription))
                    {
                        ImportModel(directory);
                    }
                };

                foreach (string file in Directory.GetFiles(TargetDirectory))
                {
                    FileInfo info = new FileInfo(file);
                    if (Path.GetExtension(file).ToLowerInvariant().Equals(".urdf"))
                    {
                        LoadURDF(file);
                    }
                }

            }
            finally
            {

                // Cleanup and move assets back to package
                MoveAssetToPackageDirectory();
            }

        }

        private static void MoveDirectory(string source, string target)
        {
            Directory.Move(source, target);

            string metaFileSource = source.TrimEnd(new[] { '/', '\\' }) + ".meta";
            string metaFileTarget = target.TrimEnd(new[] { '/', '\\' }) + ".meta";

            if (File.Exists(metaFileSource))
            {
                File.Delete(metaFileTarget);
                File.Move(metaFileSource, metaFileTarget);
            }
        }

        private static void MovePackageToAssetDirectory()
        {
            if (Directory.Exists(TargetDirectory))
            {
                throw new IOException(TargetDirectory + " already exists. This probably means a previous import has failed. Revert the changes in your git repository, remove this directory and try again.");
            }

            try
            {
                AssetDatabase.StartAssetEditing();
                // Create parent of temp directory
                string parent = Path.Combine(TargetDirectory, "..");
                if (!Directory.Exists(parent))
                {
                    Directory.CreateDirectory(parent);
                }




                if (Directory.Exists(PackageDirectory))
                {
                    MoveDirectory(PackageDirectory, TargetDirectory);
                }
                else
                {
                    Directory.CreateDirectory(TargetDirectory);
                }
            }
            finally
            {
                AssetDatabase.StopAssetEditing();
                AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
            }
        }

        private static void MoveAssetToPackageDirectory()
        {
            try
            {
                AssetDatabase.StartAssetEditing();
                // Make sure parent exists
                Directory.CreateDirectory(Path.Combine(PackageDirectory, ".."));
                AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);


                if (Directory.Exists(TargetDirectory))
                {
                    MoveDirectory(TargetDirectory, PackageDirectory);
                }

            }
            finally
            {
                AssetDatabase.StopAssetEditing();
                AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
            }

        }

        private static string RelativeToAssetDatabase(string path)
        {
            if (path.StartsWith(AssetDatabaseRoot))
            {
                if (AssetDatabaseRoot.EndsWith(Path.DirectorySeparatorChar.ToString()))
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


            // Create a new scene to keep things clean
            Selection.activeGameObject = null;  // Deselect gameobject to avoid crap
            Scene tempScene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Additive);

            try
            {
                UrdfRobotExtensions.Create(URDF);

                UrdfRobot robot = Selection.activeGameObject.GetComponent<UrdfRobot>();
                if (robot == null)
                {
                    throw new Exception("Cannot instantiate URDF");
                }

                string PrefabName = robot.name;
                GameObject rootObject = robot.gameObject;
                RobotModelPostProcessorRunner.PostProcess(PrefabName, rootObject, URDF);


                string AssetTarget = RelativeToAssetDatabase(TargetDirectory);

                string PrefabAsset = Path.Combine(AssetTarget, PrefabName + ".prefab");

                
                bool success;
                GameObject prefab = PrefabUtility.SaveAsPrefabAssetAndConnect(rootObject, PrefabAsset, InteractionMode.AutomatedAction, out success);

                if (!success)
                {
                    throw new Exception("Cannot create prefab of " + URDF);
                }
            }
            catch(Exception e)
            {
                Debug.LogWarning("Cannot import " + URDF + ". Skipping. \n" + e.Message);
                
            }
            finally
            {
                EditorSceneManager.CloseScene(tempScene, true);
            }

        }


        private static void ImportModel(string source)
        {
            try
            {
                AssetDatabase.StartAssetEditing();

                CopyDirectory(source, TargetDirectory, IgnoredFiles);

                string[] urdfModels = Directory.GetFiles(source, "*.urdf", SearchOption.AllDirectories);

                foreach (string urdfModel in urdfModels)
                {
                    if (!urdfModel.EndsWith(InputURDFExtension))
                    {
                        string URDF = ImportFile(urdfModel, TargetDirectory);
                    }
                }
            }
            finally
            {
                AssetDatabase.StopAssetEditing();
                AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
            }

        }

        private static void CopyDirectory(string source, string target, string[] Ignored)
        {
            string SourceName = Path.GetFileName(source);

            string TargetPath = Path.Combine(target, SourceName);
            if (!Directory.Exists(TargetPath))
            {
                AssetDatabase.CreateFolder(RelativeToAssetDatabase(target), SourceName);
            }

            string[] files = Directory.GetFiles(source);
            foreach (string file in files)
            {
                if (!IgnoreFile(file, Ignored))
                {
                    ImportFile(file, TargetPath);
                }
            }

            string[] directories = Directory.GetDirectories(source);
            foreach (string directory in directories)
            {
                if (!IgnoreFile(directory, Ignored))
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

            return TargetFile;
        }
    }
}
