using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using RosSharp.Urdf.Editor;
using UnityEditor;
using UnityEngine;

namespace Halodi.Models.Editor
{
    /// <summary>
    /// This post processor writes all meshes to file, so we can make a prefab.
    /// 
    /// If a mesh with the same name can be found as the object, it'll get replaced with the object instance. This fixes some weird bug.
    /// </summary>
    public class ExportMeshToFilePostProcessor
    {
        private static string MeshFolderName = "Meshes";

        internal static void PostProccess(string robotName, GameObject root, string urdfFilename)
        {

            string packageRoot = UrdfAssetPathHandler.GetRelativeAssetPath(Path.GetDirectoryName(urdfFilename));
            EmptyMeshFolder(packageRoot, robotName);


            MeshCollider[] colliders = root.GetComponentsInChildren<MeshCollider>();
            foreach (var collider in colliders)
            {
                Mesh mesh = collider.sharedMesh;
                if (String.IsNullOrEmpty(AssetDatabase.GetAssetPath(mesh)))
                {
                    collider.sharedMesh = MakeMeshExportable(robotName, collider.gameObject, mesh, packageRoot);
                }
            }

            MeshFilter[] meshFilters = root.GetComponentsInChildren<MeshFilter>();
            foreach (var meshFilter in meshFilters)
            {
                Mesh mesh = meshFilter.sharedMesh;
                if (String.IsNullOrEmpty(AssetDatabase.GetAssetPath(mesh)))
                {
                    meshFilter.sharedMesh = MakeMeshExportable(robotName, meshFilter.gameObject, mesh, packageRoot);
                }
            }
        }

        private static void EmptyMeshFolder(string packageRoot, string robotName)
        {
            string meshFolder = Path.Combine(packageRoot, MeshFolderName);
            if (!AssetDatabase.IsValidFolder(meshFolder))
            {
                AssetDatabase.CreateFolder(packageRoot, MeshFolderName);
            }

            string meshRobotFolder = Path.Combine(meshFolder, robotName);
            if (!AssetDatabase.IsValidFolder(meshRobotFolder))
            {
                AssetDatabase.CreateFolder(meshFolder, robotName);
            }

            DirectoryInfo info = new DirectoryInfo(Path.Combine(Application.dataPath, "..", meshRobotFolder));

            AssetDatabase.StartAssetEditing();
            try
            {
                foreach (var file in info.GetFiles())
                {
                    file.Delete();
                }

                File.WriteAllText(Path.Combine(info.FullName, ".gitkeep"), "Force git to checkin this directory. Avoid dangling meta files");


            }
            finally
            {
                AssetDatabase.StopAssetEditing();
                AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
            }




        }

        private static Mesh MakeMeshExportable(String robotName, GameObject parent, Mesh mesh, String packageRoot)
        {
            if (!String.IsNullOrEmpty(mesh.name))
            {
                String[] possibleMeshes = AssetDatabase.FindAssets(mesh.name, new string[] { packageRoot });
                if (possibleMeshes.Length > 0)
                {
                    // Debug.LogWarning("Replacing mesh " + mesh.name + " with " + AssetDatabase.GUIDToAssetPath(possibleMeshes[0]));
                    // return AssetDatabase.LoadAssetAtPath<Mesh>(AssetDatabase.GUIDToAssetPath(possibleMeshes[0]));

                    throw new System.Exception("Import of " + robotName + " failed. Found mesh with name " + mesh.name + " in assets, but is not linked. Restart Unity.");
                }
            }

            string meshFolder = Path.Combine(packageRoot, MeshFolderName, robotName);

            String newName = AssetDatabase.GenerateUniqueAssetPath(Path.Combine(meshFolder, CreateMeshName(parent, mesh) + ".asset"));
            AssetDatabase.CreateAsset(mesh, newName);
            return mesh;
        }



        private static string CreateMeshName(GameObject parent, Mesh mesh)
        {
            if (String.IsNullOrEmpty(mesh.name))
            {
                return parent.name;
            }
            else
            {
                return mesh.name;
            }
        }
    }
}