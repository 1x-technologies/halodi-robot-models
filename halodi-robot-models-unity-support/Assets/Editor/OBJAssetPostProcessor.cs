/*
Copyright 2019 Halodi Robotics AS
Author: Jesper Smith (jesper@halodi.com)
Co-author: Daniel Ervik-Børnich (daniel@halodi.com)

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

using System;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace Halodi.RobotModels
{
    public class OBJAssetPostProcessor : AssetPostprocessor
    {
        private bool isObj;

        public void OnPreprocessModel()
        {
            ModelImporter modelImporter = assetImporter as ModelImporter;

            modelImporter.materialImportMode = ModelImporterMaterialImportMode.None; // Don't import materials from objects

            isObj = Path.GetExtension(modelImporter.assetPath).ToLowerInvariant() == ".obj";

            if (!isObj)
                return;
        }
        

        public void OnPostprocessModel(GameObject gameObject)
        {
            if (!isObj)
                return;
                

            gameObject.transform.SetPositionAndRotation(
                getPositionFix(gameObject.transform.position),
                Quaternion.Euler(getRotationFix()) * gameObject.transform.rotation);

            setupHDPRMaterial(gameObject);

            addFaceplate(gameObject);
        }

        public Vector3 getPositionFix(Vector3 position)
        {
            return new Vector3(-position.z, position.y, -position.x);
        }

        
        private static Vector3 getRotationFix()
        {
            return new Vector3(-90, 90, 0);
        }

        private void setupHDPRMaterial(GameObject gameObject)
        {
            gameObject.GetComponentInChildren<MeshRenderer>().material = Resources.Load("Materials/eve_r3/" + gameObject.name, typeof(Material)) as Material;  // Attach custom HDRP materials to models based on name
            return;
        }

        private void addFaceplate(GameObject gameObject)
        {
            if(gameObject.name == "head")
            {
                GameObject facePlatePrefab = Resources.Load("Models/eve_r3/head_faceplate", typeof(GameObject)) as GameObject;
                GameObject newFacePlateGO = GameObject.Instantiate(facePlatePrefab) as GameObject;
                newFacePlateGO.transform.SetParent(gameObject.transform, true);
            }
            return;
        }
    }
}
