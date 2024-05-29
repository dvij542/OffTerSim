using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace danilec
{
    public class PrepareBushOrGrassMesh : EditorWindow
    {
        static GameObject obj;
        private GameObject refObj;
        private Mesh currentMeshCopy;
        private Mesh originalMesh;

        private float bendMultiplier = 0.7f;
        private float groundYOffset = 0f;
        private float refGroundOffset = 0f;

        [MenuItem("Tools/Prepare detail for bend")]
        static void Init()
        {
            PrepareBushOrGrassMesh window = (PrepareBushOrGrassMesh)GetWindow(typeof(PrepareBushOrGrassMesh));
        }
        private void clearData()
        {
            refObj = null;
            currentMeshCopy = null;
            originalMesh = null;

            bendMultiplier = 0.7f;
            groundYOffset = 0f;
            refGroundOffset = 0f;
        }
        private void Update()
        {
            if(obj || refObj)
                if (obj != refObj)
                {
                    clearData();
                    refObj = obj;
                }
        }
        private void OnDestroy()
        {
            clearData();
        }

        void OnGUI()
        {
            obj = (GameObject)EditorGUILayout.ObjectField(obj, typeof(GameObject), true);

            GUILayout.Space(20);
            GUILayout.Label("Bend potential: " + (float)Math.Round(bendMultiplier * 100) + "%");
            bendMultiplier = GUILayout.HorizontalScrollbar(bendMultiplier, 0.1f, 0.0f, 1.1f);

            GUILayout.Space(20);
            groundYOffset = EditorGUILayout.FloatField("Ground Y offset:", groundYOffset);
            if(refGroundOffset != groundYOffset)
            {
                if (obj == null)
                    return;
                if (obj.GetComponent<MeshFilter>() == null)
                    return;

                var btnStyle0 = new GUIStyle(GUI.skin.button);
                btnStyle0.fontSize = 18;
                GUILayout.Space(20);
                if (GUILayout.Button("Preview", btnStyle0))
                {
                    refGroundOffset = groundYOffset;
                    viewInitMeshData();
                    preview();
                }
            }
            

            var btnStyle1 = new GUIStyle(GUI.skin.button);
            btnStyle1.fontSize = 18;
            GUILayout.Space(20);
            if (GUILayout.Button("Save", btnStyle1))
            {
                clearData();
                if (obj == null)
                {
                    ShowNotification(new GUIContent("Assign a GameObject from the scene"), 1);
                    return;
                }
                if (obj.GetComponent<MeshFilter>() == null)
                {
                    ShowNotification(new GUIContent("No mesh found in the object"), 1);
                    return;
                }
                refObj = obj;


                viewInitMeshData();
                saveMesh();
            }

            if (originalMesh == null)
                return;

            var btnStyle2 = new GUIStyle(GUI.skin.button);
            btnStyle2.fontSize = 18;
            GUILayout.Space(20);
            if (GUILayout.Button("Revert and stop", btnStyle2))
            {
                obj.GetComponent<MeshFilter>().mesh = originalMesh;

                clearData();
                originalMesh = null;
            }
        }

        private void viewInitMeshData()
        {
            MeshFilter mf = obj.GetComponent<MeshFilter>();
            string name = obj.name;//mf.sharedMesh.name;
            currentMeshCopy = Instantiate(mf.sharedMesh) as Mesh;
            //bgb_ - bush grass bend
            currentMeshCopy.name = ((mf.sharedMesh.name.Substring(0, 4).CompareTo("bgb_") == 0) ? "" : "bgb_") + name;

            if (originalMesh == null)
                originalMesh = obj.GetComponent<MeshFilter>().sharedMesh;
        }

        private void preview()
        {
            Vector3[] vertices = originalMesh.vertices;
            int vCount = vertices.Length;
            for (int i = 0; i < vCount; i++)
            {
                vertices[i].y = vertices[i].y + groundYOffset;
            }
            currentMeshCopy.SetVertices(vertices);
            obj.GetComponent<MeshFilter>().mesh = currentMeshCopy;

        }
        private void saveMesh()
        {
            Vector3[] vertices = originalMesh.vertices;
            int vCount = vertices.Length;

            for (int i = 0; i < vCount; i++)
            {
                vertices[i].y = vertices[i].y + groundYOffset;
            }
            currentMeshCopy.SetVertices(vertices);


            float minY = vertices[0].y, maxY = minY;

            for (int i = 1; i < vCount; i++)
            {
                Vector3 vPos = obj.transform.TransformPoint(vertices[i]);
                if (vPos.y < minY)
                    minY = vPos.y;
                if (vPos.y > maxY)
                    maxY = vPos.y;
            }
            if (minY < 0)
                minY = 0;

            Color[] colors = currentMeshCopy.colors;

            if(colors.Length > 0)
            {
                for (int i = 0; i < vCount; i++)
                {
                    Vector3 vPos = obj.transform.TransformPoint(vertices[i]);
                    if (vPos.y < minY)
                        colors[i].a = 0;
                    else
                        colors[i].a = (vPos.y - minY) / (maxY - minY) * bendMultiplier;
                }
                currentMeshCopy.SetColors(colors);
            }
           
            if(colors.Length == 0)
            {
                colors = new Color[vCount];
                for (int i = 0; i < vCount; i++)
                {
                    colors[i] = new Color(1, 1, 1, 1);
                    Vector3 pos = vertices[i];
                    colors[i].a = (pos.y - minY) / (maxY - minY) * bendMultiplier;
                }
                currentMeshCopy.SetColors(colors);
            }

            

            string path = EditorUtility.SaveFilePanelInProject("Choose location for a new Mesh to save", currentMeshCopy.name, "asset", "Save mesh");
            AssetDatabase.CreateAsset(currentMeshCopy, path);
            AssetDatabase.Refresh();
            obj.GetComponent<MeshFilter>().mesh = AssetDatabase.LoadAssetAtPath(path, typeof(Mesh)) as Mesh;
        }
    }
}
